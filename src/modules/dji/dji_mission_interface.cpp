#include <cmath>
#include <vector>
#include <dji_vehicle.hpp>
#include <dji_linux_helpers.hpp>

#include "dji_camera_gimbal_def.h"
#include "dji_mission_interface.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

static std::vector<WayPointSettings> wp_list;
static WayPointFinishData wayPointFinishData;
static WayPointFinishData eventWayPointFinishData;
static VehicleCallBackHandler missionHandler;

static int responseTimeout = 1;
static const int DEFAULT_PACKAGE_INDEX = 0;

//-------------------------------------------------------------
// Static funcitions
//-------------------------------------------------------------

static void
setWaypointDefaults(WayPointSettings* wp);
static void
setWaypointInitDefaults(WayPointInitSettings* fdata);
static void
setGimbalAngle(Vehicle* vehicle, GimbalContainer* gimbal);

EResult 
cmdMissionCallback(EVehicleCmd cmd, void* data, size_t len) {
    EResult ret = EResult::MSP_FAILED; 
    switch (cmd)
    {
        case EVehicleCmd::MSP_CMD_READ_STATE:
            ret = handleStateRequest();
            break;
        case EVehicleCmd::MSP_CMD_UPLOAD_WAY_POINTS:
            createWaypoints();
            ret = EResult::MSP_SUCCESS;
            break;
        case EVehicleCmd::MSP_CMD_MISSION_START:
            ret = uploadWaypoints();
            if (ret == EResult::MSP_SUCCESS){
                ret = runWaypointMission();
            }
            break;
        case EVehicleCmd::MSP_CMD_MISSION_PAUSE:
            ret = pauseWaypointMission();
            break;
        case EVehicleCmd::MSP_CMD_MISSION_RESUME:
            ret = resumeWaypointMission();
            break;
        case EVehicleCmd::MSP_CMD_TAKE_PICTURE:
            ret = takePicture(data);
        default:
            break;
    }
    return ret;
}

EResult 
handleStateRequest() {

    vehicleStateData_t data = { };
    if (vehicle) {
        data.state |= EVehicleState::MSP_VHC_AVAILABLE;
    }

    MspController::getInstance()->vehicleNotification(EVehicleNotification::MSP_VHC_STATE, &data);
}


void wayPointEventCallback(Vehicle* vehicle, RecvContainer recvFrame, UserData userData) {
    waypointReachedData_t wpdata;

    if (recvFrame.recvData.wayPointStatusPushData.current_status == 6) {
        spdlog::debug("event way point reached");

        // Current waypoint data
        ACK::WayPointReachedData wpReachedData = recvFrame.recvData.wayPointReachedData;

        waypointReachedData_t wpdata;
        wpdata.index = wpReachedData.waypoint_index;

        // Global position retrieved via subscription
        Telemetry::TypeMap<TOPIC_GPS_FUSED>::type subscribeGPosition;
        subscribeGPosition = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
        wpdata.latitude  = subscribeGPosition.latitude;
        wpdata.longitude = subscribeGPosition.longitude;
        wpdata.altitude  = subscribeGPosition.altitude;
        
        MspController::getInstance()->vehicleNotification(EVehicleNotification::MSP_VHC_WAY_POINT_REACHED, &wpdata); 
    }

}

void missionPauseCallback(Vehicle* vehicle, RecvContainer recvFrame, UserData userData) {
    spdlog::debug("missionPauseCallback");
}

void missionResumeCallback(Vehicle* vehicle, RecvContainer recvFrame, UserData userData) {
    spdlog::debug("missionResumeCallback");
}


bool
setUpSubscription(DJI::OSDK::Vehicle* vehicle, int responseTimeout)
{
    // Telemetry: Verify the subscription
    ACK::ErrorCode subscribeStatus;

    subscribeStatus = vehicle->subscribe->verify(responseTimeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
        ACK::getErrorCodeMessage(subscribeStatus, __func__);
        return false;
    }

    // Telemetry: Subscribe to flight status and mode at freq 10 Hz
    int       freq            = 10;
    TopicName topicList10Hz[] = { TOPIC_GPS_FUSED };
    int       numTopic        = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
    bool      enableTimestamp = false;

    bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
    DEFAULT_PACKAGE_INDEX, numTopic, topicList10Hz, enableTimestamp, freq);
    if (!(pkgStatus))
    {
        return pkgStatus;
    }

    // Start listening to the telemetry data
    subscribeStatus =
    vehicle->subscribe->startPackage(DEFAULT_PACKAGE_INDEX, responseTimeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
        ACK::getErrorCodeMessage(subscribeStatus, __func__);
        // Cleanup
        ACK::ErrorCode ack =
        vehicle->subscribe->removePackage(DEFAULT_PACKAGE_INDEX, responseTimeout);
        if (ACK::getError(ack))
        {
            std::cout << "Error unsubscribing; please restart the drone/FC to get back to a clean state.\n";
        }
        return false;
    }
    return true;
}

bool
teardownSubscription(DJI::OSDK::Vehicle* vehicle, const int pkgIndex, int responseTimeout) {
    ACK::ErrorCode ack = vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
    if (ACK::getError(ack))
    {
        std::cout << "Error unsubscribing; please restart the drone/FC to get back "
            "to a clean state.\n";
        return false;
    }
    return true;
}

//-------------------------------------------------------------
// Mission upload and running
//-------------------------------------------------------------
EResult
uploadWaypoints() {
    EResult ret = EResult::MSP_FAILED;


    // Waypoint Mission : Initialization
    WayPointInitSettings fdata;
    setWaypointInitDefaults(&fdata);

    fdata.indexNumber = wp_list.size(); // We add 1 to get the aircarft back to the start.

    if (vehicle) {
        //vehicle->missionManager->wpMission->stop();
        ACK::ErrorCode initAck = vehicle->missionManager->init(DJI_MISSION_TYPE::WAYPOINT, responseTimeout, &fdata);
        if (ACK::getError(initAck))
        {
            ACK::getErrorCodeMessage(initAck, __func__);
            spdlog::error("uploadWaypoints, fail");
            return ret;
        }

        for (int i = 0; i < wp_list.size(); i++)
        {
            WayPointSettings* wp = &wp_list[i];
            printf("Waypoint created at (LLA): %f \t%f \t%f\n ", wp->latitude, wp->longitude, wp->altitude);
            ACK::WayPointIndex wpDataACK = vehicle->missionManager->wpMission->uploadIndexData(&(*wp), responseTimeout);
            ACK::getErrorCodeMessage(wpDataACK.ack, __func__);
        }

        // Add callback for waypoint management
        vehicle->missionManager->wpMission->setWaypointEventCallback(wayPointEventCallback, &eventWayPointFinishData);
        ret = EResult::MSP_SUCCESS;
    }
    else {
        spdlog::warn("uploadWaypoints, vehicle not initialized");
    }
    return ret;
}

EResult
runWaypointMission() {
    EResult ret = EResult::MSP_FAILED;
    
    // Waypoint Mission: Start
    if (vehicle)
    {
        ACK::ErrorCode startAck = vehicle->missionManager->wpMission->start(responseTimeout);
        if (ACK::getError(startAck))
        {
            ACK::getErrorCodeMessage(startAck, __func__);
            spdlog::error("runWaypointMission, fail");
        }
        else
        {
            ret = EResult::MSP_SUCCESS;
            spdlog::info("runWaypointMission");
        }
    }
    else {
        spdlog::warn("runWaypointMission, vehicle not initialized");
    }
    return ret;
}

EResult
pauseWaypointMission() {
    EResult ret = EResult::MSP_FAILED;
    if (vehicle) {
        vehicle->missionManager->wpMission->pause(&missionPauseCallback, NULL);
        ret = EResult::MSP_SUCCESS;
    }
    else {
        spdlog::warn("pauseWaypointMission, vehicle not initialized");
    }    
}

EResult
resumeWaypointMission() {
    EResult ret = EResult::MSP_FAILED;
    if (vehicle) {
        vehicle->missionManager->wpMission->resume(&missionResumeCallback, NULL);
        ret = EResult::MSP_SUCCESS;
    }
    else {
        spdlog::warn("resumeWaypointMission, vehicle not initialized");
    }
}

//-------------------------------------------------------------
// Waypoint creation
//-------------------------------------------------------------
void
createWaypoints() {

    wp_list.clear();

    if (!setUpSubscription(vehicle, responseTimeout))
    {
        std::cout << "Failed to set up Subscription!" << std::endl;
    }
    // Create Start Waypoint
    WayPointSettings start_wp;
    setWaypointDefaults(&start_wp);

    // Global position retrieved via subscription
    Telemetry::TypeMap<TOPIC_GPS_FUSED>::type subscribeGPosition;

    subscribeGPosition = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
    start_wp.latitude  = subscribeGPosition.latitude;
    start_wp.longitude = subscribeGPosition.longitude;
    start_wp.altitude  = 10;
    start_wp.index = 0;
    //teardownSubscription(vehicle, DEFAULT_PACKAGE_INDEX, responseTimeout);

    //wp_list.push_back(start_wp);
    

    for (int i = 0; i < MspController::getInstance()->getMissionItemCount(); i++)
    {
        WayPointSettings  wp;
        setWaypointDefaults(&wp);

        mavlink_mission_item_t* item = MspController::getInstance()->getMissionBehaviorItem(i);
        if (item){
            wp.index     = wp_list.size();
            wp.longitude  = item->x * M_PI / 180;
            wp.latitude = item->y * M_PI / 180;
            wp.altitude  = item->z;
            
            wp.actionNumber = 0;
            wp.hasAction = 0;// TODO
            //wp.gimbalPitch
            //wp.yaw 
            wp_list.push_back(wp);
        }
    }
    //wp_list = createWaypoints2(vehicle, numWaypoints,0.000001,20);
    //start_wp.index = wp_list.size();
    //wp_list.push_back(start_wp);
}

void
setWaypointDefaults(WayPointSettings* wp)
{
    wp->damping         = 0;
    wp->yaw             = 0;
    wp->gimbalPitch     = 0;
    wp->turnMode        = 0;
    wp->hasAction       = 0;
    wp->actionTimeLimit = 100;
    wp->actionNumber    = 0;
    wp->actionRepeat    = 0;
    for (int i = 0; i < 16; ++i)
    {
        wp->commandList[i]      = 0;
        wp->commandParameter[i] = 0;
    }
}

void
setWaypointInitDefaults(WayPointInitSettings* fdata)
{
    fdata->maxVelocity    = 10;
    fdata->idleVelocity   = 5;
    fdata->finishAction   = 0;
    fdata->executiveTimes = 1;
    fdata->yawMode        = 0;
    fdata->traceMode      = 0;
    fdata->RCLostAction   = 1;
    fdata->gimbalPitch    = 0;
    fdata->latitude       = 0;
    fdata->longitude      = 0;
    fdata->altitude       = 0;
}

//-------------------------------------------------------------
// Camera actions
//-------------------------------------------------------------
EResult 
takePicture(void* data) {
    EResult ret = EResult::MSP_FAILED;

    mavlink_mission_item_t* item = (mavlink_mission_item_t*)data;
    
    RotationAngle initialAngle;
    GimbalContainer gimbal = GimbalContainer(0, item->param1, item->param2, 0, 1, initialAngle);
    setGimbalAngle(vehicle, &gimbal);
    vehicle->camera->shootPhoto();
    ret = EResult::MSP_SUCCESS;
}

/**
 * From DJI Sample
*/
void
setGimbalAngle(Vehicle* vehicle, GimbalContainer* gimbal) {
  DJI::OSDK::Gimbal::AngleData gimbalAngle = {};
  gimbalAngle.roll     = gimbal->roll;
  gimbalAngle.pitch    = gimbal->pitch;
  gimbalAngle.yaw      = gimbal->yaw;
  gimbalAngle.duration = gimbal->duration;
  gimbalAngle.mode |= 0;
  gimbalAngle.mode |= gimbal->isAbsolute;
  gimbalAngle.mode |= gimbal->yaw_cmd_ignore << 1;
  gimbalAngle.mode |= gimbal->roll_cmd_ignore << 2;
  gimbalAngle.mode |= gimbal->pitch_cmd_ignore << 3;

  vehicle->gimbal->setAngle(&gimbalAngle);
}