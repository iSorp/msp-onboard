#include <cmath>
#include <vector>
#include <dji_vehicle.hpp>

#include "controller.h"
#include "dji_mspvehicle.h"
#include "dji_camera_gimbal_def.h"


using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

static int responseTimeout = 1;

// WayPointStatusPushData
static const uint8_t PRE_MISSION        = 0; 
static const uint8_t IN_ACTION          = 1; 
static const uint8_t FIRST_WAYPOINT     = 5; 
static const uint8_t WAYPOINT_REACHED   = 6; 

//-------------------------------------------------------------
// Static funcitions
//-------------------------------------------------------------
static void
setWaypointDefaults(WayPointSettings* wp);
static void
setWaypointInitDefaults(WayPointInitSettings* fdata);
static void
setGimbalAngle(Vehicle* vehicle, GimbalContainer* gimbal);

//-------------------------------------------------------------
// Callbacks
//-------------------------------------------------------------
void wayPointEventCallback(Vehicle* vehicle, RecvContainer recvFrame, UserData userData) {
 
    if (recvFrame.recvData.wayPointStatusPushData.current_status == WAYPOINT_REACHED) {
        spdlog::debug("way point reached");

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
void wayPointCallback(Vehicle* vehicle, RecvContainer recvFrame, UserData userData) {
 
    MspDjiVehicle* mspVehicle = static_cast<MspDjiVehicle*>(userData);
    if (mspVehicle){
        mspVehicle->status = recvFrame.recvData.wayPointStatusPushData.current_status;
    }
}

void missionPauseCallback(Vehicle* vehicle, RecvContainer recvFrame, UserData userData) {
    spdlog::debug("missionPauseCallback");
}

void missionResumeCallback(Vehicle* vehicle, RecvContainer recvFrame, UserData userData) {
    spdlog::debug("missionResumeCallback");
}

//-------------------------------------------------------------
// Mission upload and running
//-------------------------------------------------------------
EResult
MspDjiVehicle::uploadWaypoints() {
    EResult ret = EResult::MSP_FAILED;

    // Waypoint Mission initialization
    WayPointInitSettings fdata;
    setWaypointInitDefaults(&fdata);

    // number of expected mission waypoints
    fdata.indexNumber = wp_list.size(); 

    //vehicle->missionManager->wpMission->stop();
    ACK::ErrorCode initAck = vehicle->missionManager->init(DJI_MISSION_TYPE::WAYPOINT, responseTimeout, &fdata);
    if (ACK::getError(initAck))
    {
        ACK::getErrorCodeMessage(initAck, __func__);
        spdlog::error("uploadWaypoints, fail");
        return ret;
    }

    for (size_t i = 0; i < wp_list.size(); i++)
    {
        WayPointSettings* wp = &wp_list[i];
        printf("Waypoint created at (LLA): %f \t%f \t%f\n ", wp->latitude, wp->longitude, wp->altitude);
        ACK::WayPointIndex wpDataACK = vehicle->missionManager->wpMission->uploadIndexData(&(*wp), responseTimeout);
        ACK::getErrorCodeMessage(wpDataACK.ack, __func__);
    }

    // Add callback for waypoint management
    vehicle->missionManager->wpMission->setWaypointEventCallback(wayPointEventCallback, this);
    vehicle->missionManager->wpMission->setWaypointCallback(wayPointCallback, this);
    ret = EResult::MSP_SUCCESS;

    return ret;
}

EResult
MspDjiVehicle::runWaypointMission() {
    EResult ret = EResult::MSP_FAILED;
    
    // Waypoint Mission: Start
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
    return ret;
}

EResult
MspDjiVehicle::pauseWaypointMission() {
    vehicle->missionManager->wpMission->pause(&missionPauseCallback, NULL);
    return EResult::MSP_SUCCESS;   
}

EResult
MspDjiVehicle::resumeWaypointMission() {
    vehicle->missionManager->wpMission->resume(&missionResumeCallback, NULL);
    return EResult::MSP_SUCCESS;
}

//-------------------------------------------------------------
// Waypoint creation
//-------------------------------------------------------------
void
MspDjiVehicle::createWaypoints() {

    wp_list.clear();

    // Create Start Waypoint
    WayPointSettings start_wp;
    setWaypointDefaults(&start_wp);

    // subscribe for gps position, add start_wp to the waypoint vector if 
    // start position is needed
    Telemetry::TypeMap<TOPIC_GPS_FUSED>::type subscribeGPosition;
    subscribeGPosition = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();

    start_wp.latitude  = subscribeGPosition.latitude;
    start_wp.longitude = subscribeGPosition.longitude;
    start_wp.altitude  = 10;
    start_wp.index = 0;

    for (size_t i = 0; i < MspController::getInstance()->getMissionItemCount(); i++)
    {
        WayPointSettings  wp;
        setWaypointDefaults(&wp);

        mavlink_mission_item_t* item = MspController::getInstance()->getMissionBehaviorItem(i);
        if (item) {
            wp.index        = wp_list.size();
            wp.longitude    = item->x * M_PI / 180;   // degree to radian 
            wp.latitude     = item->y * M_PI / 180;   // degree to radian 
            wp.altitude     = item->z;
            
            wp.actionNumber = 0;
            wp.hasAction    = 0;
            //wp.gimbalPitch
            //wp.yaw 
            wp_list.push_back(wp);
        }
    }
}

//-------------------------------------------------------------
// Camera actions
//-------------------------------------------------------------
EResult 
MspDjiVehicle::takePicture(void* data) {

    mavlink_mission_item_t* item = (mavlink_mission_item_t*)data;
    
    RotationAngle initialAngle;
    GimbalContainer gimbal = GimbalContainer(0, item->param1, item->param2, 0, 1, initialAngle);
    setGimbalAngle(vehicle, &gimbal);
    vehicle->camera->shootPhoto();
    return EResult::MSP_SUCCESS;
}


//-------------------------------------------------------------
// static helpers
//-------------------------------------------------------------

/**
 * From DJI Sample
*/
static void
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

static void
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

static void
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