/**
    @file dji_mspvehicle.cpp
    @brief

    @Copyright (c) 2019 Samuel Ackermann, Simon Wälti
*/        


#include "dji_mspvehicle.h"
#include "dji_mobile_interface.h"
#include "controller.h"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

static const int GPS_PACKAGE_INDEX = 0;
static const int STATUS_PACKAGE_INDEX = 1;


static const int responseTimeout = 1;

//-------------------------------------------------------------
// Static funcitions
//-------------------------------------------------------------
static EResult 
commandCallback(EVehicleCmd command, VehicleData vehicleData, void* userData, size_t len) {
   MspDjiVehicle* vehicle = static_cast<MspDjiVehicle*>(vehicleData);
    if (!vehicle) {
        spdlog::error("dji vehicle command, no valid vehicle in vehicleData");
        return EResult::MSP_FAILED;
    }
    return vehicle->command(command, userData, len);
}

static void 
vehicleStateCallback(Vehicle* vehicle, RecvContainer recvFrame, UserData userData){

    Telemetry::TypeMap<TOPIC_STATUS_FLIGHT>::type flightState = vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>();
    Telemetry::TypeMap<TOPIC_STATUS_DISPLAYMODE>::type displayMode = vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>();

    VehicleInfoData vehicleInfo = { };
    if (vehicle && vehicle->getActivationStatus()) {

        // Vehicle status
        vehicleInfo.state = EVehicleState::MSP_VHC_AVAILABLE;

        // DJI -> Mavlink Mode 
        if (displayMode == VehicleStatus::FlightStatus::ON_GROUND 
            || displayMode == VehicleStatus::FlightStatus::IN_AIR) {
                vehicleInfo.mode |= MAV_MODE_FLAG_SAFETY_ARMED;
        }
        
        if (displayMode == VehicleStatus::DisplayMode::MODE_ATTITUDE 
            || displayMode == VehicleStatus::DisplayMode::MODE_P_GPS 
            || displayMode == VehicleStatus::DisplayMode::MODE_SEARCH_MODE 
            || displayMode == VehicleStatus::DisplayMode::MODE_NAVI_SDK_CTRL
            || displayMode == VehicleStatus::DisplayMode::MODE_HOTPOINT_MODE
            || displayMode == VehicleStatus::DisplayMode::MODE_FORCE_AUTO_LANDING
            || displayMode == VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF
            || displayMode == VehicleStatus::DisplayMode::MODE_AUTO_LANDING
            || displayMode == VehicleStatus::DisplayMode::MODE_FORCE_AUTO_LANDING) {
                vehicleInfo.mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;
                vehicleInfo.mode |= MAV_MODE_FLAG_AUTO_ENABLED;
                vehicleInfo.mode |= MAV_MODE_FLAG_GUIDED_ENABLED;
                vehicleInfo.mode |= MAV_MODE_FLAG_SAFETY_ARMED;
        }
    }else {
        vehicleInfo.state = EVehicleState::MSP_VHC_SIMULATION;
    }
    MspController::getInstance()->vehicleNotification(EVehicleNotification::MSP_VHC_STATE, &vehicleInfo);
}

//-------------------------------------------------------------
// Class MspDjiVehicle
//-------------------------------------------------------------
MspDjiVehicle::MspDjiVehicle(Vehicle* vehicleDJI, Mavlink* mavlinkDJI) {
    mavlink = static_cast<MavlinkDJI*>(mavlinkDJI);
    vehicle = vehicleDJI;
}

MspDjiVehicle::~MspDjiVehicle() {
    // teardown up data subscription
    vehicle->subscribe->removeAllExistingPackages();
    vehicle->releaseCtrlAuthority(responseTimeout);
}

void
MspDjiVehicle::initialize() {
    
    // Set callback for MspController, all vehicle commands run over this function
    MspController::getInstance()->setVehicleCommandCallback(commandCallback, this);

    // Obtain Control Authority
    vehicle->obtainCtrlAuthority(responseTimeout);

    // set callback for mavlink send data
    mavlink->setSendDataCallback(sendDataToMobile, vehicle);
    
    // set callback for mobile data
    vehicle->mobileDevice->setFromMSDKCallback(mobileCallback, mavlink);

    // set up data subscription, Topics with the same refresh rate belongs to the same package
    setUpSubscription(responseTimeout,  GPS_PACKAGE_INDEX, 10, { TOPIC_GPS_FUSED });
    setUpSubscription(responseTimeout,  STATUS_PACKAGE_INDEX, 100, { TOPIC_STATUS_FLIGHT, TOPIC_STATUS_DISPLAYMODE });

    // callback registration for vehicle status
    vehicle->subscribe->registerUserPackageUnpackCallback(STATUS_PACKAGE_INDEX, vehicleStateCallback, this);

    VehicleInfoData vehicleInfo = { };
    vehicleInfo.state = EVehicleState::MSP_VHC_AVAILABLE;
    MspController::getInstance()->vehicleNotification(EVehicleNotification::MSP_VHC_STATE, &vehicleInfo);
}

//-------------------------------------------------------------
// Telemetrie data subscription
//-------------------------------------------------------------
bool
MspDjiVehicle::setUpSubscription(int responseTimeout, int pkgIndex, int freq, std::vector<TopicName> topicList) {
    // Telemetry: Verify the subscription
    ACK::ErrorCode subscribeStatus;

    subscribeStatus = vehicle->subscribe->verify(responseTimeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
        spdlog::error("setUpSubscription, fail");
        ACK::getErrorCodeMessage(subscribeStatus, __func__);
        return false;
    }

    // Subscribe
    bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(pkgIndex, (int)topicList.size() , topicList.data(), false, freq);
    if (!(pkgStatus))
    {
        return pkgStatus;
    }

    // Start listening to the telemetry data
    subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
    if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
    {
        ACK::getErrorCodeMessage(subscribeStatus, __func__);
        // Cleanup
        ACK::ErrorCode ack = vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
        if (ACK::getError(ack)){
            spdlog::error("Error unsubscribing; please restart the drone/FC to get back to a clean state");
        }
        return false;
    }
    return true;
}

bool
MspDjiVehicle::teardownSubscription(const int pkgIndex, int responseTimeout) {
    ACK::ErrorCode ack = vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
    if (ACK::getError(ack)) {
        spdlog::error("Error unsubscribing; please restart the drone/FC to get back to a clean state");
        return false;
    }
    return true;
}
//-------------------------------------------------------------
// Vehicle commands
//-------------------------------------------------------------
EResult 
MspDjiVehicle::command(EVehicleCmd cmd, void* data, size_t len) {

    EResult ret = EResult::MSP_FAILED; 
    switch (cmd)
    {
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
        case EVehicleCmd::MSP_CMD_MISSION_STOP:
            ret = stopWaypointMission();
            break;
        case EVehicleCmd::MSP_CMD_TAKE_PICTURE:
            ret = takePicture(data);
        default:
            break;
    }
    return ret;
}