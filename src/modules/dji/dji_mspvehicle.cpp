        
#include "dji_mspvehicle.h"
#include "dji_mobile_interface.h"
#include "controller.h"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

static const int GPS_PACKAGE_INDEX = 0;
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

//-------------------------------------------------------------
// Class MspDjiVehicle
//-------------------------------------------------------------
MspDjiVehicle::MspDjiVehicle(Vehicle* vehicleDJI, Mavlink* mavlinkDJI) {
    mavlink = static_cast<MavlinkDJI*>(mavlinkDJI);
    vehicle = vehicleDJI;
}

MspDjiVehicle::~MspDjiVehicle() {
    // teardown up data subscription
    teardownSubscription(vehicle, GPS_PACKAGE_INDEX, responseTimeout);
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

    // set up data subscription
    setUpSubscription(vehicle, responseTimeout,  GPS_PACKAGE_INDEX, 10, { TOPIC_GPS_FUSED });
}

//-------------------------------------------------------------
// Vehicle information
//-------------------------------------------------------------
EResult 
MspDjiVehicle::handleStateRequest() {

    vehicleStateData_t data = { };
    if (vehicle && vehicle->getActivationStatus()) {
        data.state |= EVehicleState::MSP_VHC_AVAILABLE;

        if (this->status > 0) {
            data.state |= EVehicleState::MSP_VHC_MISSION;
        }

        // TODO: more state information
        // vehicle->broadcast->getStatus()
    }

    MspController::getInstance()->vehicleNotification(EVehicleNotification::MSP_VHC_STATE, &data);
    return EResult::MSP_SUCCESS;
}

//-------------------------------------------------------------
// Telemetrie data subscription
//-------------------------------------------------------------
bool
MspDjiVehicle::setUpSubscription(Vehicle* vehicle, int responseTimeout, int pkgIndex, int freq, std::vector<TopicName> topicList) {
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
MspDjiVehicle::teardownSubscription(Vehicle* vehicle, const int pkgIndex, int responseTimeout) {
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