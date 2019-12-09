#include <thread>
#include "controller.h"
#include "mspvehicle.h"

static std::thread runner;


//-------------------------------------------------------------
// Static funcitions
//-------------------------------------------------------------
static EResult 
commandCallback(EVehicleCmd command, VehicleData vehicleData, void* userData, size_t len) {
    MspMockVehicle* vehicle = static_cast<MspMockVehicle*>(vehicleData);
    if (!vehicle) {
        spdlog::error("mock vehicle command, no valid vehicle in vehicleData");
        return EResult::MSP_FAILED;
    }
    return vehicle->command(command, userData, len);
}

//-------------------------------------------------------------
// Class MspMockVehicle 
//-------------------------------------------------------------
void 
MspMockVehicle::initialize() {
    MspController::getInstance()->setVehicleCommandCallback(commandCallback, this);
}

EResult 
MspMockVehicle::handleStateRequest() {

    VehicleStateData vehicleInfo = { };
    vehicleInfo.state = EVehicleState::MSP_VHC_SIMULATION;
 
    MspController::getInstance()->vehicleNotification(EVehicleNotification::MSP_VHC_STATE, &vehicleInfo);
    return EResult::MSP_SUCCESS;
}

//-------------------------------------------------------------
// Mission upload and running
//-------------------------------------------------------------

EResult
MspMockVehicle::runWaypointMission() {
    if (runner.joinable()){
        runner.join();
    }

    spdlog::info("start mission simulation thread");
    runner = std::thread(&MspMockVehicle::missionRun, this);
    return EResult::MSP_SUCCESS;
}

EResult
MspMockVehicle::pauseWaypointMission() {
    return EResult::MSP_SUCCESS;
}

EResult
MspMockVehicle::resumeWaypointMission() {
    return EResult::MSP_SUCCESS;
}

//-------------------------------------------------------------
// Mission running helper thread
//-------------------------------------------------------------
void 
MspMockVehicle::missionRun() {
    spdlog::info("thread running");
    sleep(2);
    
    for (size_t i = 0; i < MspController::getInstance()->getMissionItemCount(); i++) {
    
        mavlink_mission_item_t* item = MspController::getInstance()->getMissionBehaviorItem(i);
        if (item) {
            spdlog::debug("way point reached");

            // Current waypoint data
            WaypointReachedData wpdata;
            wpdata.index = i;

            wpdata.latitude  = item->y;
            wpdata.longitude = item->x;
            wpdata.altitude  = item->z;

            MspController::getInstance()->vehicleNotification(EVehicleNotification::MSP_VHC_WAY_POINT_REACHED, &wpdata);
        }
        
        sleep(1);
    }
}

EResult 
MspMockVehicle::command(EVehicleCmd cmd, void* data, size_t len) {
  EResult ret = EResult::MSP_FAILED; 
    switch (cmd)
    {
        case EVehicleCmd::MSP_CMD_READ_STATE:
            ret = handleStateRequest();
            break;
        case EVehicleCmd::MSP_CMD_UPLOAD_WAY_POINTS:
            ret = EResult::MSP_SUCCESS;
            break;
        case EVehicleCmd::MSP_CMD_MISSION_START:
            ret = runWaypointMission();
            break;
        case EVehicleCmd::MSP_CMD_MISSION_PAUSE:
            ret = pauseWaypointMission();
            break;
        case EVehicleCmd::MSP_CMD_MISSION_RESUME:
            ret = resumeWaypointMission();
            break;
        case EVehicleCmd::MSP_CMD_TAKE_PICTURE:
           
        default:
            break;
    }
    
    return ret;
}