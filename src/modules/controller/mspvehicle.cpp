/**
    Contains the implementation of the MspMockVehicle.
    @file mspvehicle.h
    @version 1.0
    @author Simon Waelti
    @version 1.0 1.12.2019
*/
#include <thread>
#include "controller.h"
#include "mspvehicle.h"


static std::thread runner;

//-------------------------------------------------------------
// Static funcitions
//-------------------------------------------------------------

/**
    @brief
    Callback function for commands emitted by MspController
    @param EVehicleCmd command 
    @param VehicleData vehicleData
    @param void* userData 
    @param size_t len
    @return EResult
*/
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

// Intializes the vehicle and notifies the controller about the state.
void 
MspMockVehicle::initialize() {
    MspController::getInstance()->setVehicleCommandCallback(commandCallback, this);
    
    VehicleInfoData vehicleInfo = { };
    vehicleInfo.state = EVehicleState::MSP_VHC_SIMULATION;
 
    MspController::getInstance()->vehicleNotification(EVehicleNotification::MSP_VHC_STATE, &vehicleInfo);
}

//-------------------------------------------------------------
// Mission upload and running
//-------------------------------------------------------------

// Simulates the start of a new waypoint mission. This function is synchronized with the waypoint handling thread. 
EResult
MspMockVehicle::runWaypointMission() {
    if (runner.joinable()){
        runner.join();
    }

    spdlog::info("start mission simulation thread");
    exit = false;
    runner = std::thread(&MspMockVehicle::missionRun, this);
    return EResult::MSP_SUCCESS;
}
// Simulates the pausing of a running waypoint mission.
EResult
MspMockVehicle::pauseWaypointMission() {
    return EResult::MSP_PROGRESS;
}
// Simulates the resuming of a running waypoint mission.
EResult
MspMockVehicle::resumeWaypointMission() {
    return EResult::MSP_PROGRESS;
}
// Simulates the stopping of a running waypoint mission.
EResult
MspMockVehicle::stopWaypointMission() {
    exit = true;
    return EResult::MSP_PROGRESS;
}

//-------------------------------------------------------------
// Mission running helper thread
//-------------------------------------------------------------

// Runner function for the waypint handling thread. On each waypoint the MspController is notified.
void 
MspMockVehicle::missionRun() {
    spdlog::info("thread running");
    sleep(2);

    for (size_t i = 0; i < MspController::getInstance()->getMissionItemCount(); i++) {
        
        if (exit) break;

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
// Calls the requestet command function
EResult 
MspMockVehicle::command(EVehicleCmd cmd, void* data, size_t len) {
  EResult ret = EResult::MSP_FAILED; 
    switch (cmd)
    {
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
        case EVehicleCmd::MSP_CMD_MISSION_STOP:
            ret = stopWaypointMission();
            break;
        case EVehicleCmd::MSP_CMD_TAKE_PICTURE:
           
        default:
            break;
    }
    
    return ret;
}