#include "msp_vehicle.h"

static MspMockVehicle* mspVehicle;
static std::thread runner;

static EResult 
cmdCallback(EVehicleCmd cmd, void* data, size_t len) {
    EResult ret = EResult::MSP_FAILED; 
    switch (cmd)
    {
        case EVehicleCmd::MSP_CMD_READ_STATE:
            ret = mspVehicle->handleStateRequest();
            break;
        case EVehicleCmd::MSP_CMD_UPLOAD_WAY_POINTS:
            ret = EResult::MSP_SUCCESS;
            break;
        case EVehicleCmd::MSP_CMD_MISSION_START:
            ret = mspVehicle->runWaypointMission();
            break;
        case EVehicleCmd::MSP_CMD_MISSION_PAUSE:
            ret = mspVehicle->pauseWaypointMission();
            break;
        case EVehicleCmd::MSP_CMD_MISSION_RESUME:
            ret = mspVehicle->resumeWaypointMission();
            break;
        case EVehicleCmd::MSP_CMD_TAKE_PICTURE:
           
        default:
            break;
    }
    
    return ret;
}

//-------------------------------------------------------------
// Class MspMockVehicle 
//-------------------------------------------------------------
void 
MspMockVehicle::initialize() {

    mspVehicle = this;

    // set callback for controller command data
    MspController::getInstance()->vehicleCmd = &cmdCallback;
}

EResult 
MspMockVehicle::handleStateRequest() {

    vehicleStateData_t data = { };
    data.state |= EVehicleState::MSP_VHC_AVAILABLE | EVehicleState::MSP_VHC_READY;
    
    MspController::getInstance()->vehicleNotification(EVehicleNotification::MSP_VHC_STATE, &data);
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
    
    for (int i = 0; i < MspController::getInstance()->getMissionItemCount(); i++) {

        mavlink_mission_item_t* item = MspController::getInstance()->getMissionBehaviorItem(i);
        if (item) {
            spdlog::debug("way point reached");

            // Current waypoint data
            waypointReachedData_t wpdata;
            wpdata.index = i;

            wpdata.latitude  = item->y;
            wpdata.longitude = item->x;
            wpdata.altitude  = item->z;

            MspController::getInstance()->vehicleNotification(EVehicleNotification::MSP_VHC_WAY_POINT_REACHED, &wpdata);
        }
        sleep(1);
    }
}