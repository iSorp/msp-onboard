
#include <iostream>

#include "controller.h"
#include "sensors.h"


//-------------------------------------------------------------
// Class Init 
//-------------------------------------------------------------
void
MspController::Init::entry() {

    // Initialize oboard-controller Sensors
    MspSensors::getInstance()->initialize();

    // Read the current vehicle state -> result in vehicleNotification
    MspController::getInstance()->vehicleCmd(EVehicleCmd::MSP_CMD_READ_STATE, NULL, 0);
}

void 
MspController::Init::vehicleNotification(EVehicleNotification notification, VehicleData data) {
    if (notification == EVehicleNotification::MSP_VHC_STATE) {
        vehicleStateData_t* stateData = (vehicleStateData_t*)data;

        bool available = stateData->state & EVehicleState::MSP_VHC_AVAILABLE;
        if (available)
        {
            context->setState(&context->stateIdle);
        }
        else {
            context->setState(&context->stateSim);
        } 
    }
}

//-------------------------------------------------------------
// Class Simulation 
//-------------------------------------------------------------
void
MspController::Simulation::entry() {
    spdlog::info("Simulation active");
}

EResult 
MspController::Simulation::cmdExecute(uint16_t command, mavlink_command_long_t cmd) {
    EResult res = EResult::MSP_FAILED;
    switch (command)
    {
        case MAV_CMD_NAV_TAKEOFF:
        
            break;
        case MAV_CMD_NAV_LAND:
        
        break;
        case MAV_CMD_NAV_RETURN_TO_LAUNCH:
            
            break;
        case MAV_CMD_MISSION_START:
            if (runner.joinable()) {
                res = EResult::MSP_BUSY;
            }
            missionStart();
            res = EResult::MSP_SUCCESS;
            break;
        case MAV_CMD_DO_PAUSE_CONTINUE:

            break;
        default:
            spdlog::warn("MspController::Simulation::cmdExecute, command not available");
            res =  EResult::MSP_INVALID;
            break;
    }

    return res; 
}

void 
MspController::Simulation::vehicleNotification(EVehicleNotification notification, VehicleData data) {
    spdlog::debug("wayPointCallback");
}

void 
MspController::Simulation::missionStart() {

	spdlog::info("start mission simulation thread");
    runner = std::thread(&MspController::Simulation::missionRun, this);
}

void 
MspController::Simulation::missionRun() {
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