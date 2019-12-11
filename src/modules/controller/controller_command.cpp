/**
    Implementation of the Command state class.
    @file controller_command.cpp
    @version 1.0
    @author Simon Waelti
    @version 1.0 1.12.2019
*/

#include "controller.h"


EResult 
MspController::Command::setCommand(uint16_t command, mavlink_command_long_t cmd){
    EResult res = EResult::MSP_FAILED;
    switch (command)
    {
    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        res = MspController::getInstance()->setVehicleCommand(EVehicleCmd::MSP_CMD_RETURN_TO_ORIGIN);
    case MAV_CMD_NAV_TAKEOFF:
        res =  MspController::getInstance()->setVehicleCommand(EVehicleCmd::MSP_CMD_TAKEOFF);
    case MAV_CMD_NAV_LAND:
        res =  MspController::getInstance()->setVehicleCommand(EVehicleCmd::MSP_CMD_LAND);
    default:
        res = EResult::MSP_INVALID;
        break;
    }

    return res;
}

void 
MspController::Command::vehicleNotification(EVehicleNotification notification, VehicleData data) {

    if (notification == EVehicleNotification::MSP_VHC_LANDED) {
        MspController::getInstance()->mavlink->setCmdResult(EResult::MSP_SUCCESS);
    }
}