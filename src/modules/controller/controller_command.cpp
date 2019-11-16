#include "controller.h"


EResult 
MspController::Command::cmdExecute(uint16_t command, mavlink_command_long_t cmd){
    EResult res = EResult::MSP_FAILED;
    switch (command)
    {
    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        MspController::getInstance()->vehicleCmd(EVehicleCmd::MSP_CMD_RETURN_TO_ORIGIN, NULL, 0);
        res = EResult::MSP_PROGRESS;
    case MAV_CMD_NAV_TAKEOFF:
        MspController::getInstance()->vehicleCmd(EVehicleCmd::MSP_CMD_TAKEOFF, NULL, 0);
        res = EResult::MSP_PROGRESS;
        case MAV_CMD_NAV_LAND:
        MspController::getInstance()->vehicleCmd(EVehicleCmd::MSP_CMD_LAND, NULL, 0);
        res = EResult::MSP_PROGRESS;
    default:
        res = EResult::MSP_INVALID;
        break;
    }

    return res;
}

void 
MspController::Command::vehicleNotification(EVehicleNotification notification, VehicleData data) {

    if (notification == EVehicleNotification::MSP_VHC_LANDED) {
        Mavlink* mavlink = MspController::getInstance()->mavlink;
        mavlink->setCmdResult(EResult::MSP_SUCCESS);
    }
}