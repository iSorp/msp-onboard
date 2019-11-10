#include "controller.h"


EResult 
MspController::Command::cmdExecute(uint16_t command){
    switch (command)
    {
    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        MspController::getInstance()->vehicleCmd(EVehicleCmd::RETURN_TO_ORIGIN, NULL, 0);
        return EResult::PROGRESS;
    default:
        return EResult::INVALID;
        break;
    }
}


void 
MspController::Command::vehicleNotification(EVehicleNotification notification) {

    if (notification == EVehicleNotification::VEHICLE_LANDED) {
        Mavlink* mavlink = MspController::getInstance()->mavlink;
        mavlink->setCmdResult(EResult::SUCCESS);
    }
}