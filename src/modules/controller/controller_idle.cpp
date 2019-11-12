#include "controller.h"

//-------------------------------------------------------------
// Class Idle 
//-------------------------------------------------------------
void
MspController::Idle::entry() {

}

EResult
MspController::Idle::cmdExecute(uint16_t command, mavlink_command_long_t cmd) {
    
    // set new state depending on command
    switch (command)
    {
    case MAV_CMD_MISSION_START:
        context->setState(&context->stateMission);
        break;
    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        context->setState(&context->stateCommand);
        break;
    default:
        return EResult::MSP_INVALID;
        break;
    }

    // execute command in current state
    return context->getState()->cmdExecute(command, cmd);
}