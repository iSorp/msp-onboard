#include "controller.h"

//-------------------------------------------------------------
// Class Idle 
//-------------------------------------------------------------
void
MspController::Idle::entry() {

}

EResult
MspController::Idle::cmdExecute(uint16_t command) {
    switch (command)
    {
    case MAV_CMD_MISSION_START:
        context->setState(&context->stateMission);
        return context->stateMission.missionStart();
        break;
    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        context->setState(&context->stateCommand);
        return context->stateCommand.cmdExecute(command);
        break;
    default:
        return EResult::INVALID;
        break;
    }
}