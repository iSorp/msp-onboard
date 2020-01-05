/**
    @file controller_idle.cpp
    @brief 
    Implementation of the Idle state class.

    @Copyright (c) 2019 Samuel Ackermann, Simon Wälti
*/

#include "controller.h"

//-------------------------------------------------------------
// Class Idle 
//-------------------------------------------------------------
void
MspController::Idle::entry() { }

//------------------------------------------------------------- 
// Command and notification interface
//-------------------------------------------------------------
EResult
MspController::Idle::setCommand(uint16_t command, mavlink_command_long_t cmd) {
    
    // set new state depending on command
    switch (command)
    {
    case MAV_CMD_MISSION_START:
        context->setState(&context->stateMission);
        break;
    case MAV_CMD_DO_PAUSE_CONTINUE:
        context->setState(&context->stateMission);
        break;
    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        context->setState(&context->stateCommand);
        break;
    default:
        spdlog::warn("MspController::Idle::setCommand, command not available");
        return EResult::MSP_INVALID;
        break;
    }

    // execute command in current state
    return context->getState()->setCommand(command, cmd);
}