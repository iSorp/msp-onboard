#include <string>

#include "helper.h"
#include "controller.h"
#include "mav_mavlink.h"
#include "mav_command.h"


//-------------------------------------------------------------
// Class MavlinkCommandManager
//-------------------------------------------------------------

void
MavlinkCommandManager::run() {
    commandService.getState()->run();
}

void
MavlinkCommandManager::handle_message(const mavlink_message_t *msg)
{
    // command int not supported
    if(msg->msgid == MAVLINK_MSG_ID_COMMAND_INT) {
        spdlog::warn("MavlinkCommandManager::handle_message, mavlink command not supported");
        commandService.sendCmdAck(0, MAV_RESULT_UNSUPPORTED);
        return;
    }

    if (msg->msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
        // Accept command request only if no command execution is pending
        if (typeid(*commandService.getState()) == typeid(MavlinkCommandManager::CommandService::CommandInit))
        {
            commandService.setState(&commandService.commandReceive);   
        } 
        else {

            // Send error response
            if (msg->msgid == MAVLINK_MSG_ID_COMMAND_LONG){
                commandService.sendCmdAck(mavlink_msg_command_long_get_command(msg), MAV_RESULT_TEMPORARILY_REJECTED);
            }
            if (msg->msgid == MAVLINK_MSG_ID_COMMAND_INT){
                commandService.sendCmdAck(mavlink_msg_command_long_get_command(msg), MAV_RESULT_TEMPORARILY_REJECTED);
            }

            return;
        }
        commandService.getState()->handleMessage(msg);
    }
}

//-------------------------------------------------------------
// Class MissionDownloadService
//-------------------------------------------------------------
void
MavlinkCommandManager::CommandService::sendCmdAck(uint16_t cmd, uint8_t result)
{
    mavlink_command_ack_t cmda;
    cmda.command = cmd;
    cmda.result = result;
    mavlink_msg_command_ack_send_struct(mavlink->getChannel(), &cmda);
}

void
MavlinkCommandManager::CommandService::handleCmdErrorResultAndInit(EResult result, uint16_t command)
{
    switch (result)
    {
    case EResult::MSP_INVALID:
        sendCmdAck(command, MAV_RESULT_UNSUPPORTED);
        setState(&commandInit);
        break;
    case EResult::MSP_BUSY:
        sendCmdAck(command, MAV_RESULT_TEMPORARILY_REJECTED);
        setState(&commandInit);
        break;
    case EResult::MSP_FAILED:
        sendCmdAck(command, MAV_RESULT_FAILED);
        setState(&commandInit);
        break;
    default:
        break;
    }
}

//-------------------------------------------------------------
// Class CommandInit (only for activating command state machine)
//-------------------------------------------------------------
void 
MavlinkCommandManager::CommandService::CommandInit::entry() {
}

//-------------------------------------------------------------
// Class CommandReceive 
//-------------------------------------------------------------
void 
MavlinkCommandManager::CommandService::CommandReceive::handleMessage(const mavlink_message_t *msg) {

    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(msg, &cmd);

    // Execute command
    EResult res = MspController::getInstance()->cmdExecute(cmd.command, cmd);
    switch (res)
    {
    case EResult::MSP_SUCCESS:
        context->setState(&context->commandEnd);
        break;
    case EResult::MSP_PROGRESS:
        // command is finished later
        context->setState(&context->commandProgress);
        break;

    default:
        context->handleCmdErrorResultAndInit(res, cmd.command);
        break;
    }
}

//-------------------------------------------------------------
// Class CommandProgress 
//-------------------------------------------------------------
void 
MavlinkCommandManager::CommandService::CommandProgress::run() {
    // TODO send progress
    /*if (controllerprogress..) {

    }*/

    // TODO handle timeout

     // command is done 
    if (context->cmdResult == EResult::MSP_SUCCESS) {
        context->setState(&context->commandEnd);
    }
    else if (context->cmdResult != EResult::MSP_PROGRESS) {
        spdlog::error("CommandService::run, command failed: " + std::to_string(context->cmdResult));
        context->handleCmdErrorResultAndInit(context->cmdResult, context->currentCmd);
    }
}

//-------------------------------------------------------------
// Class CommandEnd (send command acknowledge message)
//-------------------------------------------------------------
void 
MavlinkCommandManager::CommandService::CommandEnd::entry() {

    // send command acknowledge message
    context->sendCmdAck(MAVLINK_MSG_ID_COMMAND_ACK, MAV_RESULT_ACCEPTED);
    context->setState(&context->commandInit);
}