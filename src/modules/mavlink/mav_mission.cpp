#include <stdio.h>
#include "mav_mission.h"
#include "mav_mavlink.h"
#include "controller.h"
#include "helper.h"

//-------------------------------------------------------------
// Class MavlinkMissionManager
//-------------------------------------------------------------
void
MavlinkMissionManager::run() {
    missionDownloadService.getState()->run();
}

void
MavlinkMissionManager::handle_message(const mavlink_message_t *msg)
{
    switch (msg->msgid) {

        // Start mission item upload (Creates a new mission)
        case MAVLINK_MSG_ID_MISSION_COUNT:
            missionDownloadService.setState(&missionDownloadService.missionDownloadInit);
            break;

        // deletes the current mission
        case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
            missionDelete(msg);
            break;
            
        default:
            break;
    }
    missionDownloadService.getState()->handleMessage(msg);
}

void
MavlinkMissionManager::missionDelete(const mavlink_message_t *msg)
{
    mavlink_mission_clear_all_t msgCa;
    mavlink_msg_mission_clear_all_decode(msg, &msgCa);
    
    EResult res = MspController::getInstance()->missionDelete();
    if (res == EResult::MSP_SUCCESS) {
        sendMissionAck(msg->sysid, msg->compid, 0);
    }
    else {
        sendMissionAck(msg->sysid, msg->compid, MAV_MISSION_ERROR);
    }
}

void
MavlinkMissionManager::sendMissionAck(uint8_t sysid, uint8_t compid, uint8_t type)
{
	mavlink_mission_ack_t wpa;
	wpa.target_system    = sysid;
	wpa.target_component = compid;
	wpa.type = type;

	mavlink_msg_mission_ack_send_struct(mavlink->getChannel(), &wpa);
}

/****************************************************************/
/*****************classes for mission upload********************/
/****************************************************************/

//-------------------------------------------------------------
// Class MissionDownloadService 
//-------------------------------------------------------------
void
MavlinkMissionManager::MissionDownloadService::handleMissionCount(const mavlink_message_t *msg)
{
	mavlink_mission_count_t itemCount;
	mavlink_msg_mission_count_decode(msg, &itemCount);

    MAV_MISSION_TYPE mission_type;

    if (itemCount.count >= MAX_MISSION_ITEM_COUNT) {
        printf("handleMissionCount: too many mission items");
        manager->sendMissionAck(transferSysId, transferCompId, MAV_MISSION_NO_SPACE);
        return;
    }
    
    transferSysId = msg->sysid;
    transferCompId = msg->compid;
    count = itemCount.count;
}

void
MavlinkMissionManager::MissionDownloadService::handleMissionRequest(uint8_t sysid, uint8_t compid, uint16_t seq)
{
	if (seq < MAX_MISSION_ITEM_COUNT) {
        mavlink_mission_request_t wpr;
        wpr.target_system = sysid;
        wpr.target_component = compid;
        wpr.seq = seq;
        mavlink_msg_mission_request_send_struct(mavlink->getChannel(), &wpr);
        printf("item request: %i \n", seq);
	} else {
        printf("ERROR: Waypoint index exceeds list capacity");
        manager->sendMissionAck(transferSysId, transferCompId, MAV_MISSION_NO_SPACE);
	}
}

void
MavlinkMissionManager::MissionDownloadService::handleMissionItem(const mavlink_message_t *msg)
{
	mavlink_mission_item_t wp;
	mavlink_msg_mission_item_decode(msg, &wp);

    // -> Save the mission item <-
    MspController::getInstance()->missionAddItem(wp);
}

//-------------------------------------------------------------
// Class MissionDownloadInit 
//-------------------------------------------------------------
void
MavlinkMissionManager::MissionDownloadService::MissionDownloadInit::entry() { 
    // initialize 
    context->retries = 0;
    context->count = 0;
    context->seq = 1;
    context->transferSysId = 0;
    context->transferCompId = 0;
    context->repeatCounter = 0;
}

void
MavlinkMissionManager::MissionDownloadService::MissionDownloadInit::handleMessage(const mavlink_message_t *msg) { 
    if (msg->msgid == MAVLINK_MSG_ID_MISSION_COUNT) { 
        
        // Send error if a mission is active (stop mission and delete mission)
        if (MspController::getInstance()->missionIsActive()) {
            context->manager->sendMissionAck(context->transferSysId, context->transferCompId, MAV_MISSION_ERROR);
            context->setState(&context->missionDownloadInit);
        }
        else {
            // Start with mission download
            context->handleMissionCount(msg);
            context->handleMissionRequest(context->transferSysId, context->transferCompId, context->seq);
            context->setState(&context->missionDownloadItem);
        }
    }
}

void
MavlinkMissionManager::MissionDownloadService::MissionDownloadInit::run() { }

//-------------------------------------------------------------
// Class MissionDownloadItemInt 
//-------------------------------------------------------------

void
MavlinkMissionManager::MissionDownloadService::MissionDownloadItem::handleMessage(const mavlink_message_t *msg) { 
    if (msg->msgid == MAVLINK_MSG_ID_MISSION_ITEM) {

        mavlink_mission_item_t wp;
        mavlink_msg_mission_item_decode(msg, &wp);
        printf("item received: %i \n", wp.seq);

        // check item sequence number, musst be equal than the requested item
        if (wp.seq == context->seq) {
            context->handleMissionItem(msg);
            context->seq++;
        }
        else {
            // wrong sequence number received, retry MAX_RETRIES times
            printf("wrong sequence number received: %i \n", context->seq);

            if (MAV_MAX_RETRIES < context->retries) {
                ++context->retries;
                context->manager->sendMissionAck(context->transferSysId, context->transferCompId, MAV_MISSION_INVALID_SEQUENCE);
                context->setState(&context->missionDownloadInit);
                return;
            }
        }

        if (context->seq <= context->count) {
            // send next item request
            context->handleMissionRequest(context->transferSysId, context->transferCompId, context->seq);
        }
        else {
            // send mission acknowledge
            context->setState(&context->missionDownloadEnd);
        }
    }
}

void
MavlinkMissionManager::MissionDownloadService::MissionDownloadItem::run() { 

    if ((microsSinceEpoch() - context->mavlink->getSendTime()) > context->sendResponseTimeout) {
        if (context->repeatCounter < MAV_MAX_RETRIES){
            context->handleMissionRequest(context->transferSysId, context->transferCompId, context->seq);
            ++context->repeatCounter;
        }
        else{
            printf("max retries reached");
            context->manager->sendMissionAck(context->transferSysId, context->transferCompId, MAV_MISSION_ERROR);
            context->setState(&context->missionDownloadInit);
            return;
        }
    }
}

//-------------------------------------------------------------
// Class MissionDownloadEnd 
//-------------------------------------------------------------
void
MavlinkMissionManager::MissionDownloadService::MissionDownloadEnd::entry() { 
    
    // Activate new mission
    context->manager->sendMissionAck(context->transferSysId, context->transferCompId, MAV_MISSION_ACCEPTED);
    context->setState(&context->missionDownloadInit);
}