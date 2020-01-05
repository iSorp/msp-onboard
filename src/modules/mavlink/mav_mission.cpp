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
MavlinkMissionManager::handleMessages(const mavlink_message_t *msg) {
    switch (msg->msgid) {

        // Start mission item upload (Creates a new mission)
        case MAVLINK_MSG_ID_MISSION_COUNT:
            spdlog::info("MavlinkMissionManager::handleMessages, start mission download");
            missionDownloadService.setState(&missionDownloadService.missionDownloadInit);
            break;

      case MAVLINK_MSG_ID_MISSION_CURRENT:
            spdlog::info("MavlinkMissionManager::handleMessages, return current item");
            missionCurrent();
            break;

        // stops and deletes the current mission
        case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
            spdlog::info("MavlinkMissionManager::handleMessages, mission delete");
            missionDelete(msg);
            break;
            
        default:
            break;
    }
    missionDownloadService.getState()->handleMessage(msg);
}

void
MavlinkMissionManager::missionDelete(const mavlink_message_t *msg) {
    mavlink_mission_clear_all_t msgCa;
    mavlink_msg_mission_clear_all_decode(msg, &msgCa);
    
    EResult res = MspController::getInstance()->missionDelete();
    if (res == EResult::MSP_SUCCESS) {
        sendMissionAck(msg->sysid, msg->compid, MAV_MISSION_ACCEPTED);
    }
    else {
        sendMissionAck(msg->sysid, msg->compid, MAV_MISSION_ERROR);
    }
}

void
MavlinkMissionManager::missionCurrent() {
    mavlink_mission_current_t item;
    item.seq = MspController::getInstance()->getCurrentWp();
	mavlink_msg_mission_current_send_struct(mavlink->getChannel(), &item);
}

void
MavlinkMissionManager::sendMissionAck(uint8_t sysid, uint8_t compid, uint8_t type) {
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

    if (itemCount.count >= MAX_MISSION_ITEM_COUNT) {
        spdlog::error("MissionDownloadService::handleMissionCount, too many mission items");
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
        spdlog::debug("MissionDownloadService::handleMissionRequest, item request");
	} else {
        spdlog::error("MissionDownloadService::handleMissionCount, too many mission items");
        manager->sendMissionAck(transferSysId, transferCompId, MAV_MISSION_NO_SPACE);
	}
}

void
MavlinkMissionManager::MissionDownloadService::handleMissionItem(const mavlink_message_t *msg)
{
	mavlink_mission_item_t wp;
	mavlink_msg_mission_item_decode(msg, &wp);

    // Store the mission item
    // mission items with the same coordinates define actions, only the first item is used as waypoint
    if (MspController::getInstance()->getMissionItemCount() > 0) {
        if (wp.x != lastRcvItem.x || wp.y != lastRcvItem.y || wp.z != lastRcvItem.z) {
            ++wpIndex;
        }
    }

    MspController::getInstance()->missionAddItem(wpIndex, wp);
    lastRcvItem = wp;
}

//-------------------------------------------------------------
// Class MissionDownloadInit 
//-------------------------------------------------------------
void
MavlinkMissionManager::MissionDownloadService::MissionDownloadInit::entry() { 

    // initialize 
    context->wpIndex = 0;
    context->retries = 0;
    context->count = 0;
    context->seq = 0;
    context->transferSysId = 0;
    context->transferCompId = 0;
    context->repeatCounter = 0;
    context->lastRcvItem = {};  
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
            // first delete current mission
            MspController::getInstance()->missionDelete();

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
        spdlog::debug("MissionDownloadItem::handleMessage, item received: " + std::to_string(wp.seq));

        // check item sequence number, musst be equal than the requested item
        if (wp.seq == context->seq) {
            context->handleMissionItem(msg);
            context->seq++;
        }
        else {
            // wrong sequence number received, retry MAX_RETRIES times
            spdlog::warn("MissionDownloadItem::handleMessage, wrong sequence number received: " + std::to_string(context->seq));
            if (MAV_MAX_RETRIES < context->retries) {
                ++context->retries;
                context->manager->sendMissionAck(context->transferSysId, context->transferCompId, MAV_MISSION_INVALID_SEQUENCE);
                context->setState(&context->missionDownloadInit);
                return;
            }
        }

        if (context->seq < context->count) {
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
            spdlog::error("MissionDownloadItem::run, max retries reached: " + context->seq);
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
    
    spdlog::info("MissionDownloadService::entry, mission download success");

    // Activate new mission
    context->manager->sendMissionAck(context->transferSysId, context->transferCompId, MAV_MISSION_ACCEPTED);
    context->setState(&context->missionDownloadInit);
}