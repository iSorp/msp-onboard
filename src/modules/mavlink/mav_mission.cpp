#include <stdio.h>
#include "mav_mission.h"
#include "mav_mavlink.h"
#include "helper.h"



//-------------------------------------------------------------
// Class MavlinkMissionManager
//-------------------------------------------------------------

void 
MavlinkMissionManager::checkActiveMission()
{
    // TODO
}


void 
MavlinkMissionManager::activateMission()
{
    // TODO
}

void
MavlinkMissionManager::run() {
    missionUploader->getState()->run();
}


void
MavlinkMissionManager::handle_message(const mavlink_message_t *msg)
{
    switch (msg->msgid) {

        case MAVLINK_MSG_ID_MISSION_COUNT:
            missionUploader->setState(&missionUploader->missionUploadInit);
            break;

        default:
            break;
    }
    
    missionUploader->getState()->handleMessage(msg);
}

//-------------------------------------------------------------
// Class MissionState 
//-------------------------------------------------------------
void
MavlinkMissionManager::MissionState::handleMessage(const mavlink_message_t *msg) { }
void
MavlinkMissionManager::MissionState::entry() {}
void
MavlinkMissionManager::MissionState::run() {}
void
MavlinkMissionManager::MissionState::exit() { }

void
MavlinkMissionManager::MissionState::handleMissionCount(const mavlink_message_t *msg)
{
	mavlink_mission_count_t itemCount;
	mavlink_msg_mission_count_decode(msg, &itemCount);

    MAV_MISSION_TYPE mission_type;

    if (itemCount.count >= MAX_MISSION_ITEM_COUNT) {
        printf("handleMissionCount: too many mission items");
        sendMissionAck(context->transferSysId, context->transferCompId, MAV_MISSION_NO_SPACE);
        return;
    }
    

    context->transferSysId = msg->sysid;
    context->transferCompId = msg->compid;
    context->count = itemCount.count;
}

void
MavlinkMissionManager::MissionState::handleMissionRequest(uint8_t sysid, uint8_t compid, uint16_t seq)
{
	if (seq < MAX_MISSION_ITEM_COUNT) {
        mavlink_mission_request_t wpr;
        wpr.target_system = sysid;
        wpr.target_component = compid;
        wpr.seq = seq;
        mavlink_msg_mission_request_send_struct(context->mavlink->getChannel(), &wpr);
        printf("item request: %i \n", seq);
	} else {
        printf("ERROR: Waypoint index exceeds list capacity");
        sendMissionAck(context->transferSysId, context->transferCompId, MAV_MISSION_NO_SPACE);
	}
}

void
MavlinkMissionManager::MissionState::handleMissionItem(const mavlink_message_t *msg)
{
	mavlink_mission_item_t wp;
	mavlink_msg_mission_item_decode(msg, &wp);

    // ready for next item


    // -> Save the mission item <-
    
}

void
MavlinkMissionManager::MissionState::sendMissionAck(uint8_t sysid, uint8_t compid, uint8_t type)
{
	mavlink_mission_ack_t wpa;
	wpa.target_system    = sysid;
	wpa.target_component = compid;
	wpa.type = type;

	mavlink_msg_mission_ack_send_struct(context->mavlink->getChannel(), &wpa);
}

//-------------------------------------------------------------
// Class MissionUploadInit 
//-------------------------------------------------------------
void
MavlinkMissionManager::MissionUploadInit::entry() { 
    // initialize 
    context->retries = 0;
    context->count = 0;
    context->seq = 1;
    context->transferSysId = 0;
    context->transferCompId = 0;
    context->repeatCounter = 0;
}

void
MavlinkMissionManager::MissionUploadInit::handleMessage(const mavlink_message_t *msg) { 
    if (msg->msgid == MAVLINK_MSG_ID_MISSION_COUNT) { 
        handleMissionCount(msg);
        handleMissionRequest(context->transferSysId, context->transferCompId, context->seq);

        context->setState(&context->missionUploadItem);
    }
}

void
MavlinkMissionManager::MissionUploadInit::run() { }

//-------------------------------------------------------------
// Class MissionUploadItemInt 
//-------------------------------------------------------------

void
MavlinkMissionManager::MissionUploadItem::handleMessage(const mavlink_message_t *msg) { 
    if (msg->msgid == MAVLINK_MSG_ID_MISSION_ITEM) {

        mavlink_mission_item_t wp;
        mavlink_msg_mission_item_decode(msg, &wp);
        printf("item received: %i \n", wp.seq);

        // check item sequence number, musst be equal than the requested item
        if (wp.seq == context->seq) {
            handleMissionItem(msg);
            context->seq++;
        }
        else {
            // wrong sequence number received, retry MAX_RETRIES times
            printf("wrong sequence number received: %i \n", context->seq);

            if (MAV_MAX_RETRIES < context->retries) {
                ++context->retries;
                sendMissionAck(context->transferSysId, context->transferCompId, MAV_MISSION_INVALID_SEQUENCE);
                context->setState(&context->missionUploadInit);
                return;
            }
        }

        if (context->seq <= context->count) {
            // send next item request
            handleMissionRequest(context->transferSysId, context->transferCompId, context->seq);
        }
        else {
            // send mission acknowledge
            context->setState(&context->missionUploadEnd);
        }
    }
}

void
MavlinkMissionManager::MissionUploadItem::run() { 

    if ((microsSinceEpoch() - context->mavlink->getSendTime()) > context->sendResponseTimeout) {
        if (context->repeatCounter < MAV_MAX_RETRIES){
            handleMissionRequest(context->transferSysId, context->transferCompId, context->seq);
            ++context->repeatCounter;
        }
        else{
            printf("max retries reached");
            sendMissionAck(context->transferSysId, context->transferCompId, MAV_MISSION_ERROR);
            context->setState(&context->missionUploadInit);
            return;
        }
    }
}


//-------------------------------------------------------------
// Class MissionUploadEnd 
//-------------------------------------------------------------
void
MavlinkMissionManager::MissionUploadEnd::entry() { 
    sendMissionAck(context->transferSysId, context->transferCompId, MAV_MISSION_ACCEPTED);

    context->setState(&context->missionUploadInit);
}