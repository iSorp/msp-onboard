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
        //sendMissionAck(transferSysId, transferCompId, MAV_MISSION_NO_SPACE);
        return;
    }
    
    switch (mission_type) {
        case MAV_MISSION_TYPE_MISSION:
            // TODO create mission
            break;

        default:
            //printf("handleMissionCount: wrong misson");
            //Wrong missiong type
            break;
    }
    context->transferSysId = msg->sysid;
    context->transferCompId = msg->compid;
    context->count = itemCount.count;
}

void
MavlinkMissionManager::MissionState::handleMissionRequest(uint8_t sysid, uint8_t compid, uint16_t seq)
{
	if (seq < MAX_MISSION_ITEM_COUNT) {
        mavlink_mission_request_int_t wpr;
        wpr.target_system = sysid;
        wpr.target_component = compid;
        wpr.seq = seq;
        mavlink_msg_mission_request_int_send_struct(context->mavlink->getChannel(), &wpr);
	} else {
        printf("ERROR: Waypoint index exceeds list capacity");
        sendMissionAck(context->transferSysId, context->transferCompId, MAV_MISSION_ERROR);
	}
}

void
MavlinkMissionManager::MissionState::handleMissionItemInt(const mavlink_message_t *msg)
{
	mavlink_mission_item_t wp;
	mavlink_msg_mission_item_decode(msg, &wp);

    printf("item received");
    context->seq++;

    // Save the mission item
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

        context->setState(&context->missionUploadItemInt);
    }
}

void
MavlinkMissionManager::MissionUploadInit::run() {

}

//-------------------------------------------------------------
// Class MissionUploadItemInt 
//-------------------------------------------------------------

void
MavlinkMissionManager::MissionUploadItemInt::handleMessage(const mavlink_message_t *msg) { 
    if (msg->msgid == MAVLINK_MSG_ID_MISSION_ITEM_INT) {
        handleMissionItemInt(msg);
        if (context->seq <= context->count) {
            handleMissionRequest(context->transferSysId, context->transferCompId, context->seq);
        }
        else {
            context->setState(&context->missionUploadEnd);
        }
    }
}

void
MavlinkMissionManager::MissionUploadItemInt::run() { 

    if ((microsSinceEpoch() - context->mavlink->getSendTime()) > context->sendResponseTimeout) {
        if (context->repeatCounter < MAX_RETRIES){
            handleMissionRequest(context->transferSysId, context->transferCompId, context->seq);
            ++context->repeatCounter;
        }
        else{
            printf("max retries rached");
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