/**
    @file mav_message.cpp
    @brief

    @Copyright (c) 2019 Samuel Ackermann, Simon Wälti
*/        


#include "helper.h"
#include "controller.h"
#include "mav_mavlink.h"
#include "mav_message.h"


//-------------------------------------------------------------
// Class MavlinkCommandManager
//-------------------------------------------------------------

void
MavlinkMessageManager::run() {

    // send heartbeat interval
    if ((microsSinceEpoch() - heartbeatSendTime) > MAV_HEARTBEAT_INTERVAL) {
        sendHeartbeat();
    }
}

void
MavlinkMessageManager::handleMessages(const mavlink_message_t *msg)
{
    switch (msg->msgid)
    {
    case MAVLINK_MSG_ID_HEARTBEAT:
        handleMessages_heartbeat(msg);
        break;
    
    default:
        break;
    }
}

void
MavlinkMessageManager::handleMessages_heartbeat(const mavlink_message_t *msg)
{
	if (mavlink->getChannel() < (mavlink_channel_t)ORB_MULTI_MAX_INSTANCES) {
		mavlink_heartbeat_t hb;
		mavlink_msg_heartbeat_decode(msg, &hb);
        
        /// handle heartbeat from gcs
	}
}

void
MavlinkMessageManager::sendHeartbeat()
{
    // send heartbeat
    mavlink_msg_heartbeat_send(mavlink->getChannel(), 
        MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY, 
        MspController::getInstance()->getMavMode(), 
        0, 
        MspController::getInstance()->getMavState());

        heartbeatSendTime = microsSinceEpoch();
}



