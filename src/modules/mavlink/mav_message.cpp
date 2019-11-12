
#include "helper.h"
#include "controller.h"
#include "mav_mavlink.h"
#include "mav_message.h"


//-------------------------------------------------------------
// Class MavlinkCommandManager
//-------------------------------------------------------------

void
MavlinkMessageManager::run() {
    
}

void
MavlinkMessageManager::handle_message(const mavlink_message_t *msg)
{
    switch (msg->msgid)
    {
    case MAVLINK_MSG_ID_HEARTBEAT:
        handle_message_heartbeat(msg);
        break;
    
    default:
        break;
    }
}

void
MavlinkMessageManager::handle_message_heartbeat(const mavlink_message_t *msg)
{
	if (mavlink->getChannel() < (mavlink_channel_t)ORB_MULTI_MAX_INSTANCES) {
		mavlink_heartbeat_t hb;
		mavlink_msg_heartbeat_decode(msg, &hb);

        // response heartbeat
        mavlink_msg_heartbeat_send(mavlink->getChannel(), MAV_TYPE_GENERIC, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
	}
}