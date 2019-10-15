#include "mav_mavlink.h"
#include "mav_mission.h"

MavlinkMissionManager::MavlinkMissionManager(Mavlink *mavlink) :
	_mavlink(mavlink)
{

}

void
MavlinkMissionManager::handle_message(const mavlink_message_t *msg)
{
	switch (msg->msgid) {
	case MAVLINK_MSG_ID_MISSION_ACK:
		//handle_mission_ack(msg);
		break;

	case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
		//handle_mission_set_current(msg);
		break;

	case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
		//handle_mission_request_list(msg);
		break;

	case MAVLINK_MSG_ID_MISSION_REQUEST:
		//handle_mission_request(msg);
		break;

	case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
		//handle_mission_request_int(msg);
		break;

	case MAVLINK_MSG_ID_MISSION_COUNT:
		//handle_mission_count(msg);
		break;

	case MAVLINK_MSG_ID_MISSION_ITEM:
		//handle_mission_item(msg);
		break;

	case MAVLINK_MSG_ID_MISSION_ITEM_INT:
		//handle_mission_item_int(msg);
		break;

	case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
		//handle_mission_clear_all(msg);
		break;

	default:
		break;
	}
}