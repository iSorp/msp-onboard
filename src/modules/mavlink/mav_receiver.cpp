
#include <stdio.h>
#include <pthread.h>
#include <limits.h>
#include <poll.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include "defines.h"
#include "mav_mavlink.h"

//-------------------------------------------------------------
// Abstract class Mavlink
//-------------------------------------------------------------

void Mavlink::readMessage() {
    // handle timeouts and resets
    mission_manager.run();
    ftp_manager.run();
}


void
Mavlink::handle_message_heartbeat(mavlink_message_t *msg)
{
	if (getChannel() < (mavlink_channel_t)ORB_MULTI_MAX_INSTANCES) {
		mavlink_heartbeat_t hb;
		mavlink_msg_heartbeat_decode(msg, &hb);

        // response heartbeat
        mavlink_msg_heartbeat_send(getChannel(), MAV_TYPE_GENERIC, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
	}
}

void
Mavlink::handle_message(mavlink_message_t *msg)
{
	switch (msg->msgid) {

        case MAVLINK_MSG_ID_HEARTBEAT:
            handle_message_heartbeat(msg);
            break;

        default:
            mission_manager.handle_message(msg);
            ftp_manager.handle_message(msg);
            break;
	}
}


//-------------------------------------------------------------
// Class MavlinkUDP
//-------------------------------------------------------------
void
MavlinkUDP::readMessage() {

    // check for received data 
    if (poll(&fds[0], 1, timeout) > 0) {

        mavlink_message_t msg;
        mavlink_status_t status;
        int nread = 0;

        if (fds[0].revents & POLLIN) {
            nread = recvfrom(get_socket_fd(), buf, sizeof(buf), 0, (struct sockaddr *)&srcaddr, &addrlen);
        }
        
        for (ssize_t i = 0; i < nread; i++) {
            if (mavlink_parse_char(getChannel(), buf[i], &msg, &status)) {
                handle_message(&msg);
            }
        }
    }
    
    Mavlink::readMessage();
}