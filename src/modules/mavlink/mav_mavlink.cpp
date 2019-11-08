#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <limits.h>
#include <sys/time.h> 
#include <time.h>
#include <arpa/inet.h>
#include <list> 
#include <iterator> 

#include "mav_mavlink.h"
#include "helper.h"


static std::list<Mavlink*> instanceList;

//-------------------------------------------------------------
// Mavlink helper interface
//-------------------------------------------------------------
mavlink_system_t mavlink_system = {
	SYSID,
	COMPID
};

void mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t *ch, int length)
{
	Mavlink *m = Mavlink::get_instance(chan);

	if (m != nullptr) {
		m->sendBytes(ch, length);
	}
}

void mavlink_start_uart_send(mavlink_channel_t chan, int length)
{
	Mavlink *m = Mavlink::get_instance(chan);

	if (m != nullptr) {
		(void)m->beginSend();
	}
}

void mavlink_end_uart_send(mavlink_channel_t chan, int length)
{
	Mavlink *m = Mavlink::get_instance(chan);

	if (m != nullptr) {
		(void)m->sendPacket();
	}
}

mavlink_status_t *mavlink_get_channel_status(uint8_t channel)
{
	Mavlink *m = Mavlink::get_instance(channel);

	if (m != nullptr) {
		return m->getStatus();

	} else {
		return nullptr;
	}
}

mavlink_message_t *mavlink_get_channel_buffer(uint8_t channel)
{
	Mavlink *m = Mavlink::get_instance(channel);

	if (m != nullptr) {
		return m->getBuffer();

	} else {
		return nullptr;
	}
}

//-------------------------------------------------------------
// Abstract class Mavlink
//-------------------------------------------------------------
Mavlink::~Mavlink() {
    stopThread = true;
}

std::thread Mavlink::start() 
{
    instance_id = instanceList.size();
    set_channel();
    instanceList.push_back(this);
    init();

    std::thread t( [] (void *arg) {
            Mavlink *object = (Mavlink*)arg;
            object->run(); 
        }, this);

    return t;
} 

void
Mavlink::run()
{
    // Mavlink task loop
    while (!stopThread) {
        readMessage();
    }
}

/*
* Returns a pointer to a Mavlink instance depending on a id
* @param instance number
*/
Mavlink *
Mavlink::get_instance(int instance)
{
    for (Mavlink *inst : instanceList){
        if (instance == inst->get_instance_id()) {
			return inst;
		}
    }
	return nullptr;
}

void
Mavlink::set_channel()
{
	switch (instance_id) {
	case 0:
		channel = MAVLINK_COMM_0;
		break;

	case 1:
		channel = MAVLINK_COMM_1;
		break;

	default:
    	printf("only udp and dji channels");
		exit(EXIT_FAILURE);
		break;
	}
}

int
Mavlink::sendPacket()
{
    sendTime = microsSinceEpoch();
    return 0;
}


//-------------------------------------------------------------
// Class MavlinkUDP
//-------------------------------------------------------------
MavlinkUDP::~MavlinkUDP() {
    if (socket_fd >= 0) {    
		close(socket_fd);
		socket_fd = -1;
	}
}

void
MavlinkUDP::init(){
  memset(&loc_addr, 0, sizeof(loc_addr));
	loc_addr.sin_family = AF_INET;
	loc_addr.sin_addr.s_addr = INADDR_ANY;
	loc_addr.sin_port = htons(src_port);

    if ((socket_fd = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
		exit(EXIT_FAILURE);
	}

	if (-1 == bind(socket_fd,(struct sockaddr *)&loc_addr, sizeof(struct sockaddr)))
    {
		perror("error bind failed");
		close(socket_fd);
		exit(EXIT_FAILURE);
    } 

	if (fcntl(socket_fd, F_SETFL, O_NONBLOCK | O_ASYNC) < 0)
    {
		fprintf(stderr, "error setting nonblocking: %s\n", strerror(errno));
		close(socket_fd);
		exit(EXIT_FAILURE);
    }

    memset(&src_addr, 0, sizeof(src_addr));
    src_addr.sin_family = AF_INET;
    inet_aton(remote_ip, &src_addr.sin_addr);
    src_addr.sin_port = htons(remote_port);

    // add the UDP Socket to the event loop file descriptor 
    fds[0].fd = get_socket_fd();
    fds[0].events = POLLIN;
}

void
MavlinkUDP::sendBytes(const uint8_t *buf, unsigned packet_len)
{
    if (send_buf_len + packet_len < sizeof(send_buf) / sizeof(send_buf[0])) {
        memcpy(&send_buf[send_buf_len], buf, packet_len);
        send_buf_len += packet_len;
    }
}

int
MavlinkUDP::sendPacket()
{
    Mavlink::sendPacket();
	if (send_buf_len == 0) {
		return 0;
	}

    int ret = sendto(socket_fd, send_buf, send_buf_len, 0, 
        (struct sockaddr *)&src_addr, sizeof(src_addr));
    
    send_buf_len = 0;
	return ret;
}
