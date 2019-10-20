#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <pthread.h>
#include <limits.h>
#include <sys/time.h>
#include <time.h>
#include <arpa/inet.h>

#include "utlist.h"
#include "mav_mavlink.h"
#include "helper.h"


#define SRC_ADDR "127.0.0.1"

// mavlink instances (channels)
static Mavlink *_mavlink_instances = nullptr;

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
#ifdef MAVLINK_PRINT_PACKETS

		for (unsigned i = 0; i < length; i++) {
			printf("%02x", (unsigned char)ch[i]);
		}

#endif
	}
}

void mavlink_start_uart_send(mavlink_channel_t chan, int length)
{
	Mavlink *m = Mavlink::get_instance(chan);

	if (m != nullptr) {
		(void)m->beginSend();
#ifdef MAVLINK_PRINT_PACKETS
		printf("START PACKET (%u): ", (unsigned)chan);
#endif
	}
}

void mavlink_end_uart_send(mavlink_channel_t chan, int length)
{
	Mavlink *m = Mavlink::get_instance(chan);

	if (m != nullptr) {
		(void)m->sendPacket();
#ifdef MAVLINK_PRINT_PACKETS
		printf("\n");
#endif
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
// Public scope functions
//-------------------------------------------------------------
void 
start_mavlink_task() {

    pthread_t tid;
    pthread_attr_t attr;
    pthread_attr_init(&attr);

    pthread_attr_setstacksize(&attr, (size_t)PTHREAD_STACK_MIN);
    pthread_create(&tid, &attr, Mavlink::startTask, NULL);
}

//-------------------------------------------------------------
// Class Mavlink
//-------------------------------------------------------------
Mavlink::~Mavlink() {
    if (socket_fd >= 0) {
		close(socket_fd);
		socket_fd = -1;
	}
}

void *
Mavlink::startTask(void *arg)
{
	Mavlink *instance = new Mavlink();
	instance->mainTask();
}

void
Mavlink::mainTask()
{
	set_instance_id();
	set_channel();
    init_udp();

    LL_APPEND(_mavlink_instances, this);

    // add the UDP Socket to the event loop file descriptor 
    fds[0].fd = get_socket_fd();
    fds[0].events = POLLIN;

    // Mavlink task loop
    while (true) {
        readMessage();
    }
}

int
Mavlink::instance_count()
{
	size_t inst_index = 0;
	Mavlink *inst;
	LL_FOREACH(::_mavlink_instances, inst) {
		inst_index++;
	}
	return inst_index;
}

void
Mavlink::set_instance_id() {
	instance_id = Mavlink::instance_count();
}
/*
* Returns a pointer to a Mavlink instance depending on a id
* @param instance number
*/
Mavlink *
Mavlink::get_instance(int instance)
{
	Mavlink *inst;
	LL_FOREACH(::_mavlink_instances, inst) {
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

void
Mavlink::sendBytes(const uint8_t *buf, unsigned packet_len)
{
	size_t ret = -1;

    if (network_buf_len + packet_len < sizeof(network_buf) / sizeof(network_buf[0])) {
        memcpy(&network_buf[network_buf_len], buf, packet_len);
        network_buf_len += packet_len;
        ret = packet_len;
    }
}

int
Mavlink::sendPacket()
{
    sendTime = microsSinceEpoch();

	if (network_buf_len == 0) {
		return 0;
	}

    int ret = sendto(socket_fd, network_buf, network_buf_len, 0, 
        (struct sockaddr *)&src_addr, sizeof(src_addr));
    
    network_buf_len = 0;
	return ret;
}

void
Mavlink::init_udp()
{
    memset(&loc_addr, 0, sizeof(loc_addr));
	loc_addr.sin_family = AF_INET;
	loc_addr.sin_addr.s_addr = INADDR_ANY;
	loc_addr.sin_port = htons(network_port);

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
    inet_aton(SRC_ADDR, &src_addr.sin_addr);
    src_addr.sin_port = htons(remote_port);
}
