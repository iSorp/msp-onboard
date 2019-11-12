#include <stdio.h>
#include <list> 
#include <iterator> 

#include "mav_mavlink.h"
#include "helper.h"

static std::list<Mavlink*> instanceList;
static std::mutex send_lock;

//-------------------------------------------------------------
// Mavlink helper interface
//-------------------------------------------------------------
mavlink_system_t mavlink_system = {
	SYSID,
	COMPID
};


/****** -> send_lock.lock();*****/
void mavlink_start_uart_send(mavlink_channel_t chan, int length)
{
	send_lock.lock();

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
	send_lock.unlock();
}
/****** -> send_lock.unlock();*****/



void mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t *ch, int length)
{
	Mavlink *m = Mavlink::get_instance(chan);

	if (m != nullptr) {
		m->sendBytes(ch, length);
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
        handleMessages();
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

void Mavlink::runServices() {
    // handle timeouts and resets
    message_manager.run();
    command_manager.run();
    mission_manager.run();
    ftp_manager.run();
}

void
Mavlink::handle_message(mavlink_message_t *msg)
{
    message_manager.handle_message(msg);
    command_manager.handle_message(msg);
    mission_manager.handle_message(msg);
    ftp_manager.handle_message(msg);
}

/*
*   Set command result after command returned "MSP_PROGRESS"
*/
void 
Mavlink::setCmdResult(EResult result) {
	command_manager.setCmdResult(result);
}