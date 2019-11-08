
#include <stdio.h>
#include <pthread.h>
#include <limits.h>
#include <poll.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <mutex>

#include "defines.h"
#include "mav_mavlink.h"
#include "dji_mavlink.h"


std::mutex message_lock;

//-------------------------------------------------------------
// Class MavlinkUDP
//-------------------------------------------------------------
void
MavlinkDJI::readMessage()
{
    message_lock.lock();

    if (bufferLength <= 0)
        return;

    mavlink_message_t msg;
    mavlink_status_t status;

    for (ssize_t i = 0; i < bufferLength; i++) {
        if (mavlink_parse_char(getChannel(), buffer[i], &msg, &status)) {
            handle_message(&msg); 
        }
    }
    Mavlink::readMessage();
    delete this->buffer;
    
    message_lock.unlock();
    usleep(timeout*1000);
}

void MavlinkDJI::setBuffer(uint8_t* data, uint8_t len){

    message_lock.lock();

    bufferLength = len;
    this->buffer = new uint8_t[len];
    memcpy(this->buffer, data, len); 

    message_lock.unlock();
}



