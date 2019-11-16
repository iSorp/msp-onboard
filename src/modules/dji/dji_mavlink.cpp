#include <mutex>

#include "defines.h"
#include "helper.h"
#include "mav_mavlink.h"
#include "dji_mavlink.h"

std::mutex message_lock;


//-------------------------------------------------------------
// Class MavlinkDJI
//-------------------------------------------------------------

void
MavlinkDJI::sendBytes(const uint8_t* buf, unsigned packet_len)
{
    size_t ret = -1;
    if (send_buf_len + packet_len < sizeof(send_buf) / sizeof(send_buf[0])) {
        memcpy(&send_buf[send_buf_len], buf, packet_len);
        send_buf_len += packet_len;
        ret = packet_len;
    }
}

int
MavlinkDJI::sendPacket()
{
    Mavlink::sendPacket();
	if (send_buf_len == 0 || sendDataCallback == NULL) {
		return 0;
	}

    sendDataCallback(send_buf, send_buf_len);
    int ret = send_buf_len;

    send_buf_len = 0;
	return ret;
}

void
MavlinkDJI::handleMessages()
{
    // Synchronizing message buffer from OSDK
    message_lock.lock();

    if (bufferLength > 0)
    {
        mavlink_message_t msg;
        mavlink_status_t status;

        for (ssize_t i = 0; i < bufferLength; i++) {
            if (mavlink_parse_char(getChannel(), buffer[i], &msg, &status)) {
                handle_message(&msg); 
            }
        }
        delete this->buffer;
    }
    Mavlink::runServices();
    
    message_lock.unlock();
    usleep(timeout*1000);
}

void MavlinkDJI::setBuffer(uint8_t* data, size_t len){

    message_lock.lock();

    bufferLength = len;
    this->buffer = new uint8_t[len];
    memcpy(this->buffer, data, len); 

    message_lock.unlock();
}