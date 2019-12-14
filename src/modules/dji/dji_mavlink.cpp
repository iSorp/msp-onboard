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
    if (send_buf_len + packet_len < sizeof(send_buf) / sizeof(send_buf[0])) {
        memcpy(&send_buf[send_buf_len], buf, packet_len);
        send_buf_len += packet_len;
    }
}

int
MavlinkDJI::sendPacket()
{
    Mavlink::sendPacket();
	if (send_buf_len == 0 || sendDataCallbackHandler.callback == NULL) {
		return 0;
	}

    sendDataCallbackHandler.callback(send_buf, send_buf_len, sendDataCallbackHandler.userData);
    int ret = send_buf_len;

    send_buf_len = 0;
	return ret;
}

void
MavlinkDJI::runService()
{
    usleep(timeout*1000);

    // Synchronizing message buffer from OSDK
    message_lock.lock();

    if (bufferLength > 0)
    {
        mavlink_message_t msg;
        mavlink_status_t status;

        for (ssize_t i = 0; i < bufferLength; i++) {
            if (mavlink_parse_char(getChannel(), buffer[i], &msg, &status)) {
                Mavlink::handleMessages(&msg);
            }
        }
        if (this->buffer) {
            bufferLength = 0;
            delete this->buffer;
            this->buffer = nullptr;
        }
    }
    Mavlink::runService();
    message_lock.unlock();
}

void 
MavlinkDJI::setBuffer(uint8_t* data, size_t len){

    message_lock.lock();

    bufferLength = len;
    this->buffer = new uint8_t[len];
    memcpy(this->buffer, data, len); 

    message_lock.unlock();
}


void 
MavlinkDJI::setSendDataCallback(SendDataCallback callback, void* userData) {
    this->sendDataCallbackHandler.callback = callback;
    this->sendDataCallbackHandler.userData = userData;
}