/**
    @file dji_mavlink.h
    @brief

    @Copyright (c) 2019 Samuel Ackermann, Simon Wälti
*/

#pragma once

#include <stdlib.h>
#include <poll.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <thread>

#include "mavlink_bridge_header.h"
#include "mav_mavlink.h"
#include "mav_mission.h"


// Typ for callback function which the data actually sends to the receiver
typedef void (*SendDataCallback)(uint8_t* data, size_t len, void* userData);

typedef struct DataCallbackHandler{
    SendDataCallback callback;
    void* userData;
}DataCallbackHandler;

class MavlinkDJI : public Mavlink {
    public: 
        MavlinkDJI() {}
        
        ~MavlinkDJI() {}

        void setSendDataCallback(SendDataCallback callback, void* userData);

        void setBuffer(uint8_t* data, size_t len);
        void beginSend() override { }
        void sendBytes(const uint8_t* buf, unsigned packet_len) override;
        int sendPacket() override;

    protected:
        void init() override { };
        void runService() override;

    private:
        uint8_t* buffer;
        int bufferLength;
        DataCallbackHandler sendDataCallbackHandler;
};