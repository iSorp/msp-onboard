#pragma once

#include <stdlib.h>
#include <poll.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <thread>

#include "mavlink_bridge_header.h"
#include "mav_mavlink.h"
#include "mav_mission.h"


class MavlinkDJI : public Mavlink {

    // Typ for callback function which the data actually sends to the receiver
    using SendDataCallback = void (*)(uint8_t* data, uint8_t len);

    public: 
        MavlinkDJI() { }
        
        ~MavlinkDJI() {}

        SendDataCallback sendDataCallback = nullptr; 

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
};