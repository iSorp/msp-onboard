/**
    @file mav_message.h
    @brief

    @Copyright (c) 2019 Samuel Ackermann, Simon Wälti
*/    

#pragma once

#include "mav_service.h"
#include "defines.h"


struct Mavlink;

class MavlinkMessageManager : public MavlinkServiceManager
{
    public:
        MavlinkMessageManager(Mavlink *mavlink) : MavlinkServiceManager(mavlink) { }

        void handleMessages(const mavlink_message_t *msg) override;
        void run() override;

    private:
        void handleMessages_heartbeat(const mavlink_message_t *msg);
        void sendHeartbeat();

        uint64_t heartbeatSendTime;
};


