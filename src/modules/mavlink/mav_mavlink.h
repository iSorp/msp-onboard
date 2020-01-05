/**
    @file mav_mavlink.h
    @brief

    @Copyright (c) 2019 Samuel Ackermann, Simon Wälti
*/        


#pragma once

#include <stdlib.h>
#include <thread>
#include <queue>
#include <mutex>

#include "mavlink_bridge_header.h"
#include "mav_message.h"
#include "mav_command.h"
#include "mav_mission.h"
#include "mav_ftp.h"

struct Mavlink
{
    using ControllerErrorCallback = void (*)(int msg);

    public:
        Mavlink() : 
        message_manager(this),
        command_manager(this),
        mission_manager(this),
        ftp_manager(this) 
        {
            services.push_back(&message_manager);     
            services.push_back(&command_manager);  
            services.push_back(&mission_manager);  
            services.push_back(&ftp_manager);  
         }

        virtual ~Mavlink();

        // Setter/getter
        mavlink_channel_t getChannel() const { return channel; }
        uint64_t getSendTime() {return sendTime; } 
        virtual void setCmdResult(EResult result);

        // Funcitons
        std::thread start();
        void stop() { stopThread = true; }
        
        static Mavlink *get_instance(int instance);

	    mavlink_message_t *getBuffer() { return &mavlinkBuffer; }
	    mavlink_status_t *getStatus() { return &mavlinkStatus; }

	    int	get_system_id() const { return mavlink_system.sysid; }
	    int	get_component_id() const { return mavlink_system.compid; }

        virtual void beginSend() = 0;
        virtual void sendBytes(const uint8_t *buf, unsigned packet_len) = 0;
        virtual int sendPacket() = 0;

    protected:
        const int timeout = 10; // ms
        uint8_t send_buf[MAVLINK_MAX_PACKET_LEN];
        unsigned send_buf_len=0;
        std::vector<MavlinkServiceManager*> services;

        // Virtual functions
        virtual void init() = 0;
        virtual void runService();
        virtual void handleMessages(mavlink_message_t *msg);
        
        // Microservices
        MavlinkMessageManager message_manager;
        MavlinkCommandManager command_manager;
        MavlinkMissionManager mission_manager;
        MavlinkFtpManager ftp_manager;

    private:
        // Variables
        Mavlink *next;

        bool stopThread = false;
        int	instance_id;
        uint64_t sendTime = 0;

        mavlink_channel_t channel=MAVLINK_COMM_0;
        mavlink_message_t mavlinkBuffer {};
	    mavlink_status_t mavlinkStatus {};


        // Setter/getter
        int	getInstanceId() const { return instance_id; };

        // Funcitons
        void run();
        void setChannel();
        static int instanceCount();
};
