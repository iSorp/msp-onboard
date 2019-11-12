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
    typedef void (*ControllerErrorCallback)(int msg);

    struct queue_message_t{
        mavlink_message_t msg;
        size_t len;
        ControllerErrorCallback errorCallback;
    };

    public:
        Mavlink() : 
        message_manager(this),
        command_manager(this),
        mission_manager(this),
        ftp_manager(this) 
        { }

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
        std::queue<queue_message_t> sendQueue;

        // functions
        void runServices();

        // Message functions
        void handle_message(mavlink_message_t *msg);
        
        // Virtual functions
        virtual void init() = 0;
        virtual void handleMessages() = 0;
        
        // Microservices
        MavlinkMessageManager message_manager;
        MavlinkCommandManager command_manager;
        MavlinkMissionManager mission_manager;
        MavlinkFtpManager ftp_manager;

    private:
        // Variables
        Mavlink *next;

        bool stopThread;
        int	instance_id;

        mavlink_channel_t channel=MAVLINK_COMM_0;
        mavlink_message_t mavlinkBuffer {};
	    mavlink_status_t mavlinkStatus {};

        uint64_t sendTime = 0;

        // Setter/getter
        int	get_instance_id() const { return instance_id; };

        // Static functions
        static int instance_count();

        // Funcitons
        void run();
        void set_channel();
};
