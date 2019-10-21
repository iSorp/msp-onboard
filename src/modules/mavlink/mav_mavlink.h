#pragma once

#include <stdlib.h>
#include <poll.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <thread>

#include "mavlink_bridge_header.h"
#include "mav_mission.h"

struct Mavlink
{
    public:
        Mavlink() : mission_manager(this) { }

        virtual ~Mavlink();

        // Setter/getter
        mavlink_channel_t getChannel() const { return channel; }
        uint64_t getSendTime() {return sendTime; } 

        // Funcitons
        std::thread start();
        void stop() { stopThread = true; }
        
        // Static functions
        static Mavlink *get_instance(int instance);

	    mavlink_message_t *getBuffer() { return &mavlinkBuffer; }
	    mavlink_status_t *getStatus() { return &mavlinkStatus; }

	    int	get_system_id() const { return mavlink_system.sysid; }
	    int	get_component_id() const { return mavlink_system.compid; }

        virtual void beginSend() = 0;
        virtual void sendBytes(const uint8_t *buf, unsigned packet_len) = 0;
        virtual int sendPacket() = 0;

    protected:
        

        // Virtual functions
        virtual void init() = 0;
        virtual void readMessage() = 0;

        // Message functions
        void handle_message(mavlink_message_t *msg);
        void handle_message_heartbeat(mavlink_message_t *msg);
        
        // Microservices
        MavlinkMissionManager mission_manager;

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


class MavlinkUDP : public Mavlink {
    public: 
        MavlinkUDP(int _src_port, int _remote_port, const char* _remote_ip) :
            src_port(_src_port),
            remote_port(_remote_port)
            { 
                remote_ip = _remote_ip;
            }
        ~MavlinkUDP();

        void beginSend() override { }
        void sendBytes(const uint8_t *buf, unsigned packet_len) override;
        int sendPacket() override;

    protected:
        void init() override;
        void readMessage() override;

    private:
        int	socket_fd = -1;
        sockaddr_in	loc_addr;
        sockaddr_in	src_addr;
        unsigned short src_port;
        unsigned short remote_port;
        const char* remote_ip;
	    const int timeout = 10;
	    struct pollfd fds[1] = {};
        uint8_t buf[2048];
        
	    struct sockaddr_in srcaddr = {};
	    socklen_t addrlen = sizeof(srcaddr);
        uint8_t network_buf[MAVLINK_MAX_PACKET_LEN];
        unsigned network_buf_len=0;

        // Setter/getter
        int get_socket_fd() { return socket_fd; };
};