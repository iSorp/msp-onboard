#include <stdlib.h>
#include <poll.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include "mavlink_bridge_header.h"
#include "mav_mission.h"

void start_mavlink_task();

class Mavlink
{
    public:
        Mavlink() : mission_manager(this) { }

        ~Mavlink();

        // Setter/getter
        mavlink_channel_t getChannel() const { return channel; }
        uint64_t getSendTime() {return sendTime; } 

        // Funcitons
        int sendPacket();
        void beginSend() { };
        void sendBytes(const uint8_t *buf, unsigned packet_len);

        // Static functions
        static void * startTask(void *arg);
        static Mavlink *get_instance(int instance);

	    mavlink_message_t *getBuffer() { return &mavlinkBuffer; }
	    mavlink_status_t *getStatus() { return &mavlinkStatus; }

	    int	get_system_id() const { return mavlink_system.sysid; }
	    int	get_component_id() const { return mavlink_system.compid; }


    private:
        // Variables
        Mavlink *next;

        int	instance_id;
        int	socket_fd = -1;
        sockaddr_in	loc_addr;
        sockaddr_in	src_addr;

	    const int timeout = 10;
	    struct pollfd fds[1] = {};
        uint8_t buf[2048];
        
	    struct sockaddr_in srcaddr = {};
	    socklen_t addrlen = sizeof(srcaddr);

        uint8_t network_buf[MAVLINK_MAX_PACKET_LEN];
        unsigned network_buf_len=0;

        unsigned short network_port=5001;
        unsigned short remote_port=5000;

        mavlink_channel_t channel=MAVLINK_COMM_0;
        mavlink_message_t mavlinkBuffer {};
	    mavlink_status_t mavlinkStatus {};

        uint64_t sendTime = 0;

        // Setter/getter
        int	get_instance_id() const { return instance_id; };
        int get_socket_fd() { return socket_fd; };

        // Static functions
        static int instance_count();

        // Funcitons
        void set_instance_id(); 
        void mainTask();
        void readMessage();

        void set_channel();
        void init_udp();

        // Message functions
        void handle_message(mavlink_message_t *msg);
        void handle_message_heartbeat(mavlink_message_t *msg);
        
        // Microservices
        MavlinkMissionManager mission_manager;
};