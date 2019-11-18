#include <poll.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include "mav_mavlink.h"

class MavlinkUDP : public Mavlink {
    public: 
        MavlinkUDP(int _src_port, int _remote_port) :
            src_port(_src_port), remote_port(_remote_port) { }
        ~MavlinkUDP();

        void beginSend() override { }
        void sendBytes(const uint8_t *buf, unsigned packet_len) override;
        int sendPacket() override;

    protected:
        void init() override;
        void handleMessages() override;

    private:
        bool initialized;
        int	socket_fd = -1;
        sockaddr_in	loc_addr;
        sockaddr_in	src_addr;
        unsigned short src_port;
        unsigned short remote_port;
        const char* remote_ip;

	    struct pollfd fds[1] = {};
        uint8_t buf[2048];
        
	    struct sockaddr_in srcaddr = {};
	    socklen_t addrlen = sizeof(srcaddr);
};