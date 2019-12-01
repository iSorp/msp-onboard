#include <poll.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include "mav_mavlink.h"

class MavlinkUDP : public Mavlink {
    public: 
        MavlinkUDP(int _src_port, int _remote_port) :
            local_port(_src_port), remote_port(_remote_port) { }
        ~MavlinkUDP();

        void beginSend() override { }
        void sendBytes(const uint8_t *buf, unsigned packet_len) override;
        int sendPacket() override;

    protected:
        void init() override;
        void runService() override;

    private:
        bool initialized;
        int loc_socket_fd = -1;
        int	rmt_socket_fd = -1;
        sockaddr_in	loc_addr;
        sockaddr_in	rmt_addr;

        unsigned short local_port;
        unsigned short remote_port;
        const char* remote_ip;

	    struct pollfd fds[1] = {};
        uint8_t buf[2048];
        
	    struct sockaddr_in srcaddr = {};
	    socklen_t addrlen = sizeof(srcaddr);
};