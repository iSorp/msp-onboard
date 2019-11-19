
#include <string>
#include <arpa/inet.h>
#include <fcntl.h>

#include "mav_mavlink_udp.h"
#include "helper.h"

//-------------------------------------------------------------
// Class MavlinkUDP
//-------------------------------------------------------------
MavlinkUDP::~MavlinkUDP() {
    if (socket_fd >= 0) {    
		close(socket_fd);
		socket_fd = -1;
	}
}

void
MavlinkUDP::init(){

	loc_addr.sin_family      = AF_INET;
	loc_addr.sin_addr.s_addr = INADDR_ANY;
	loc_addr.sin_port        = htons(src_port);

    if ((socket_fd = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
        spdlog::error("MavlinkUDP::init, " + EXIT_FAILURE);
		exit(EXIT_FAILURE);
	}

	if (-1 == bind(socket_fd,(struct sockaddr *)&loc_addr, sizeof(struct sockaddr)))
    {
        spdlog::error("MavlinkUDP::init, error bind failed");
		close(socket_fd);
		exit(EXIT_FAILURE);
    } 

	if (fcntl(socket_fd, F_SETFL, O_NONBLOCK | O_ASYNC) < 0)
    {
        spdlog::error("MavlinkUDP::init, error setting nonblocking " + errno);
		close(socket_fd);
		exit(EXIT_FAILURE);
    }

    // initialize socket for remote (address is binded when the first paket arrives)
    //memset(&src_addr, 0, sizeof(src_addr));
    src_addr.sin_family = AF_INET;
    src_addr.sin_port   = htons(remote_port);

    // add the UDP Socket to the event loop file descriptor 
    fds[0].fd     = socket_fd;
    fds[0].events = POLLIN;
}

void
MavlinkUDP::sendBytes(const uint8_t *buf, unsigned packet_len)
{
    if (send_buf_len + packet_len < sizeof(send_buf) / sizeof(send_buf[0])) {
        memcpy(&send_buf[send_buf_len], buf, packet_len);
        send_buf_len += packet_len;
    }
}

int
MavlinkUDP::sendPacket()
{
    Mavlink::sendPacket();
	if (send_buf_len == 0 || !initialized) {
		return 0;
	}

    int ret = sendto(socket_fd, send_buf, send_buf_len, 0, 
        (struct sockaddr *)&src_addr, sizeof(src_addr));
    
    send_buf_len = 0;
	return ret;
}


void
MavlinkUDP::handleMessages() {

    // check for received data, timeout = 10ms 
    if (poll(&fds[0], 1, timeout) > 0) {

        mavlink_message_t msg;
        mavlink_status_t status;
        int nread = 0;

        if (fds[0].revents & POLLIN) {
            nread = recvfrom(socket_fd, buf, sizeof(buf), 0, (struct sockaddr *)&srcaddr, &addrlen);
        }

        // binds the remote ip address when the first paket arrives (remote port is fix)
        if (!initialized) {
            src_addr.sin_addr.s_addr = srcaddr.sin_addr.s_addr;
            initialized = true;
            spdlog::info("udp remote address initialized: " + std::string(inet_ntoa(srcaddr.sin_addr)));
        }
        
        for (ssize_t i = 0; i < nread; i++) {
            if (mavlink_parse_char(getChannel(), buf[i], &msg, &status)) {
                handle_message(&msg);
            }
        }
    }
    
    Mavlink::runServices();
}