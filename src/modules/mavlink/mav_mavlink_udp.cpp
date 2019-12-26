/**
    @file mav_mavlink_udp.cpp
    @brief

    @Copyright (c) 2019 Samuel Ackermann, Simon Wälti
*/        


#include <string>
#include <arpa/inet.h>
#include <fcntl.h>

#include "mav_mavlink_udp.h"
#include "helper.h"

//-------------------------------------------------------------
// Class MavlinkUDP
//-------------------------------------------------------------
MavlinkUDP::~MavlinkUDP() {
    if (loc_socket_fd >= 0) {    
		close(loc_socket_fd);
	}
    if (rmt_socket_fd >= 0) {    
		close(rmt_socket_fd);
	}
}

void
MavlinkUDP::init(){

    // Initialize socket for receiving
    loc_addr.sin_family      = AF_INET;
	loc_addr.sin_addr.s_addr = INADDR_ANY;
	loc_addr.sin_port        = htons(local_port);

    if ((loc_socket_fd = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
        spdlog::error("MavlinkUDP::init, " + EXIT_FAILURE);
		exit(EXIT_FAILURE);
	}
	if (-1 == bind(loc_socket_fd,(struct sockaddr *)&loc_addr, sizeof(struct sockaddr))) {
        spdlog::error("MavlinkUDP::init, error bind failed");
		exit(EXIT_FAILURE);
    } 
    if (fcntl(loc_socket_fd, F_SETFL, O_NONBLOCK | O_ASYNC) < 0){
        spdlog::error("MavlinkUDP::init, error setting nonblocking " + errno);
		exit(EXIT_FAILURE);
    }

    // add the UDP Socket to the event loop file descriptor 
    fds[0].fd     = loc_socket_fd;
    fds[0].events = POLLIN;


    // Initialize socket for sending
    rmt_addr.sin_family = AF_INET;
    rmt_addr.sin_port   = htons(remote_port);
    rmt_addr.sin_addr.s_addr = inet_addr("255.255.255.255"); // set broatcast address

    if ((rmt_socket_fd = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
        spdlog::error("MavlinkUDP::init, " + EXIT_FAILURE);
		exit(EXIT_FAILURE);
	}
    int broadcastEnable=1;
    if (-1 == setsockopt(rmt_socket_fd, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable))) {
        spdlog::error("MavlinkUDP::init, error bind failed");
		exit(EXIT_FAILURE);
    }
	if (-1 == bind(rmt_socket_fd,(struct sockaddr *)&rmt_addr, sizeof(struct sockaddr))){
        spdlog::error("MavlinkUDP::init, error bind failed");
		exit(EXIT_FAILURE);
    } 
	if (fcntl(rmt_socket_fd, F_SETFL, O_NONBLOCK | O_ASYNC) < 0){
        spdlog::error("MavlinkUDP::init, error setting nonblocking " + errno);
		exit(EXIT_FAILURE);
    }

    spdlog::info("MavlinkUDP::init, udp socket initialized, start listening for remote connection");
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
	if (send_buf_len == 0) {
		return 0;
	}

    int ret = sendto(rmt_socket_fd, send_buf, send_buf_len, 0, 
        (struct sockaddr *)&rmt_addr, sizeof(rmt_addr));
    
    send_buf_len = 0;
	return ret;
}


void
MavlinkUDP::runService() {

    // check for received data, timeout = 10ms 
    if (poll(&fds[0], 1, timeout) > 0) {

        mavlink_message_t msg;
        mavlink_status_t status;
        int nread = 0;

        if (fds[0].revents & POLLIN) {
            nread = recvfrom(loc_socket_fd, buf, sizeof(buf), 0, (struct sockaddr *)&srcaddr, &addrlen);
        }

        // binds the remote ip address when the first paket arrives (remote port is fix)
        if (!initialized) {

            // disable broatcasting
            int broadcastEnable=0;
            if (-1 == setsockopt(rmt_socket_fd, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable))) {
                spdlog::error("MavlinkUDP::handleMessages, disable broatcast failed");
                exit(EXIT_FAILURE);
            }

            // assign ip address from remote
            rmt_addr.sin_addr.s_addr = srcaddr.sin_addr.s_addr;
            initialized = true;
            spdlog::info("udp remote address initialized: " + std::string(inet_ntoa(srcaddr.sin_addr)));
        }
        
        for (ssize_t i = 0; i < nread; i++) {
            if (mavlink_parse_char(getChannel(), buf[i], &msg, &status)) {
                Mavlink::handleMessages(&msg);
            }
        }
    }
    Mavlink::runService();
}