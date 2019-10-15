
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <time.h>
#if (defined __QNX__) | (defined __QNXNTO__)
/* QNX specific headers */
#include <unix.h>
#else
/* Linux / MacOS POSIX timer headers */
#include <sys/time.h>
#include <time.h>
#include <arpa/inet.h>
#include <stdbool.h> /* required for the definition of bool in C99 */
#endif

#include "mav_mavlink.h"


#define BUFFER_LENGTH 2041 // minimum buffer size that can be used with qnx (I don't know why)

uint64_t microsSinceEpoch();

Mavlink::Mavlink()
{

}


int Mavlink::test(int argc, char* argv[])
{
    int portno = atoi("5001");
    int sockfd, newsockfd;
    socklen_t clilen;
    char buffer[256];
    struct sockaddr_in serv_addr, cli_addr;
    int n;

    sockfd =  socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
    printf("ERROR opening socket");

    // Initialize server socket
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family      = AF_INET;  
    serv_addr.sin_addr.s_addr = INADDR_ANY;  
    serv_addr.sin_port        = htons(portno); // Convert port number into network byte order

    // bind the socket to the current IP address on port
    if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) 
        printf("ERROR on binding");

    // listen to the incoming connections max 1 connection.
    listen(sockfd, 1);

    // return new socket file descriptor for client connection
    clilen = sizeof(cli_addr);
    newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
    if (newsockfd < 0) 
        printf("ERROR on accept");

    printf("server: got connection from %s port %d\n",
    inet_ntoa(cli_addr.sin_addr), ntohs(cli_addr.sin_port));

    uint8_t buf[BUFFER_LENGTH];
    mavlink_message_t msg;
    uint16_t len;
    int bytes_sent;

    // receive
    ssize_t recsize;
    unsigned int temp = 0;

    while (true)
    {
		/* Send Heartbeat */
		mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_GENERIC, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
		len = mavlink_msg_to_send_buffer(buf, &msg);
        bytes_sent = send(newsockfd, buf, len, 0);

        sleep(1);


		memset(buf, 0, BUFFER_LENGTH);
		recsize = recv(newsockfd, (void *)buf, BUFFER_LENGTH, 0);
		if (recsize > 0)
      	{
			// Something received - print out all bytes and parse packet
			mavlink_message_t msg;
			mavlink_status_t status;
			
			printf("Bytes Received: %d\nDatagram: ", (int)recsize);
			for (int i = 0; i < recsize; ++i)
			{
				temp = buf[i];
				printf("%02x ", (unsigned char)temp);
				if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status))
				{
					// Packet received
					printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg.sysid, msg.compid, msg.len, msg.msgid);
				}
			}
			printf("\n");
		}
    }
}
