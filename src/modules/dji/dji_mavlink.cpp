#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "helper.h"
#include "mav_mavlink.h"
#include "dji_mavlink.h"



//-------------------------------------------------------------
// Class MavlinkDJI
//-------------------------------------------------------------

void
MavlinkDJI::sendBytes(const uint8_t* buf, unsigned packet_len)
{
    size_t ret = -1;
    if (send_buf_len + packet_len < sizeof(send_buf) / sizeof(send_buf[0])) {
        memcpy(&send_buf[send_buf_len], buf, packet_len);
        send_buf_len += packet_len;
        ret = packet_len;
    }
}

int
MavlinkDJI::sendPacket()
{
    Mavlink::sendPacket();
	if (send_buf_len == 0) {
		return 0;
	}

    sendDataCallback(send_buf, send_buf_len);
    int ret = send_buf_len;

    send_buf_len = 0;
	return ret;
}
