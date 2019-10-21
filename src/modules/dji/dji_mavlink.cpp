#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "helper.h"
#include "mav_mavlink.h"
#include "dji_mavlink.h"



//-------------------------------------------------------------
// Class MavlinkDJI
//-------------------------------------------------------------
MavlinkDJI::~MavlinkDJI() {

}

void
MavlinkDJI::init(){
 
}

void
MavlinkDJI::sendBytes(const uint8_t *buf, unsigned packet_len)
{

}

int
MavlinkDJI::sendPacket()
{
    Mavlink::sendPacket();

    int ret = 0;
	return ret;
}
