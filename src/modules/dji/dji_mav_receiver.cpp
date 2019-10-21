
#include <stdio.h>
#include <pthread.h>
#include <limits.h>
#include <poll.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include "defines.h"
#include "mav_mavlink.h"
#include "dji_mavlink.h"


//-------------------------------------------------------------
// Class MavlinkUDP
//-------------------------------------------------------------
void
MavlinkDJI::readMessage()
{
    sleep(1);

    // handle timeouts and resets
    mission_manager.run();
}