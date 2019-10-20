
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <pthread.h>
#include <limits.h>

#include <sys/time.h>
#include <time.h>
#include <arpa/inet.h>






// System Includes
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>

#include "mav_mavlink.h"
#include "mav_mission.h"


/*#include <cmath>
#include <stdio.h>

// DJI OSDK includes
#include "dji_status.hpp"
#include <dji_vehicle.hpp>
*/


int main() {
	printf("%s", "hello world");
    
    start_mavlink_task();
    //test(0,0);   
    //Mavlink* mavlink = new Mavlink();
    //MavlinkMissionManager* mission = new MavlinkMissionManager(mavlink);
    //mavlink -> test(0,0);
    while (1) {
        sleep(1);
        setvbuf (stdout, NULL, _IONBF, 0);
    }
   
    return 1;
}