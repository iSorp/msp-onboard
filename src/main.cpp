
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

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <thread>

#include "mav_mavlink.h"

#ifdef DJI_OSDK
    #include "dji_mavlink.h"
#endif

/*#include <cmath>
#include <stdio.h>

// DJI OSDK includes
#include "dji_status.hpp"
#include <dji_vehicle.hpp>
*/


int main() {
	printf("%s", "hello world");
    
    MavlinkUDP* mavlinkUDP = new MavlinkUDP(5001, 5000, "127.0.0.1");
    std::thread t1 = mavlinkUDP->start();
    
    #ifdef DJI_OSDK
        MavlinkDJI* mavlinkDJI = new MavlinkDJI();
        std::thread t2 = mavlinkDJI->start();
    #endif

    //start_mavlink_task();
    //test(0,0);   
    //Mavlink* mavlink = new Mavlink();
    //MavlinkMissionManager* mission = new MavlinkMissionManager(mavlink);
    //mavlink -> test(0,0);
    while (1) {
        sleep(1);
        setvbuf (stdout, NULL, _IONBF, 0);
    }
   
    mavlinkUDP->stop();
    t1.join();
    delete mavlinkUDP;

    #ifdef DJI_OSDK
        mavlinkDJI->stop();
        t2.join();
        delete mavlinkDJI;
    #endif
    return 0;
}