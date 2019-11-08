
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
//#include "controller.h"

#ifdef DJI_OSDK
    #include <dji_vehicle.hpp>
    #include <dji_linux_helpers.hpp>
    #include "dji_mavlink.h"
    #include "dji_mobileComm.h"
#endif


int main(int argc, char** argv) {
	
#ifdef DJI_OSDK
    // Setup OSDK.
    LinuxSetup linuxEnvironment(argc, argv);
    Vehicle* vehicle = linuxEnvironment.getVehicle();
    if (vehicle == nullptr)
    {
        std::cout << "Vehicle not initialized, exiting.\n";
        return -1;
    }

    // Setup mavlink for DJI
    MavlinkDJI* mavlinkDJI = new MavlinkDJI();
    std::thread threadMavlinkDJI = mavlinkDJI->start();

    // Setup mobile communication
    setupMSDKComm(vehicle, &linuxEnvironment, mavlinkDJI);
#endif
  
    // Initialize mavlink connection
    MavlinkUDP* mavlinkUDP = new MavlinkUDP(5001, 5000, "127.0.0.1");
    std::thread threadMavlinkUDP = mavlinkUDP->start();


    // Initialize the controller (DJI communication, sensors)
    // FlightController::getInstance()->initialize();


    while (1) {
        sleep(1);
        setvbuf (stdout, NULL, _IONBF, 0);
    }
   
    mavlinkUDP->stop();
    threadMavlinkUDP.join();
    delete mavlinkUDP;

    #ifdef DJI_OSDK
        mavlinkDJI->stop();
        threadMavlinkDJI.join();
        delete mavlinkDJI;
    #endif
    return 0;
}