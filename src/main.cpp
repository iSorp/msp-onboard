
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

#include "mav_mavlink_udp.h"
#include "controller.h"

#ifdef DJI_OSDK
    #include "dji_vehicle.hpp"
    #include "dji_linux_helpers.hpp"
    #include "dji_mavlink.h"
    #include "dji_mobile_interface.h"
    #include "dji_mission_interface.h"
#endif

int main(int argc, char** argv) {
	
#ifdef DJI_OSDK
    // Setup OSDK.
    /*LinuxSetup linuxEnvironment(argc, argv);
    Vehicle* vehicle = linuxEnvironment.getVehicle();
    if (vehicle == nullptr)
    {
        std::cout << "Vehicle not initialized, exiting.\n";
        return -1;
    }*/

    // Setup mavlink for DJI
    MavlinkDJI* mavlinkDJI = new MavlinkDJI();
    std::thread threadMavlinkDJI = mavlinkDJI->start();

    // Setup mobile communication
    //setupMSDKComm(vehicle, &linuxEnvironment, mavlinkDJI);


    //setupDJIMission(vehicle, &linuxEnvironment);

    // dji mavlink test
    //setupMSDKComm(NULL, NULL, mavlinkDJI);
    //uint8_t heartBeat[17] = {0xfe, 0x09, 00, 0xff, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x08, 0x00, 0x00, 0x03, 0xf4, 0x5a };
    //mavlinkDJI->setBuffer(heartBeat, 17);
#endif
  
    // Initialize mavlink connection
    MavlinkUDP* mavlinkUDP = new MavlinkUDP(5001, 5000, "192.168.1.132");//"127.0.0.1");
    std::thread threadMavlinkUDP = mavlinkUDP->start();

    // Initialize the controller (DJI communication, sensors)
    MspController::getInstance()->initialize(mavlinkUDP);

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



/*
tests
    Mavlink* mavlink = mavlinkUDP;
    mavlink_message_t msg;
    mavlink_msg_command_ack_pack(mavlink->get_system_id(), mavlink->get_component_id(), &msg, MAV_CMD_MISSION_START, 0);
    mavlink->queueSendMessage(msg, MAVLINK_MSG_ID_COMMAND_INT_LEN);
    mavlink->queueSendMessage(msg, 1);
    mavlink_msg_command_ack_pack(111, mavlink->get_component_id(), &msg, MAV_CMD_MISSION_START, 0);
    mavlink->queueSendMessage(msg, MAVLINK_MSG_ID_COMMAND_INT_LEN);

*/