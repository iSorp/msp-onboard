#include <signal.h>

#include "mav_mavlink_udp.h"
#include "controller.h"

#define MAVLKIN_UDP
#define DJI_OSDK

#ifdef DJI_OSDK
    #include "dji_vehicle.hpp"
    #include "dji_linux_helpers.hpp"
    #include "dji_mavlink.h"
    #include "dji_mobile_interface.h"
    #include "dji_mission_interface.h"
#endif

const char* config_path = "/home/simon/UserConfig.txt";

int main(int argc, char** argv) {
	
    #ifdef DJI_OSDK
    const char* arg[3];
    arg[0] = argv[0];
    arg[1] = config_path;

    MavlinkDJI* mavlinkDJI;
    std::thread threadMavlinkDJI;

    // Setup OSDK.
    LinuxSetup linuxEnvironment(2, arg);
    Vehicle* vehicle = linuxEnvironment.getVehicle();
    if (vehicle != nullptr)
    {
        // Setup mavlink for DJI
        mavlinkDJI = new MavlinkDJI();
        std::thread threadMavlinkDJI = mavlinkDJI->start();

        // Setup mobile communication
        setupMSDKComm(vehicle, &linuxEnvironment, mavlinkDJI);
        setupDJIMission(vehicle, &linuxEnvironment);
    }
    else{
        std::cout << "Vehicle not initialized, UDP simulation mode.\n";

        // Setup mobile communication
        setupMSDKComm(NULL, &linuxEnvironment, NULL);
        setupDJIMission(NULL, &linuxEnvironment);
    }

    #endif

    #ifdef MAVLKIN_UDP    
    MavlinkUDP* mavlinkUDP = new MavlinkUDP(5001, 5000, "192.168.1.132");//"127.0.0.1");
    //MavlinkUDP* mavlinkUDP = new MavlinkUDP(5001, 5000, "127.0.0.1");
    std::thread threadMavlinkUDP = mavlinkUDP->start();
    #endif

    // dji mavlink test
    //setupMSDKComm(NULL, NULL, mavlinkDJI);
    //uint8_t heartBeat[17] = {0xfe, 0x09, 00, 0xff, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x08, 0x00, 0x00, 0x03, 0xf4, 0x5a };
    //mavlinkDJI->setBuffer(heartBeat, 17);

  
    // Initialize mavlink connection


    // Initialize the controller (DJI communication, sensors)
    MspController::getInstance()->initialize(mavlinkUDP);

    while (1) {
        sleep(1);
        setvbuf (stdout, NULL, _IONBF, 0);
    }
   
    #ifdef MAVLKIN_UDP
    if (mavlinkUDP){
        mavlinkUDP->stop();
        threadMavlinkUDP.join();
        delete mavlinkUDP;
    }
    #endif

    #ifdef DJI_OSDK
    if (mavlinkDJI){
        mavlinkDJI->stop();
        threadMavlinkDJI.join();
        delete mavlinkDJI;
    }
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