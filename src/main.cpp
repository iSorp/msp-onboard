#include <exception>
#include <iostream>

#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_sinks.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/rotating_file_sink.h"

#include "mav_mavlink_udp.h"
#include "controller.h"

#define MAVLKIN_UDP
//#define DJI_OSDK

#ifdef DJI_OSDK
    #include "dji_vehicle.hpp"
    #include "dji_linux_helpers.hpp"
    #include "dji_mavlink.h"
    #include "dji_mobile_interface.h"
    #include "dji_mission_interface.h"
#endif

int main(int argc, char** argv) {
	
    try 
    {   
        std::vector<spdlog::sink_ptr> sinks;
        sinks.push_back(std::make_shared<spdlog::sinks::ansicolor_stdout_sink_st>());
        sinks.push_back(std::make_shared<spdlog::sinks::basic_file_sink_mt>("logs/msplog.txt"));
        auto combined_logger = std::make_shared<spdlog::logger>("logger", begin(sinks), end(sinks));

        // Log level
        combined_logger->flush_on(spdlog::level::warn);
        combined_logger->set_level(spdlog::level::info);

        //register it if you need to access it globally
        spdlog::register_logger(combined_logger);
        spdlog::set_default_logger(combined_logger);
    }
    catch (const spdlog::spdlog_ex &ex)
    {
        std::cout << "Log init failed: " << ex.what() << std::endl;
    }
    spdlog::info("start app msp-onboard");

    #ifdef DJI_OSDK
    spdlog::info("DJI OSDK available");
    const char* arg[3];
    arg[0] = argv[0];
    arg[1] = DJI_USER_CONFIG;

    MavlinkUDP* mavlinkUDP = nullptr;
    MavlinkDJI* mavlinkDJI = nullptr;
    LinuxSetup* linuxEnvironment = nullptr;
    Vehicle* vehicle = nullptr;
    std::thread threadMavlinkDJI;

    // Setup OSDK.
    try {
        linuxEnvironment = new LinuxSetup(2, arg);
        vehicle = linuxEnvironment->getVehicle();
    }
    catch (std::exception& e) {
        spdlog::error(e.what());
    }

    if (vehicle)
    {
        spdlog::info("DJI vehicle found");

        // Setup mavlink for DJI
        mavlinkDJI = new MavlinkDJI();
        std::thread threadMavlinkDJI = mavlinkDJI->start();

        // Setup mobile communication
        setupMSDKComm(vehicle, linuxEnvironment, mavlinkDJI);
        setupDJIMission(vehicle, linuxEnvironment);
    }
    else{
        spdlog::warn("Vehicle not initialized, try to start UDP simulation mode");

        // Setup mobile communication
        setupMSDKComm(NULL, linuxEnvironment, NULL);
        setupDJIMission(NULL, linuxEnvironment);
    }
    #endif

    // Initialize mavlink connection
    #ifdef MAVLKIN_UDP   
    spdlog::info("mavlink udp available");
    mavlinkUDP = new MavlinkUDP(5001, 5000);
    std::thread threadMavlinkUDP = mavlinkUDP->start();
    #endif

    // Initialize the controller (DJI communication, sensors)
    MspController::getInstance()->initialize(mavlinkUDP);

    int exit = 0;
    while (exit == 0) {
        std::cin >> exit;
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
    delete linuxEnvironment;
    delete vehicle;
    #endif

    spdlog::info("exit app msp-onboard");
    return 0;
}



// dji mavlink test
//setupMSDKComm(NULL, NULL, mavlinkDJI);
//uint8_t heartBeat[17] = {0xfe, 0x09, 00, 0xff, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x08, 0x00, 0x00, 0x03, 0xf4, 0x5a };
//mavlinkDJI->setBuffer(heartBeat, 17);



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