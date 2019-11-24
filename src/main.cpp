#include <exception>
#include <iostream>

#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_sinks.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/rotating_file_sink.h"

#include "mav_mavlink_udp.h"
#include "dji_vehicle.h"
#include "controller.h"
#include "sensors.h"

#define MAVLKIN_UDP
//#define DEBUG_SENSORS

#ifdef DJI_OSDK
    #include "dji_vehicle.hpp"
    #include "dji_linux_helpers.hpp"
    #include "dji_mavlink.h"
    #include "dji_mobile_interface.h"
    #include "dji_mission_interface.h"
#endif

MspMockVehicle mspMockVehicle;
MspDjiVehicle mspDjiVehicle;

Mavlink* mavlink = nullptr;
std::thread mavThread;

LinuxSetup* linuxEnvironment = nullptr;
Vehicle* vehicle = nullptr;


void waitForExit() {
    // Poll stdin, exit on input
    struct pollfd fds[1] = {};
    fds[0].fd     = STDIN_FILENO;
    fds[0].events = POLLIN;
    int exit = 0;
    while (exit == 0) {
        #ifdef DEBUG_SENSORS
        spdlog::debug("temperature: " + MspSensors::getInstance()->getSensorValue(1).value);
        spdlog::debug("pressure: " + MspSensors::getInstance()->getSensorValue(2).value);
        #endif
    
        if (poll(&fds[0], 1, 5000) > 0) {
            if (fds[0].revents & POLLIN) {
                char buf = ' ';
                read(fds[0].fd, &buf, sizeof(buf));
                if (buf == '1') {
                    exit = true;
                }
            }
        }
    }
}


int main(int argc, char** argv) {
	
    try 
    {   
        std::vector<spdlog::sink_ptr> sinks;
        sinks.push_back(std::make_shared<spdlog::sinks::ansicolor_stdout_sink_st>());
        sinks.push_back(std::make_shared<spdlog::sinks::basic_file_sink_mt>("logs/msplog.txt"));
        auto combined_logger = std::make_shared<spdlog::logger>("logger", begin(sinks), end(sinks));

        // Log level
        combined_logger->flush_on(spdlog::level::warn);
        #ifdef MSP_DEBUG
            combined_logger->set_level(spdlog::level::debug);
        #else
            combined_logger->set_level(spdlog::level::info);
        #endif

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

        // Setup mobile communication
        #ifdef MAVLKIN_UDP
        mavlink = new MavlinkUDP(5001, 5000);
        mspDjiVehicle.initialize(vehicle, linuxEnvironment, mavlink);
        #else
        mavlink = new MavlinkDJI();
        mspDjiVehicle.initialize(vehicle, linuxEnvironment, mavlink);
        #endif
    }
    else{
        spdlog::warn("Vehicle not initialized, start simulation mode");
        mavlink = new MavlinkUDP(5001, 5000);  
        mspMockVehicle.initialize();
    }
    #else
        spdlog::warn("start simulation mode");
        mavlink = new MavlinkUDP(5001, 5000);
        mspMockVehicle.initialize();
    #endif

    // Initialize the controller
    MspController::getInstance()->initialize(mavlink);

    // Start mavlink connection
    mavThread = mavlink->start();

    // exit on input '1'
    waitForExit();
   
    if (mavlink){
        mavlink->stop();
        mavThread.join();
    }

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