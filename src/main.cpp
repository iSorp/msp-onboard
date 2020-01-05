/**
    @file main.cpp
    @brief 
    This application is multithreaded and implements following main threads:
    1. Main thread: Used for initialization
    2. (either a or b)
        a. Mavlink UDP: Event loop for polling a udp socket. Handles incoming and outgoing mavlink packages. 
        b. Mavlink Airlink: Event loop for polling a byte buffer which is provided from DJI. 
            Handles incomming and outgoing mavlink packages.
    3. DJI OSDK threads
        a. Serial Read 
        b. USB Read
        c. Callback

    @Copyright (c) 2019 Samuel Ackermann, Simon Wälti
*/

#include <exception>
#include <iostream>

#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_sinks.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/rotating_file_sink.h"

#include "mav_mavlink_udp.h"
#include "controller.h"
#include "sensors.h"


// #define MAVLKIN_UDP
//#define DEBUG_SENSORS

#ifdef DJI_OSDK
    #include "dji_mspvehicle.h"
    #include "dji_vehicle.hpp"
    #include "dji_linux_helpers.hpp"
    #include "dji_mavlink.h"

    static Vehicle* vehicle = nullptr;
    static LinuxSetup* environment = nullptr;
#endif

static MspVehicle* mspVehicle = nullptr;
static Mavlink* mavlink = nullptr;
static std::thread mavThread;
static bool pollForExit();


/**
    @brief
    Initializes the main components of the application as follows:
    1. Setup logging
    2. Initialize/Start DJI Environment(LinuxSetup) and Vehicle
        - if no DJI vehicle is available, a Mock vehicle ist started
    3. Initialize MspController
    4. Initialize/Start Mavlink
         - if no DJI vehicle is available or udp mode is forced, UDP Mavlink channel is used
            otherwise DJI Airlink-Mavlink channel is used

    The application can be exited on stdin '1'.
    @param
    @return 
*/
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
    const char* arg[2] = {argv[0], DJI_USER_CONFIG};

    // Setup OSDK.
    try {
        environment = new LinuxSetup(2, arg);
        vehicle = environment->getVehicle();
    }
    catch (std::exception& e) {
        spdlog::error(e.what());
    }

    if (vehicle)
    {
        spdlog::info("DJI vehicle found");

        #ifdef MAVLKIN_UDP
        mavlink = new MavlinkUDP(5001, 5000);
        #else
        mavlink = new MavlinkDJI();
        #endif

        mspVehicle = new MspDjiVehicle(vehicle, mavlink);
    }
    else{
        spdlog::warn("Vehicle not initialized, start simulation mode");
        mavlink     = new MavlinkUDP(5001, 5000);  
        mspVehicle  = new MspMockVehicle();
    }
    #else
        spdlog::warn("start simulation mode");
        mavlink     = new MavlinkUDP(5001, 5000);
        mspVehicle  = new MspMockVehicle();
    #endif

    // Initialize the controller
    MspController::getInstance()->initialize(mavlink);
    mspVehicle->initialize();

    // Start mavlink eventloop
    mavThread = mavlink->start();

    // exit on input '1'
    while (!pollForExit()) {

        #ifdef DEBUG_SENSORS
        spdlog::debug("temperature: " + MspSensors::getInstance()->getSensorValue(1).value);
        spdlog::debug("pressure: " + MspSensors::getInstance()->getSensorValue(2).value);
        #endif
    }

    if (mavlink){
        mavlink->stop();
        mavThread.join();
    }

    delete mavlink;
    delete mspVehicle;
    delete environment;

    spdlog::info("exit msp-onboard");
    return 0;
}

/**
    @brief
    Polls the stdin for user input, the application exits on '1'.
    @param
    @return exit true/false
*/
static bool 
pollForExit() {
    // Poll stdin, exit on input
    struct pollfd fds[1] = {};
    fds[0].fd     = STDIN_FILENO;
    fds[0].events = POLLIN;
    bool exit = false;
    
    if (poll(&fds[0], 1, 5000) > 0) {
        if (fds[0].revents & POLLIN) {
            char buf = ' ';
            read(fds[0].fd, &buf, sizeof(buf));
            if (buf == '1') {
                exit = true;
            }
        }
    }
    return exit;
}