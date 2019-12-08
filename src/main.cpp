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


#define MAVLKIN_UDP
//#define MAVLKIN_DJI
//#define DEBUG_SENSORS

#ifdef DJI_OSDK
    #include "dji_mspvehicle.h"
    #include "dji_vehicle.hpp"
    #include "dji_linux_helpers.hpp"
    #include "dji_mavlink.h"

    static Vehicle* vehicle = nullptr;
#endif

static MspVehicle* mspVehicle = nullptr;
static Mavlink* mavlink = nullptr;
static std::thread mavThread;
static void waitForExit();

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
        LinuxSetup environment(2, arg);
        vehicle = environment.getVehicle();
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
        #endif

        #ifdef MAVLKIN_DJI
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
    mspVehicle->initialize();
    MspController::getInstance()->initialize(mavlink);

    // Start mavlink connection
    mavThread = mavlink->start();

    // exit on input '1'
    waitForExit();
   
    if (mavlink){
        mavlink->stop();
        mavThread.join();
    }

    delete mavlink;
    delete mspVehicle;

    spdlog::info("exit msp-onboard");
    return 0;
}


static void 
waitForExit() {
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