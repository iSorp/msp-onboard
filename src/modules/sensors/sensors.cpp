#include <iostream>
#include <sys/ioctl.h>
#include <fcntl.h>

#if defined(__linux__)
#include <linux/i2c-dev.h>
#endif

#include "spdlog/spdlog.h"
#include "bmp280_defs.h"
#include "bmp280i2c.h"
#include "sensors.h"

 // devicehandle
static int device;   

// I2C bus 1
static const char *BUS = "/dev/i2c-1";

//-------------------------------------------------------------
// Singleton Class MspSensors 
//-------------------------------------------------------------
MspSensors *MspSensors::instance = 0;

MspSensors *
MspSensors::getInstance() {
    if (!instance)
        instance = new MspSensors;
    return instance;
}


int 
MspSensors::initialize() {
    int ret = initI2C();
    if (ret == 0){
        ret = initBmc280(device);
    }
    return ret;
}

int
MspSensors::initI2C() {

    // Open i2c Bus
    if((device = open(BUS, O_RDWR)) < 0)
    {
        spdlog::error("Failed to open i2c bus");
        return 1;
    }
    spdlog::info("Open i2c Bus :" + std::to_string(device));
}



SensorValue
MspSensors::getSensorValue(int sensor_id){
    SensorValue sensor;
    switch (sensor_id){
    case 1:
        sensor.id = sensor_id;
        sensor.value = std::to_string(readTemperature());
        break;
    case 2:
        sensor.id = sensor_id;
        sensor.value = std::to_string(readPressure());
        break;
    default:
        spdlog::warn("getSensorValue(" + std::to_string(sensor_id) + "), sensor not found");
        sensor.id = -1;
        sensor.value = "";
    }
    return sensor;
}