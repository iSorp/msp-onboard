#include <iostream>
#include <sys/ioctl.h>
#include <fcntl.h>

#if defined(__linux__)
#include <linux/i2c-dev.h>
#endif

#include "spdlog/spdlog.h"
#include "bmp280i2c.h"

 // i2c devicehandle
static int device;

// I2C bus 1
static const char *BUS = "/dev/i2c-1";

static Bmp280 bmp280;

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

bool 
MspSensors::initialize() {
    bool ret = false;
    if (initI2C()){  
        bmp280.initBmc280(device);

        sensorMap[1] = &bmp280.temperature;
        sensorMap[2] = &bmp280.pressure;

        ret = true;
    }
    return ret;
}

bool
MspSensors::initI2C() {

    // Open i2c Bus
    if((device = open(BUS, O_RDWR)) < 0)
    {
        spdlog::error("Failed to open i2c bus");
        return false;
    }
    spdlog::info("Open i2c Bus :" + std::to_string(device));
    return true;
}


SensorValue
MspSensors::getSensorValue(int sensor_id){
    SensorValue sensor;
    sensor.id = -1;
    sensor.value = "";

    if (sensorMap.count(sensor_id) > 0){
        sensor.id = sensor_id;
        sensor.value = sensorMap[sensor_id]->getValue();
    }
    else {
        spdlog::warn("getSensorValue(" + std::to_string(sensor_id) + "), sensor not found");
    }
    return sensor;
}