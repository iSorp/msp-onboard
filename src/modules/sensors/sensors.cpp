#include <iostream>
#include <sys/ioctl.h>
#include <fcntl.h>

#if defined(__linux__)
#include <linux/i2c-dev.h>
#endif

#include "bmp280_defs.h"
#include "bmp280i2c.h"
#include "sensors.h"

 // devicehandle
static int device;   

// I2C bus 1
const char *BUS = "/dev/i2c-1";

int
initI2C() {

    // Open i2c Bus
    if((device = open(BUS, O_RDWR)) < 0)
    {
        std::cout << "Failed to open i2c bus";
        return 1;
    }
    std::cout << "Open i2c Bus :" + device;
}

int 
initializeSensors() {
    int ret = initI2C();
    if (ret == 0){
        ret = initBmc280(device);
    }
    return ret;
}

sensor_t 
getSensorValue(int sensor_id){
    sensor_t sensor;
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
        sensor.id = -1;
        sensor.value = "";
    }
    return sensor;
}