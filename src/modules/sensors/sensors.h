#pragma once

#include <string>
#include <map>

#include "sensor_def.h"

class MspSensors {

    struct State;

    public:
        static MspSensors *getInstance();

        bool initialize();
        SensorValue getSensorValue(int sensor_id);
        
    protected:
       

    private:
        static MspSensors *instance;
        //Bmp280 bmp280;

        std::map<int, Sensor*> sensorMap;

        bool initI2C(); 
};
