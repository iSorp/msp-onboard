#pragma once

#include <string>

typedef struct {
    int id;
    std::string value;
} SensorValue;


class Sensor {

    public:
        virtual int getStatus() { return 0; }
        virtual std::string getValue() { return "" ; }
};