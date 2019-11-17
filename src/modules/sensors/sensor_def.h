#pragma once

#include <string>

class Sensor {

    public:
        virtual int getStatus(){}
        virtual std::string getValue() {}
};