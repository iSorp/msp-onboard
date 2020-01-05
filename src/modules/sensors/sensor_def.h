/**
    @file sensor_def.h
    @brief

    @Copyright (c) 2019 Samuel Ackermann, Simon Wälti
*/   


#pragma once

#include <string>

typedef struct {
    int id;
    int command;
    std::string value;
} SensorValue;


class Sensor {

    public:
        virtual int getStatus() { return 0; }
        virtual std::string getValue() { return "" ; }
};