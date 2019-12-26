/**
    @file bmp280i2c.h
    @brief

    @Copyright (c) 2019 Samuel Ackermann, Simon Wälti
*/   


#pragma once

#include <string>
#include "sensors.h"
#include "bmp280_defs.h"

class Bmp280;

class Bmp280Temperature : public Sensor {
    public:
        Bmp280Temperature(Bmp280* board) : board(board) {}
        int getStatus() override;
        std::string getValue() override;

    private:
        Bmp280* board;
};

class Bmp280Pressure : public Sensor {
    public:
        Bmp280Pressure(Bmp280* board) : board(board) {}
        int getStatus() override;
        std::string getValue() override;

    private:
        Bmp280* board;
};

class Bmp280 {

    public:
        Bmp280() : 
            temperature(this),
            pressure(this)
        {}

        Bmp280Temperature temperature;
        Bmp280Pressure pressure;

        int initBmc280(int dev);
        int softReset();
        int setPowerMode(uint8_t mode);
        int getPowerMode(uint8_t mode);
        int setConfiguration(struct bmp280_config *conf);
        int getConfiguration(struct bmp280_config *conf);
};
