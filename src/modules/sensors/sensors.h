#include <string>

struct sensor_t {
    int id;
    std::string value;
};

int initializeSensors();

sensor_t getSensorValue(int sensor_id);