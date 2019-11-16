#include <string>

typedef struct {
    int id;
    std::string value;
} SensorValue;


class MspSensors {

    struct State;

    public:
        static MspSensors *getInstance();

        int initialize();
        SensorValue getSensorValue(int sensor_id);
        
    protected:
       

    private:
        static MspSensors *instance;

        int initI2C(); 

};
