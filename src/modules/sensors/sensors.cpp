#include "bmp280_defs.h"
#include "bmp280i2c.h"


void initializeSensors() {

    //#ifdef BMP280
        initI2C();
        initBmc280();
    //#endif

}
