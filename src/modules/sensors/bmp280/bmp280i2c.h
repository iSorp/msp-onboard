#include "bmp280_defs.h"

int
initBmc280(int dev);

int 
softReset();

int 
setPowerMode(int8_t mode);

int 
getPowerMode(const int8_t *mode);

int
setConfiguration(struct bmp280_config *conf);

int
getConfiguration(struct bmp280_config *conf);

double 
readTemperature();

double 
readPressure();