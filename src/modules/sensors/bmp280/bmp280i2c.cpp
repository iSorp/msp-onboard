
#include <iostream>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <sys/ioctl.h>

#if defined(__linux__)
    #include <linux/i2c-dev.h>
#endif

#include "spdlog/spdlog.h"
#include "bmp280.h"
#include "bmp280i2c.h"


/* BMP280-Pinouts
===============================
Vin:	Input voltage 3-5VDC 
GND:	Ground
SCK:	i2c clock line
SDI:	i2c Data line

#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22
*/

// Error codes
#define BMP280_I2C_BUS_INIT     1
#define BMP280_WRITE_REG        2
#define BMP280_WRITE_DATA       3
#define BMP280_READ_REG         4
#define BMP280_READ_DATA        5

#define REG_ADDR_SIZE 2

// i2c devicehandle
static int device;  

// Meassuring duration time
static uint8_t meas_dur;

static int altitude = 500;
static struct bmp280_dev bmp280;

/****************** Static Function Definitions *******************************/

static void 
BMP280_delay_msek(uint32_t msek)
{
    sleep(msek/1000);
}

static int8_t 
BMP280_I2C_bus_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    int8_t error = 0;
    if (write(device, &reg_addr, REG_ADDR_SIZE) != REG_ADDR_SIZE) {
        spdlog::error("Unable to write the reg_addr to i2c bus for writing");
        error = BMP280_WRITE_REG;
    }

    if (write(device, data, len) != len) {
        spdlog::error("Unable to write data to i2c bus");
        error = BMP280_WRITE_DATA;
    }
   
	return error;
}

static int8_t 
BMP280_I2C_bus_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	int8_t error = 0;
    if (write(device, &reg_addr, REG_ADDR_SIZE) != REG_ADDR_SIZE) {
        spdlog::error("Unable to write the reg_addr to i2c bus for reading");
        error = BMP280_READ_REG;
    }
    if(read(device, data, len) != len)
    {
        spdlog::error("Unable to read data from i2c bus");
        error = BMP280_READ_DATA;
    }
	return error;
}

static double 
readTemperature() {
    double temp;
    struct bmp280_uncomp_data ucomp_data;
	bmp280.delay_ms(meas_dur); 
	bmp280_get_uncomp_data(&ucomp_data, &bmp280);
    bmp280_get_comp_temp_double(&temp, ucomp_data.uncomp_temp, &bmp280);
    return temp;
}

static double 
readPressure() {
    double press;
    struct bmp280_uncomp_data ucomp_data;
	bmp280.delay_ms(meas_dur); /* Measurement time */
	bmp280_get_uncomp_data(&ucomp_data, &bmp280);
    bmp280_get_comp_pres_double(&press, ucomp_data.uncomp_press, &bmp280);
    double pressure_nn = press/pow(1 - altitude/44330.0, 5.255);
	return pressure_nn;
}

//-------------------------------------------------------------
// Class Bmp280 
//-------------------------------------------------------------
int
Bmp280::setConfiguration(struct bmp280_config *conf) {
    // Overwrite settings
	int8_t rslt = (int)bmp280_set_config(conf, &bmp280);
	if (rslt != BMP280_OK){
        spdlog::error("bmp280.setConfiguration, Unable to set config: " + std::to_string(rslt));
        return rslt;
	}
    return 0;
}
int
Bmp280::getConfiguration(struct bmp280_config *conf) {
    // Read current settings
	int8_t rslt = (int)bmp280_get_config(conf, &bmp280);
	if (rslt != BMP280_OK) {
        spdlog::error("bmp280.getConfiguration, Unable to get config: " + std::to_string(rslt));
        return rslt;
	}
    return 0;
}

int 
Bmp280::getPowerMode(uint8_t mode) {
    // Get normal power mode
	int8_t rslt = bmp280_get_power_mode(&mode, &bmp280);
    if (rslt != BMP280_OK){
        spdlog::error("bmp280.getPowerMode, Unable to get power mode: " + std::to_string(rslt));
        return rslt;
	}
    return 0;
}

int 
Bmp280::setPowerMode(uint8_t mode) {
    // Set normal power mode
	int8_t rslt = bmp280_set_power_mode(mode, &bmp280);
    if (rslt != BMP280_OK){
        spdlog::error("bmp280.setPowerMode, Unable to set power mode: " + std::to_string(rslt));
        return rslt;
	}
    return 0;
}

int 
Bmp280::softReset() {
    // Reset
	int8_t rslt = bmp280_soft_reset(&bmp280);
    if (rslt != BMP280_OK){
        spdlog::error("bmp280.softReset, Unable to force reset");
        return rslt;
	}
    return 0;
}

/*
*  Initialize the Bmp280 sensor.
*/
int
Bmp280::initBmc280(int dev) {
    device = dev;

    #if defined(__linux__)
    // get I2C sensor
    int ret = ioctl(device, I2C_SLAVE, BMP280_I2C_ADDR_SEC);
    if (ret = -1){
        spdlog::error("failed to get I2C sensor");
        return 1;
    }
    spdlog::info("get I2C sensor: " + std::to_string(ret));
    #endif
    
    bmp280.write    = BMP280_I2C_bus_write;
    bmp280.read 	= BMP280_I2C_bus_read;
    bmp280.intf 	= BMP280_I2C_INTF;
    bmp280.chip_id  = BMP280_CHIP_ID1;
    bmp280.dev_id   = BMP280_I2C_ADDR_SEC;
    bmp280.delay_ms = BMP280_delay_msek;

    // Sensor initialization
    printf("initialize bmp280\n");
	int8_t rslt = bmp280_init(&bmp280);
	if (rslt != BMP280_OK) {  
        spdlog::error("Unable to initialize bmp280: " + std::to_string(rslt));
        return rslt;
	}

	// bmp280 configuration
    printf("configure bmp280\n");
    struct bmp280_config conf;
    getConfiguration(&conf);


	conf.filter = BMP280_FILTER_COEFF_2;
	conf.os_pres = BMP280_OS_16X;
	conf.os_temp = BMP280_OS_4X; 
	conf.odr = BMP280_ODR_1000_MS;
    rslt = setConfiguration(&conf);

    // Set normal power mode
    printf("set mode bmp280\n");
	rslt = bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp280);
    if (rslt != BMP280_OK){
        spdlog::error("Unable to set power mode: " + std::to_string(rslt));
        return rslt;
	}

    // get computed meassure time
    meas_dur = bmp280_compute_meas_time(&bmp280);
    spdlog::info("set power mode %hhx: " + meas_dur);
    return 0;
}

//-------------------------------------------------------------
// Class Bmp280Temperature 
//-------------------------------------------------------------
std::string 
Bmp280Temperature::getValue() {
    return std::to_string(readTemperature());
}

int
Bmp280Temperature::getStatus() {
    return 0;
}


//-------------------------------------------------------------
// Class Bmp280Pressure 
//-------------------------------------------------------------
std::string 
Bmp280Pressure::getValue() {
    return std::to_string(readPressure());
}

int
Bmp280Pressure::getStatus() {
    return 0;
}