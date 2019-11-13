#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <math.h>
#include <time.h>

#ifndef STANDALONE
    #include <linux/i2c-dev.h>
#endif

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

 // devicehandle
static int device;   

// I2C bus 1
static char *BUS = "/dev/i2c-1";

// Meassuring duration time
static uint8_t meas_dur;

int altitude = 500;

/********************** Static function declarations ************************/

static int8_t 
BMP280_I2C_bus_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);

static int8_t 
BMP280_I2C_bus_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);

static void 
BMP280_delay_msek(uint32_t msek);

static struct bmp280_dev bmp280 = {
    .write	= BMP280_I2C_bus_write,
    .read 	= BMP280_I2C_bus_read,
    .intf 	= BMP280_I2C_INTF,
    .chip_id  = BMP280_CHIP_ID1,
    .dev_id   = BMP280_I2C_ADDR_SEC,
    .delay_ms = BMP280_delay_msek
};

/****************** User Function Definitions *******************************/

double 
readTemperature() {
    struct bmp280_uncomp_data ucomp_data;
	bmp280.delay_ms(meas_dur); /* Measurement time */
	bmp280_get_uncomp_data(&ucomp_data, &bmp280);
	return bmp280_comp_temp_double(ucomp_data.uncomp_temp, &bmp280);
}

double 
readPressure() {
    struct bmp280_uncomp_data ucomp_data;
	bmp280.delay_ms(meas_dur); /* Measurement time */
	bmp280_get_uncomp_data(&ucomp_data, &bmp280);

    double pressure = bmp280_comp_pres_double(ucomp_data.uncomp_press, &bmp280);
    double pressure_nn = pressure/pow(1 - altitude/44330.0, 5.255);
	return pressure_nn;
}

int 
softReset() {
    // Reset
	int8_t rslt = bmp280_soft_reset(&bmp280);
    if (rslt != BMP280_OK){
        printf("Unable to force reset %i\n", rslt);
        return rslt;
	}
    return 0;
}

int 
setPowerMode(int8_t mode) {
    // Set normal power mode
	int8_t rslt = bmp280_set_power_mode(mode, &bmp280);
    if (rslt != BMP280_OK){
        printf("Unable to set power mode %i\n", rslt);
        return rslt;
	}
    return 0;
}

int 
getPowerMode(int8_t *mode) {
    // Get normal power mode
	int8_t rslt = bmp280_get_power_mode(mode, &bmp280);
    if (rslt != BMP280_OK){
        printf("Unable to get power mode %i\n", rslt);
        return rslt;
	}
    return 0;
}

int
setConfiguration(struct bmp280_config *conf) {
    // Overwrite settings
	int8_t rslt = (int)bmp280_set_config(conf, &bmp280);
	if (rslt != BMP280_OK){
        printf("Unable to set config %i\n", rslt);
        return rslt;
	}
    return 0;
}
int
getConfiguration(struct bmp280_config *conf) {
    // Read current settings
	int8_t rslt = (int)bmp280_get_config(conf, &bmp280);
	if (rslt != BMP280_OK) {
        printf("Unable to get config %i\n", rslt);
        return rslt;
	}
    return 0;
}

/*
*  Initialize the Bmp280 sensor.
*/
int
initBmc280() {
   
    // Sensor initialization
    printf("initialize bmp280\n");
	int8_t rslt = bmp280_init(&bmp280);
	if (rslt != BMP280_OK) {  
        printf("Unable to initialize bmp280 %i\n", rslt);
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
        printf("Unable to set power mode %i\n", rslt);
        return rslt;
	}

    // get computed meassure time
    meas_dur = bmp280_compute_meas_time(&bmp280);
    printf("set power mode %hhx\n", meas_dur);

    return 0;
}

int
initI2C() {
#ifndef STANDALONE
    // Open i2c Bus
    if((device = open(BUS, O_RDWR)) < 0)
    {
        printf("Failed to open i2c bus. \n");
        return BMP280_I2C_BUS_INIT;
    }
    printf("Open i2c Bus %i\n", device);

    // get I2C sensor
    int ret = ioctl(device, I2C_SLAVE, BMP280_I2C_ADDR_SEC);
    printf("get I2C sensor %i\n", ret);
#endif
    return 0;
}

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
        printf("Unable to write the reg_addr to i2c bus for writing\n");
        error = BMP280_WRITE_REG;
    }

    if (write(device, data, len) != len) {
        printf("Unable to write data to i2c bus\n");
        error = BMP280_WRITE_DATA;
    }
   
	return error;
}

static int8_t 
BMP280_I2C_bus_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	int8_t error = 0;
    if (write(device, &reg_addr, REG_ADDR_SIZE) != REG_ADDR_SIZE) {
        printf("Unable to write the reg_addr to i2c bus for reading\n");
        error = BMP280_READ_REG;
    }
    if(read(device, data, len) != len)
    {
        printf("Unable to read data from i2c bus\n");
        error = BMP280_READ_DATA;
    }
	return error;
}