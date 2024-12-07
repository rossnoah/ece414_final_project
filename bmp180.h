/**
 * Copyright (C) 2023 by Misha Zaslavskis   
 * License is BSD 3-Clause
*/

/**
 * @file bmp180.h
 * @author Misha Zaslavskis
 * @date 13 May 2023
 * @brief A header file of BMP180 sensor measuring temperature functionality 
 * @see See in bmp180.c code
 * */

#ifndef __BMP180_H__
#define __BMP180_H__
#include "flight_controller.h"
#include <stdint.h>  /* For using integer values with diffirent bit size */
#include <stdbool.h> /* For using bool value for assigning means zero and one state*/
#include "hardware/i2c.h" /* Header file from RPI Pico SDK C/C++ for usage I2C interface */
#include "pico/stdlib.h" /* A standard header file for usage RPI PI board */

/**
 * @brief 7-bit Address of I2C for BMP180 sensor
 * It help easy to find this I2C address
*/
#define ADDRESS_BMP180 0x77

/**
 * @brief A own struct of retreive data from BMP180 sensor
 * All values from this struct are float data type
 * @param air_temperature_in_celisus A float value of current air temperature in Celisus
 * @param atmospheric_pressure_in_Pa A float value of current atmosphere pressure in Pascal
*/
typedef struct data_from_bmp180
{
    float air_temperature_in_celisus;
    float atmospheric_pressure_in_Pa;
} data_from_bmp180;

/**
 * @brief Function of I2C interface initialization for BMP180 (Default I2C port in RPI PICO)
*/
void i2c_init_bmp180();

/**
 * @brief Function that retrieve temperature and pressure from BMP180 sensor 
*/
data_from_bmp180 get_bmp180_sensor_data(uint8_t oss_mode);

#endif // __BMP180_H_