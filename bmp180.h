#include "flight_controller.h"
#include "inttypes.h"
#include "flight_controller.h"
#include <stdint.h>  /* For using integer values with diffirent bit size */
#include <stdbool.h> /* For using bool value for assigning means zero and one state*/
#include "hardware/i2c.h" /* Header file from RPI Pico SDK C/C++ for usage I2C interface */
#include "pico/stdlib.h" /* A standard header file for usage RPI PI board */


#ifndef BAROMETER_H
#define BAROMETER_H
//Operating Modes
#define BMP180_ULTRALOWPOWER    0
#define BMP180_STANDARD         1
#define BMP180_HIGHRES          2
#define BMP180_ULTRAHIGHRES     3

//BMP185 Registers
uint8_t  BMP180_CAL_AC1    =      0xAA;  //Calibration data (16 bits)
uint8_t  BMP180_CAL_AC2      =    0xAC;  //Calibration data (16 bits)
uint8_t  BMP180_CAL_AC3     =     0xAE;  //Calibration data (16 bits)
uint8_t  BMP180_CAL_AC4     =     0xB0;  //Calibration data (16 bits)
uint8_t  BMP180_CAL_AC5     =     0xB2;  //Calibration data (16 bits)
uint8_t  BMP180_CAL_AC6     =     0xB4;  //Calibration data (16 bits)
uint8_t  BMP180_CAL_B1      =     0xB6;  //Calibration data (16 bits)
uint8_t  BMP180_CAL_B2       =    0xB8;  //Calibration data (16 bits)
uint8_t  BMP180_CAL_MB       =    0xBA;  //Calibration data (16 bits)
uint8_t  BMP180_CAL_MC       =    0xBC; //Calibration data (16 bits)
uint8_t  BMP180_CAL_MD       =    0xBE;  //Calibration data (16 bits)
uint8_t  BMP180_CONTROL      =    0xF4;
uint8_t  BMP180_TEMPDATA     =    0xF6;
uint8_t  BMP180_PRESSUREDATA  =   0xF6;
// Calibration data
int16_t AC1, AC2, AC3, B1, B2, MB, MC, MD;
uint16_t AC4, AC5, AC6;
//Commands
#define BMP180_READTEMPCMD      0x2E
#define BMP180_READPRESSURECMD  0x34
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
/**
 * @brief I2C request address value of calibaration for BMP180
 * A initial address of calibration request values (AC1, AC3, B1, MC, etc ... )
 * You can look a datasheet 
 * @see https://cdn-shop.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf
*/
uint8_t REG_CAL = 0xAA;
/**
 * @brief Value of region size of calibration EEPROM memory (The datasheet said that BMP180 sensor EEPROM is 176 bits (22 bytes))
8 * @see https://cdn-shop.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf
*/
uint8_t REG_CAL_SIZE = 2;
/**
 * @brief I2C request address value of receiving raw data in first bit (0x2E for temperature)
 * @see https://cdn-shop.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf
*/
uint8_t GET_DATA_RAW = 0xF6;


/**
 * @brief 7-bit Address of I2C for BMP180 sensor
 * It help easy to find this I2C address
*/
#define ADDRESS_BMP180 0x76

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
