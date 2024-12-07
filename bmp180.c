/**
 * Copyright (C) 2023 by Misha Zaslavskis   
 * License is BSD 3-Clause
*/

/**
 * @file bmp180.c
 * @author Misha Zaslavskis
 * @date 13 May 2023
 * @brief 
 * A bmp180 source of code that support BMP180 temperature and barometer sensor in I2C inferface for Raspberry Pi Pico / RP2040 MCU
 * To able use function of BMP180 sensor to measure temperature and atmosphere pressure
 * Include file: bmp180.h
 * We worked with I2C regsiter in this code to enable to measure temperature and atmosphere pressure
 * Don't edit this code unless knowedge about sensor bmp180 to avoid errorness and fauly your software application (e.g. improperty receiving data from sensor). 
 * Look a datasheet of BMP180 for understanding how works register
 * @see https://cdn-shop.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf
 * Training and use examples of I2C BMP180 sensor code due misunderstanding in datasheet (Writting correct I2C transmitting and receiving). This example for stm32 MCU and isn't for Raspberry Pico W
 * @see https://controllerstech.com/interface-bmp180-with-stm32/
 * Note: Worked in i2c0 I2C port for this code. If you want to move another I2C port (use i2c1 port), you need to edit this code, changing i2c0 to i2c1 in Raspberry Pi Pico SDK API all I2C function. 
 * For example: from i2c_write_blocking(i2c0, ADDRESS_BMP180, &REG_CAL, 1, false); into i2c_write_blocking(i2c1, ADDRESS_BMP180, &REG_CAL, 1, false);
 * @see https://www.raspberrypi.com/documentation/pico-sdk/hardware.html#ga5149e9af84926fa19db6677691ab2900
 */

#include "bmp180.h" /* Including bmp180.h for use own data type when retrieving from bmp180 sensor */
/**
 * @brief I2C request address value of calibaration for BMP180
 * A initial address of calibration request values (AC1, AC3, B1, MC, etc ... )
 * You can look a datasheet 
 * @see https://cdn-shop.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf
*/
uint8_t REG_CAL = 0xAA;
/**
 * @brief Value of region size of calibration EEPROM memory (The datasheet said that BMP180 sensor EEPROM is 176 bits (22 bytes))
 * @see https://cdn-shop.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf
*/
uint8_t REG_CAL_SIZE = 22;
/**
 * @brief I2C request address value of receiving raw data in first bit (0x2E for temperature)
 * @see https://cdn-shop.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf
*/
uint8_t GET_DATA_RAW = 0xF6;

/**
 * @brief I2C inferface initialization function for BMP180 sensor (default I2C port - i2c0)
 * If you already initialized default port I2C interface for another application, so you don't need to add function i2c_init_bmp180
 * I2C default port is i2c0
*/
void i2c_init_bmp180()
{
    i2c_init(i2c_default, 1000 * 100); /* Initialize I2C interface with default port */
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C); /* Set GPIO function as I2C with default RPI Pico Pin for SDA I2C pin */
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C); /* Set GPIO function as I2C with default RPI Pico Pin for SCL I2C pin */
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN); /* Set pull-up internal resistor of MCU for SDA pin */
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN); /* Set pull-up internal resistor of MCU for SCL pin */
}
/**
 * @brief Receive data from BMP180 sensor with air temperature and atmosphere pressure
 * @param oss_mode A oversampling mode of BMP180 that it is help how much accuracy of measure pressure
 * @return @c data_from_bmp180 A own data struct of data_from_bmp180. Current air temperature and atmosphere pressure 
*/
data_from_bmp180 get_bmp180_sensor_data(uint8_t oss_mode)
{
    uint8_t REG_dt[22] = {0}; /* Array of EEPROM data of BMP180 for calibration values */

    i2c_write_blocking(i2c0, ADDRESS_BMP180, &REG_CAL, 1, false); /* Send request for data of calibration sensor */
    i2c_read_blocking(i2c0, ADDRESS_BMP180, REG_dt, REG_CAL_SIZE, false); /* Receiving calibration of this sensor */

    /* Assign 16 bit values from recieved array of data calibration */
    int16_t AC1 = ((REG_dt[0] << 8) | REG_dt[1]);
    int16_t AC2 = ((REG_dt[2] << 8) | REG_dt[3]);
    int16_t AC3 = ((REG_dt[4] << 8) | REG_dt[5]);
    uint16_t AC4 = ((REG_dt[6] << 8) | REG_dt[7]);
    uint16_t AC5 = ((REG_dt[8] << 8) | REG_dt[9]);
    uint16_t AC6 = ((REG_dt[10] << 8) | REG_dt[11]);

    int16_t B1 = ((REG_dt[12] << 8) | REG_dt[13]);
    int16_t B2 = ((REG_dt[14] << 8) | REG_dt[15]);
    int16_t MB = ((REG_dt[16] << 8) | REG_dt[17]);
    int16_t MC = ((REG_dt[18] << 8) | REG_dt[19]);
    int16_t MD = ((REG_dt[20] << 8) | REG_dt[21]);

    uint8_t measure_setting_temperature[] = {0xF4, 0x2E}; /* Set request type of data to able measure uncompersanted temperature */
    uint8_t buf_raw_temperature[2] = {0}; /* Array of uncompersanted temperature data for construction 16 bit value */

    i2c_write_blocking(i2c0, ADDRESS_BMP180, measure_setting_temperature, 2, false); /* Send request to type of data for receiving uncompersanted temperature in BMP180 sensor */
    sleep_us(4500); /* Must delay minimun 4.5 ms, according datasheet of BMP180 (See details in page 15 (algorithm) and 21 (table) ) */

    i2c_write_blocking(i2c0, ADDRESS_BMP180, &GET_DATA_RAW, 1, false); /* Send requset to receiveing uncompersanted temperature in BMP180 sensor*/
    i2c_read_blocking(i2c0, ADDRESS_BMP180, buf_raw_temperature, 2, 1); /* Receive data of uncompersanted temperature in BMP180 sensor*/

    long UT = ((buf_raw_temperature[0] << 8) + buf_raw_temperature[1]); /* Construct long value of uncompersanted temperature, according datasheet in page 15 */

    /* Calculation temperature, according datasheet in page 15 */
    long X1 = (UT - AC6) * AC5 / 32768;
    long X2 = MC * 2048 / (X1 + MD);
    long B5 = X1 + X2;
    long T = ((B5 + 8) / 16); /* Receive compersanted temperature in long value with resolution 0.1 oC (e.g. Value of T 230 is means 23.0 oC) */

    /* Reading raw pressure */
    uint8_t measuring_setting_pressure[] =  {0xF4, 0x34 + (oss_mode << 6)}; /* Request value to receiving data of uncompersanted atmosphere pressure in BMP180 barometer */
    i2c_write_blocking(i2c0, ADDRESS_BMP180, measuring_setting_pressure, 2, false); /* Send a request to retrive data of uncompersanted atmosphere pressure */

    /* Delay correctly dependant oss mode, according datasheet (see in page 21) */
    switch (oss_mode)
    {
        case 0: /* For 0th OSS mode of BMP180 */
           sleep_us(4500); /* Delay for 4.5 ms */
        break;

        case 1: /* For 1th OSS mode of BMP180 */
           sleep_us(7500); /* Delay for 7.5 ms */
        break;

        case 2: /* For 2nd OSS mode of BMP180 */
           sleep_us(13500); /* Delay for 13.5 ms */
        break;

        case 3: /* For 3rd OSS mode of BMP180 */
           sleep_us(25500); /* Delay for 25.5 ms */
        break;
    }

    uint8_t buf_raw_pressure[3] = {0}; /* Buffer for receiving uncompersanted pressure */

    i2c_write_blocking(i2c0, ADDRESS_BMP180, &GET_DATA_RAW, 1, false); /* Send request to receiving uncompersanted atmosphere pressure */
    i2c_read_blocking(i2c0, ADDRESS_BMP180, buf_raw_pressure, 3, false); /* Receive uncompersanted atmosphere pressure from BMP180 barometer sensor */

    int32_t UP = (((buf_raw_pressure[0] << 16) + (buf_raw_pressure[1] << 8) + buf_raw_pressure[2]) >> (8 - oss_mode)); /* Construction uncompersanted pressure value, according datasheet of sensor BMP180 (See page 15) */

    float p = 0; /* Value of atmosphere pressure */

    /* Calculation atmosphere pressure, according datasheet of BMP180 sensor (see in page 15) */
    int32_t B6 = B5 - 4000;
    X1 = (B2 * (B6 * B6 / 4096)) / 2048;
    X2 = AC2 * B6 / 2048;
    int32_t X3 = X1 + X2;
    int32_t B3 = (((AC1 * 4 + X3) << oss_mode) + 2) / 4;
    X1 = AC3 * B6 / 8192;
    X2 = (B1 * (B6 * B6 / 4096)) / 65536;
    X3 = ((X1 + X2) + 2) / 4;
    uint32_t B4 = (uint32_t)(AC4 * (X3 + 32768)) / 32768;
    uint32_t B7 = (uint32_t)(UP - B3) * (50000 >> oss_mode);
    if (B7 < 0x80000000) { p = (B7 * 2) / B4; }
    else { p = (B7 / B4) * 2; }
    X1 = (p / 256) * (p / 256);
    X1 = (X1 * 3038) / 65536;
    X2 = (-7357 * p) / 65536;
    p = p + (X1 + X2 + 3791) / 16;

    /* Assign temperature and pressure into own data type struct */
    data_from_bmp180 current_data = {0}; /* Initilize own data type struct */
    current_data.air_temperature_in_celisus = (float)T / 10.0; /* Air temperature in Celisus */
    current_data.atmospheric_pressure_in_Pa = p; /* Atmosphere pressure in Pascal*/

    /* Return own data of air temperature and atmosphere pressure */
    return current_data;
}