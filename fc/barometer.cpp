#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "barometer.h"

// BMP180 Registers and Commands
#define BMP180_REG_CALIB_START 0xAA
#define BMP180_REG_CONTROL 0xF4
#define BMP180_REG_OUT_MSB 0xF6
#define BMP180_REG_OUT_LSB 0xF7
#define BMP180_REG_OUT_XLSB 0xF8

#define BMP180_CMD_TEMP 0x2E
#define BMP180_CMD_PRESS 0x34

static const uint8_t OSS = 0; // Oversampling setting

// Global variables defined here
float global_temperature = 0.0f;
float global_pressure = 0.0f;

// Calibration data
static int16_t AC1, AC2, AC3, B1, B2, MB, MC, MD;
static uint16_t AC4, AC5, AC6;
static int32_t B5_global;

// Timer structure
static repeating_timer_t barometer_timer;

static bool bmp180_read_calibration();
static bool bmp180_read_uncompensated_temp(int32_t *UT);
static bool bmp180_read_uncompensated_pressure(int32_t *UP);
static int32_t bmp180_compensate_T(int32_t UT);
static int32_t bmp180_compensate_P(int32_t UP);

// I2C helper functions
static inline bool bmp180_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    int ret = i2c_write_blocking(BARO_I2C, BMP180_ADDR, buf, 2, false);
    return (ret == 2);
}

static inline bool bmp180_read_block(uint8_t reg, uint8_t *buf, uint8_t len)
{
    int ret = i2c_write_blocking(BARO_I2C, BMP180_ADDR, &reg, 1, true);
    if (ret != 1)
        return false;
    ret = i2c_read_blocking(BARO_I2C, BMP180_ADDR, buf, len, false);
    return (ret == len);
}

void barometer_init()
{
    i2c_init(BARO_I2C, 400 * 1000);
    gpio_set_function(BARO_SDA, GPIO_FUNC_I2C);
    gpio_set_function(BARO_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(BARO_SDA);
    gpio_pull_up(BARO_SCL);
    bi_decl(bi_2pins_with_func(BARO_SDA, BARO_SCL, GPIO_FUNC_I2C));

    if (!bmp180_read_calibration())
    {
        printf("BMP180 calibration read failed!\n");
    }
}

static bool barometer_measure()
{
    int32_t UT, UP;
    if (!bmp180_read_uncompensated_temp(&UT))
    {
        return false;
    }

    int32_t T = bmp180_compensate_T(UT);

    if (!bmp180_read_uncompensated_pressure(&UP))
    {
        return false;
    }

    int32_t P = bmp180_compensate_P(UP);

    global_temperature = T / 10.0f; // T is in 0.1°C increments
    global_pressure = (float)P;     // Pressure in Pa

    return true;
}

// Timer callback
static bool barometer_timer_callback(repeating_timer_t *rt)
{
    barometer_measure();
    return true; // Keep repeating
}

void barometer_start_interrupts()
{
    // e.g. read barometer every 1000 ms
    add_repeating_timer_ms(1000, barometer_timer_callback, NULL, &barometer_timer);
}

void barometer_read()
{
    printf("Barometer: Temp: %.2f C, Pressure: %.2f Pa\n", global_temperature, global_pressure);
}

static bool bmp180_read_calibration()
{
    uint8_t buf[22];
    if (!bmp180_read_block(BMP180_REG_CALIB_START, buf, 22))
    {
        return false;
    }

    AC1 = (int16_t)((buf[0] << 8) | buf[1]);
    AC2 = (int16_t)((buf[2] << 8) | buf[3]);
    AC3 = (int16_t)((buf[4] << 8) | buf[5]);
    AC4 = (uint16_t)((buf[6] << 8) | buf[7]);
    AC5 = (uint16_t)((buf[8] << 8) | buf[9]);
    AC6 = (uint16_t)((buf[10] << 8) | buf[11]);
    B1 = (int16_t)((buf[12] << 8) | buf[13]);
    B2 = (int16_t)((buf[14] << 8) | buf[15]);
    MB = (int16_t)((buf[16] << 8) | buf[17]);
    MC = (int16_t)((buf[18] << 8) | buf[19]);
    MD = (int16_t)((buf[20] << 8) | buf[21]);

    return true;
}

static bool bmp180_read_uncompensated_temp(int32_t *UT)
{
    if (!bmp180_write_reg(BMP180_REG_CONTROL, BMP180_CMD_TEMP))
    {
        return false;
    }
    sleep_ms(5); // Wait for conversion (~4.5 ms)
    uint8_t buf[2];
    if (!bmp180_read_block(BMP180_REG_OUT_MSB, buf, 2))
    {
        return false;
    }
    *UT = ((int32_t)buf[0] << 8) | buf[1];
    return true;
}

static bool bmp180_read_uncompensated_pressure(int32_t *UP)
{
    if (!bmp180_write_reg(BMP180_REG_CONTROL, BMP180_CMD_PRESS + (OSS << 6)))
    {
        return false;
    }
    sleep_ms(5); // Wait for conversion (~4.5 ms)
    uint8_t buf[3];
    if (!bmp180_read_block(BMP180_REG_OUT_MSB, buf, 3))
    {
        return false;
    }
    int32_t up_raw = ((int32_t)buf[0] << 16 | (int32_t)buf[1] << 8 | (int32_t)buf[2]) >> (8 - OSS);
    *UP = up_raw;
    return true;
}

static int32_t bmp180_compensate_T(int32_t UT)
{
    int32_t X1 = ((UT - (int32_t)AC6) * (int32_t)AC5) >> 15;
    int32_t X2 = ((int32_t)MC << 11) / (X1 + MD);
    int32_t B5 = X1 + X2;
    B5_global = B5;
    int32_t T = (B5 + 8) >> 4; // in 0.1 °C
    return T;
}

static int32_t bmp180_compensate_P(int32_t UP)
{
    int32_t B6 = B5_global - 4000;
    int32_t X1 = ((int32_t)B2 * (B6 * B6 >> 12)) >> 11;
    int32_t X2 = ((int32_t)AC2 * B6) >> 11;
    int32_t X3 = X1 + X2;
    int32_t B3 = ((((int32_t)AC1 * 4 + X3) << OSS) + 2) / 4;

    X1 = ((int32_t)AC3 * B6) >> 13;
    X2 = ((int32_t)B1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    uint32_t B4 = ((uint32_t)AC4 * (uint32_t)(X3 + 32768)) >> 15;
    uint32_t B7 = ((uint32_t)(UP - B3) * (uint32_t)(50000 >> OSS));

    int32_t p;
    if (B7 < 0x80000000)
    {
        p = (int32_t)((B7 << 1) / B4);
    }
    else
    {
        p = (int32_t)((B7 / B4) << 1);
    }

    X1 = (p >> 8) * (p >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * p) >> 16;
    p = p + ((X1 + X2 + (int32_t)3791) >> 4);

    return p; // Pressure in Pa
}
