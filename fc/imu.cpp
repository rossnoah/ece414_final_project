#include <stdio.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "imu.h"
#include "timer.h"
#include <math.h>

#define PI 3.141592

// Define globals here (declared extern in imu.h)
double global_roll = 0.0;
double global_pitch = 0.0;
double global_yaw = 0.0;

static int addr = 0x68;

// Raw data arrays
static int16_t acceleration[3], gyro[3], temp;

static const float accel_scale = 16384.0f; // LSB/g
static const float gyro_scale = 131.0f;    // LSB/(°/s)
static const float alpha = 0.98f;

// Timer for dt calculation
static uint32_t last_update_time = 0;

// Timer structure for repeating interrupts
static repeating_timer_t imu_timer;

static void mpu6050_reset()
{
    uint8_t buf[] = {0x6B, 0x80};
    i2c_write_blocking(IMU_I2C, addr, buf, 2, false);
    sleep_ms(100);
    buf[1] = 0x00;
    i2c_write_blocking(IMU_I2C, addr, buf, 2, false);
    sleep_ms(10);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp)
{
    uint8_t buffer[6];
    uint8_t val = 0x3B;

    // Read accelerometer
    i2c_write_blocking(IMU_I2C, addr, &val, 1, true);
    i2c_read_blocking(IMU_I2C, addr, buffer, 6, false);
    for (int i = 0; i < 3; i++)
    {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Read gyro
    val = 0x43;
    i2c_write_blocking(IMU_I2C, addr, &val, 1, true);
    i2c_read_blocking(IMU_I2C, addr, buffer, 6, false);
    for (int i = 0; i < 3; i++)
    {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Read temperature
    val = 0x41;
    i2c_write_blocking(IMU_I2C, addr, &val, 1, true);
    i2c_read_blocking(IMU_I2C, addr, buffer, 2, false);
    *temp = (buffer[0] << 8 | buffer[1]);
}

static void accel_to_angle(int16_t ax, int16_t ay, int16_t az, float *acc_roll, float *acc_pitch)
{
    float ax_g = (float)ax / accel_scale;
    float ay_g = (float)ay / accel_scale;
    float az_g = (float)az / accel_scale;

    *acc_roll = atan2f(ay_g, az_g) * (180.0f / (float)PI);
    *acc_pitch = atan2f(-ax_g, sqrtf((ay_g * ay_g) + (az_g * az_g))) * (180.0f / (float)PI);
}

static bool imu_timer_callback(repeating_timer_t *rt)
{
    uint32_t now = timer_read();
    uint32_t elapsed_us = timer_elapsed_us(last_update_time, now);
    float dt_s = (float)elapsed_us / 1000000.0f;
    last_update_time = now;

    mpu6050_read_raw(acceleration, gyro, &temp);

    float gx = (float)gyro[0] / gyro_scale;
    float gy = (float)gyro[1] / gyro_scale;
    float gz = (float)gyro[2] / gyro_scale;

    float acc_roll, acc_pitch;
    accel_to_angle(acceleration[0], acceleration[1], acceleration[2], &acc_roll, &acc_pitch);

    global_roll = alpha * (global_roll + gx * dt_s) + (1.0f - alpha) * acc_roll;
    global_pitch = alpha * (global_pitch + gy * dt_s) + (1.0f - alpha) * acc_pitch;
    global_yaw += gz * dt_s;

    return true;
}

void imu_init()
{
    i2c_init(IMU_I2C, 400 * 1000);
    gpio_set_function(IMU_SDA, GPIO_FUNC_I2C);
    gpio_set_function(IMU_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_SDA);
    gpio_pull_up(IMU_SCL);
    bi_decl(bi_2pins_with_func(IMU_SDA, IMU_SCL, GPIO_FUNC_I2C));
    mpu6050_reset();
    last_update_time = timer_read();
}

void imu_start_interrupts()
{
    add_repeating_timer_ms(10, imu_timer_callback, NULL, &imu_timer);
}

void imu_read()
{
    printf("Roll: %.2f Pitch: %.2f Yaw: %.2f\n", global_roll, global_pitch, global_yaw);
}
