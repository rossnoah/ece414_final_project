
/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "flight_controller.h"
#include <hardware/i2c.h>
// By default these devices  are on bus address 0x68
 extern  int addr;
 void mpu6050_reset();
 void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp);