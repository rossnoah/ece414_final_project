#include <stdint.h>
#include "hardware/i2c.h"

#define IMU_SDA 26
#define IMU_SCL 27
#define IMU_I2C i2c1

extern double global_roll;
extern double global_pitch;
extern double global_yaw;

void imu_init();
void imu_start_interrupts();
void imu_read();
