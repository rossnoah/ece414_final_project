
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "imu.h"

double roll, pitch, yaw;

int main()
{
    stdio_init_all();
    imu_init();
    printf("Hello, MPU6050! Reading raw data from registers...\n");

    while (1)
    {

        imu_read(&roll, &pitch, &yaw);

        sleep_ms(100);
    }
}
