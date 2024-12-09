
#include <stdint.h>
#include <stdbool.h>
#include "hardware/i2c.h"

// I2C0 pins for BMP180 (GY-68)
#define BARO_SDA 20
#define BARO_SCL 21
#define BARO_I2C i2c0

#define BMP180_ADDR 0x77

// Extern global variables
extern float global_temperature;
extern float global_pressure;

void barometer_init();
void barometer_start_interrupts();
void barometer_read();
