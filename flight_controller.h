#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H
#include "pico/stdlib.h"
//This section dec;lares the pins to which the pico delivers pwm 
//signaals to the ESC

#define FRONT_LEFT 
#define FRONT_RIGHT
#define BACK_LEFT
#define BACK_RIGHT 
//GYROSCOPE AND ACCELEROMETER CONNECTIONS
#define IMU_SDA
#define IMU_SCLK
//UV_SENSOR CONNECTION
#define UVS_TRIG
#define UVS_ECHO
//BLUETOOTH CONNECTOION
#define BLUETOOTH_RX
#define BLUETOOTH_TX
#define BLUETOOTH_STATE
//BAROMETER CONNECTION
#define BAROMETER_SDA
#define BAROMETER_SCLK
//RECEIVER CONNECTION
#define RECEIVER_UART0
#define RECEIVER_UART1 

#endif