#include "flight_controller_uart.h"
#include "imu.h"
#include "stdlib.h"
#include "pico/stdlib.h"
#include <hardware/i2c.h>
#include "flight_controller.h"
int main() {
    stdio_init_all();
     uart_init(UART_ID, BAUD_RATE);
    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
     gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
     gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_puts(UART_ID, "\n\nHello! is this thing on?\n");
    printf("demands to be seen and heard!\n");
    uart_puts(UART_ID," demands to be seen and heard!\n");

   #if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
    #warning i2c/mpu6050_i2c example requires a board with I2C pins
     puts("Default I2C pins were not defined");
   return 0;
   #else
    printf("Hello, MPU6050! Reading raw data from registers...\n");
    uart_puts(UART_ID, "Hello, MPU6050! Reading raw data from registers...\n");
    // This example will use I2C0 on the default SDA and SCL pins (4, 5 on a Pico)
    i2c_init(i2c0, 400000);
    gpio_set_function(IMU_SDA, GPIO_FUNC_I2C);
   gpio_set_function(IMU_SDA, GPIO_FUNC_I2C);
   gpio_pull_up(IMU_SDA);
   gpio_pull_up(IMU_SCLK);
 //Make the I2C pins available to picotool
   bi_decl(bi_2pins_with_func(IMU_SDA, IMU_SCLK, GPIO_FUNC_I2C));
   //mpu6050_reset();
   #endif
    int16_t acceleration[3], gyro[3], temp;

    while (true) {
     
        //mpu6050_read_raw(acceleration, gyro, &temp);

        // These are the raw numbers from the chip, so will need tweaking to be really useful.
        // See the datasheet for more information
       
        printf("Acc. X = %d, Y = %d, Z = %d\n", acceleration[0], acceleration[1], acceleration[2]);
        printf("Gyro. X = %d, Y = %d, Z = %d\n", gyro[0], gyro[1], gyro[2]);
        // Temperature is simple so use the datasheet calculation to get deg C.

        // Note this is chip temperature.
        printf("Temp. = %f\n", (temp / 340.0) + 36.53);
        

  //if (uart_is_readable(UART_ID)) { 
        char *GYROX,*GYROY,*GYROZ,*ACCX,*ACCY,*ACCZ,*TEMP;
        sprintf(GYROX,"%s",gyro[0]);
        sprintf(GYROY,"%s",gyro[1]);
        sprintf(GYROZ,"%s",gyro[2]);
          uart_puts(UART_ID, "GYRO. X = ,");
          uart_puts(UART_ID,GYROX);
          
           uart_puts(UART_ID,"\n\r");
           uart_puts(UART_ID, "GYRO. Y = ,");
            uart_puts(UART_ID,GYROY);
           uart_puts(UART_ID,"\n\r");
            uart_puts(UART_ID, "GYRO. Z = ,");
             uart_puts(UART_ID,GYROZ);
              uart_puts(UART_ID,"\n\r");
              sleep_ms(1000);
         // %d, Y = %d, Z = %d\n", acceleration[0], acceleration[1], acceleration[2]);
         // uart_puts(UART_ID, "Temp. = %f\n", (temp / 340.0) + 36.53");
            
      //  }
     
        sleep_ms(100);
   
    }
    }
