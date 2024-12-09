#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "crsf.h"
#include "timer.h"
#include "dshot_encoder.h"
#include <math.h>

#define CHANNEL_MIDDLE 995
#define DEAD_ZONE 50
#define MAX_INPUT 1800
#define MIN_INPUT 200

double normalize_input(int input)
{
    if (abs(input - CHANNEL_MIDDLE) <= DEAD_ZONE)
    {
        return 0.0; // in deadzone return 0
    }
    else if (input > CHANNEL_MIDDLE)
    {
        return (double)(input - (CHANNEL_MIDDLE + DEAD_ZONE)) / (MAX_INPUT - (CHANNEL_MIDDLE + DEAD_ZONE));
    }
    else
    {
        return (double)(input - (CHANNEL_MIDDLE - DEAD_ZONE)) / ((CHANNEL_MIDDLE - DEAD_ZONE) - MIN_INPUT);
    }
}

int main()
{
    stdio_init_all();
    init_crsf();

    sleep_ms(500); // wait for ESC to power on and stuff

    PIO pio = pio0;

    DShotEncoder motor1(pio, 6); // back left
    DShotEncoder motor2(pio, 7); // front left
    DShotEncoder motor3(pio, 8); // front right
    DShotEncoder motor4(pio, 9); // back right

    // init motors
    if (!motor1.init(true) || !motor2.init(true) || !motor3.init(true) || !motor4.init(true))
    {
        printf("dshot init failed\n");
        while (1)
        {
            // do nothing
        }
    }

    // check if channel 4 is > 1500 = drone is armed
    while (channels[4] <= 1500)
    {
        // loop forever until we arm the drone
    }

    // arm drone by sending 0 throttle for atleast 300ms
    motor1.sendThrottle(0.0);
    motor2.sendThrottle(0.0);
    motor3.sendThrottle(0.0);
    motor4.sendThrottle(0.0);
    sleep_ms(1000);

    // startup sequence:
    // spin each motor for 1 second at 10% throttle
    // so we can identify which motor is which
    double startThrottle = 0.1; // 10% throttle
    printf("Starting motor spin-up sequence...\n");

    motor1.sendThrottle(startThrottle);
    sleep_ms(1000);
    motor1.sendThrottle(0.0);

    motor2.sendThrottle(startThrottle);
    sleep_ms(1000);
    motor2.sendThrottle(0.0);

    motor3.sendThrottle(startThrottle);
    sleep_ms(1000);
    motor3.sendThrottle(0.0);

    motor4.sendThrottle(startThrottle);
    sleep_ms(1000);
    motor4.sendThrottle(0.0);

    printf("startup complete.\n");

    while (1)
    {

        if (channels[4] <= 1500)
        {
            motor1.sendThrottle(0.0);
            motor2.sendThrottle(0.0);
            motor3.sendThrottle(0.0);
            motor4.sendThrottle(0.0);
            // disarmed kinda (for safety after drone tried to get me)
        }

        // log channels
        printf("\nchannel Values:\n");
        for (unsigned ch = 0; ch < USED_CHANNELS; ++ch)
        {
            printf("channel %02u: %u\n", ch, channels[ch]);
        }

        // normalize inputs
        double throttle = (double)(channels[2] - MIN_INPUT) / (MAX_INPUT - MIN_INPUT); // 2 is throttle
        throttle = throttle > 1.0 ? 1.0 : (throttle < 0.0 ? 0.0 : throttle);

        double pitch = normalize_input(channels[1]); // 1 is pitch
        double roll = normalize_input(channels[0]);  // 0 is roll
        double yaw = normalize_input(channels[3]);   // 3 is yaw

        printf("throttle: %f, pitch: %f, roll: %f, yaw: %f\n", throttle, pitch, roll, yaw);

        // Calculate motor outputs based on pitch, roll, yaw
        // Negate the inputs to correct direction
        double motor1Output = throttle - (-pitch) - (-roll) + (-yaw); // back left
        double motor2Output = throttle + (-pitch) - (-roll) - (-yaw); // front left
        double motor3Output = throttle + (-pitch) + (-roll) + (-yaw); // front right
        double motor4Output = throttle - (-pitch) + (-roll) - (-yaw); // back right

        // clamp range
        motor1Output = motor1Output > 1.0 ? 1.0 : (motor1Output < 0.0 ? 0.0 : motor1Output);
        motor2Output = motor2Output > 1.0 ? 1.0 : (motor2Output < 0.0 ? 0.0 : motor2Output);
        motor3Output = motor3Output > 1.0 ? 1.0 : (motor3Output < 0.0 ? 0.0 : motor3Output);
        motor4Output = motor4Output > 1.0 ? 1.0 : (motor4Output < 0.0 ? 0.0 : motor4Output);

        // send to motors
        motor1.sendThrottle(motor1Output);
        motor2.sendThrottle(motor2Output);
        motor3.sendThrottle(motor3Output);
        motor4.sendThrottle(motor4Output);

        printf("motor outputs: m1=%f, m2=%f, m3=%f, m4=%f\n", motor1Output, motor2Output, motor3Output, motor4Output);
    }

    return 0;
}
