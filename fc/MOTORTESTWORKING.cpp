#include <stdio.h>
#include "pico/stdlib.h"
#include "timer.h"
#include "dshot_encoder.h"

int main()
{
    stdio_init_all();
    sleep_ms(2000); // wait 2 sec for esc to power on any or anything else that needs to happen

    PIO pio = pio0;
    DShotEncoder dshot(pio, 9); // gpio pin 9 for motor 1 testing

    // if init fails just loop forever
    if (!dshot.init(true))
    {
        printf("Failed to init DShot\n");
        while (1)
        {
        }
    }

    // arm drone by sending 0 throttle for 300ms+  (2 sec in this case)
    dshot.sendThrottle(0.0);
    sleep_ms(2000);

    // ramp up speed test
    int steps = 10000;
    for (int i = 0; i < steps; i++)
    {
        double t = (double)i / (double)steps; // t: 0.0 to 1.0
        dshot.sendThrottle(t);
        sleep_ms(1);
    }

    // ramp down speed
    for (int i = 0; i < steps; i++)
    {
        double t = 1.0 - (double)i / (double)steps; // t: 1.0 to 0.0
        dshot.sendThrottle(t);
        sleep_ms(1);
    }

    // turn off after test
    dshot.sendThrottle(0.0);

    return 0;
}
