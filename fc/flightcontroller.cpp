#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "crsf.h"
#include "timer.h"
#include "dshot_encoder.h"
int main()
{
    stdio_init_all();
    init_crsf();

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

    while (1)
    {

        int throttle = channels[2];

        double value;

        if (throttle < 250)
        {
            value = 0;
        }
        else
        {
            double max_throttle = 2000.0; // 2k is max throttle
            value = (double)(throttle - 250) / (max_throttle - 250);
            value = value > 1.0 ? 1.0 : value;
            printf("Throttle: %d, Mapped Value: %f\n", throttle, value);
        }

        dshot.sendThrottle(value);

        // process_crsf();

        // sleep_ms(100);
        // printf("\n\n");
        // for (unsigned ch = 0; ch < USED_CHANNELS; ++ch)
        // {

        //     printf("Channel %02u: %u\n", ch, channels[ch]);
        // }
        // printf("\n\n");
    }

    return 0;
}
