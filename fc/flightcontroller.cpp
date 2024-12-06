#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "crsf.h"

int main()
{
    stdio_init_all();
    init_crsf();

    while (1)
    {

        // process_crsf();

        sleep_ms(100);
        printf("\n\n");
        for (unsigned ch = 0; ch < USED_CHANNELS; ++ch)
        {

            printf("Channel %02u: %u\n", ch, channels[ch]);
        }
        printf("\n\n");
    }

    return 0;
}
