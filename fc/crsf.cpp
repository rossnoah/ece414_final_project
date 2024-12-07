#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "crsf.h"

static uint8_t buffer[PAYLOAD_SIZE] = {0};
uint16_t channels[NUM_CHANNELS];
static bool armStatus = false;
static int buffer_index = 0;
static bool frame_complete = false;

static void UnpackChannels(const uint8_t *payload, uint16_t *channels, bool *armStatus)
{
    // code from BetaFlight rx/crsf.cpp / bitpacker_unpack
    uint8_t bitsMerged = 0;
    uint32_t readValue = 0;
    unsigned readByteIndex = 0;

    for (uint8_t n = 0; n < NUM_CHANNELS; n++)
    {
        while (bitsMerged < CHANNEL_BITS)
        {
            readValue |= ((uint32_t)payload[readByteIndex++]) << bitsMerged;
            bitsMerged += 8;
        }
        channels[n] = (readValue & CHANNEL_MASK);
        readValue >>= CHANNEL_BITS;
        bitsMerged -= CHANNEL_BITS;
    }

    if (readByteIndex < 19)
    {
        *armStatus = payload[readByteIndex] != 0;
    }
    else
    {
        *armStatus = false;
    }
}

// UART interrupt handler
void on_crsf_uart_rx()
{
    // printf("on uart\n");
    while (uart_is_readable(RX_UART_ID))
    {
        uint8_t byte = uart_getc(RX_UART_ID);

        if (buffer_index == 0)
        {
            if (byte != 0xC8 && byte != 0xEE)
            {
                // printf("failed to sync byte 0x%02X\n", byte);
                continue; // Sync byte check
            }
            else
            {
                // printf("found sync byte\n");
            }
        }

        buffer[buffer_index++] = byte;

        // Check if payload length is valid and frame is complete
        if (buffer_index == 3)
        {
            if (buffer[2] != 0x16)
            {
                buffer_index = 0; // Reset for invalid frame
                // printf("invalid frame");
                continue;
            }
        }
        else if (buffer_index == buffer[1] + PAYLOAD_HEADER_SIZE)
        {
            frame_complete = true;
            process_crsf();
            buffer_index = 0;
        }
    }
}

void process_crsf()
{
    if (frame_complete)
    {
        frame_complete = false;
        UnpackChannels(&buffer[3], channels, &armStatus);

        // Print the parsed data
        // printf("Channels:\n");
        for (unsigned ch = 0; ch < NUM_CHANNELS; ++ch)
        {

            // if (ch <= 7)
            // printf("Channel %02u: %u\n", ch, channels[ch]);
        }
        // printf("Arm Status: %s\n", armStatus ? "Armed" : "Disarmed");
    }
}

void init_crsf()
{

    // Initialize UART
    uart_init(RX_UART_ID, RX_BAUD_RATE);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Enable UART interrupt
    uart_set_irq_enables(RX_UART_ID, true, false);
    irq_set_exclusive_handler(UART1_IRQ, on_crsf_uart_rx);
    irq_set_enabled(UART1_IRQ, true);

    printf("UART initialized with interrupts. Listening for CRSF frames...\n");
}