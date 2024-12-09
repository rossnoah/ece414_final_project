#include "hc05.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include <string.h>

void hc05_init()
{
    // Initialize the chosen UART with given baud rate
    uart_init(HC05_UART_ID, HC05_BAUD_RATE);

    // Set the UART pins
    // For UART1, common pins:
    //   TX can be on GP4 or GP8 (and some others),
    //   RX can be on GP5 or GP9 (and others).
    // Please verify if GP5 and GP6 are valid for UART usage. If not, use a known valid mapping.
    gpio_set_function(HC05_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(HC05_RX_PIN, GPIO_FUNC_UART);

    // You can set UART format if needed (default is 8N1)
    uart_set_format(HC05_UART_ID, 8, 1, UART_PARITY_NONE);

    // Optional: Set FIFO enabled/disabled
    uart_set_fifo_enabled(HC05_UART_ID, true);
}

void hc05_send_string(const char *str)
{
    // Send the provided string out over the UART
    // This will block until all characters are sent.
    for (size_t i = 0; i < strlen(str); i++)
    {
        uart_putc(HC05_UART_ID, str[i]);
    }
}
