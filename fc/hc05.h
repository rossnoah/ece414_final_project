#ifndef HC05_H
#define HC05_H

#include <stdint.h>
#include <stdbool.h>

// Define which UART and which pins you are using:
#define HC05_UART_ID uart1
#define HC05_BAUD_RATE 9600

#define HC05_TX_PIN 5 // Ensure this pin supports UART1 TX
#define HC05_RX_PIN 6 // Ensure this pin supports UART1 RX

void hc05_init();
void hc05_send_string(const char *str);

#endif
