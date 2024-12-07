

#define RX_UART_ID uart1
#define RX_BAUD_RATE 420000
#define UART_RX_PIN 5

#define NUM_CHANNELS 16
#define CHANNEL_BITS 11
#define CHANNEL_MASK ((1 << CHANNEL_BITS) - 1)
#define PAYLOAD_HEADER_SIZE 3 // Sync byte (1), Type (1), Length (1) [we r skipping the verification byte because that seems like extra work]
#define PAYLOAD_SIZE 25       // Header (3) + Channels (22) [Optional Arm Status (1)]
#define USED_CHANNELS 8

extern uint16_t channels[NUM_CHANNELS];

void on_crsf_uart_rx();
void process_crsf();
void init_crsf();