#ifndef RASPBERRY_soft_uart_H
#define RASPBERRY_soft_uart_H

#include <linux/tty.h>
#include<linux/slab.h>

#define SOFT_UART_MAJOR            0
#define N_PORTS                    8    // Note that the program is default to only be able to handle at most 8 ports
#define NONE                       0
#define TX_BUFFER_FLUSH_TIMEOUT 4000  // milliseconds

int raspberry_soft_uart_init(const unsigned gpio_tx[], const unsigned gpio_rx[]);
// int raspberry_soft_uart_init(const char* _gpio_tx[], const char* _gpio_rx[]);
int raspberry_soft_uart_finalize(const unsigned _gpio_tx[], const unsigned _gpio_rx[]);
// int raspberry_soft_uart_finalize(void);
int raspberry_soft_uart_open(struct tty_struct* tty);
int raspberry_soft_uart_close(struct tty_struct* tty);
int raspberry_soft_uart_set_baudrate(const int index, const int baudrate);
int raspberry_soft_uart_send_string(const int index, const unsigned char* string, const int string_size);
int raspberry_soft_uart_get_tx_queue_room(const int index);
int raspberry_soft_uart_get_tx_queue_size(const int index);
int raspberry_soft_uart_set_rx_callback(void (*callback)(unsigned char));

#endif
