#ifndef RASPBERRY_soft_uart_H
#define RASPBERRY_soft_uart_H

#include <linux/tty.h>

#define soft_uart_MAJOR            0
#define N_PORTS                    8
#define NONE                       0
#define TX_BUFFER_FLUSH_TIMEOUT 4000  // milliseconds

int raspberry_soft_uart_init(const int gpio_tx[], const int gpio_rx[]);
int raspberry_soft_uart_finalize(void);
int raspberry_soft_uart_open(struct tty_struct* tty);
int raspberry_soft_uart_close(struct tty_struct* tty);
int raspberry_soft_uart_set_baudrate(const int baudrate);
int raspberry_soft_uart_send_string(int index, const unsigned char* string, int string_size);
int raspberry_soft_uart_get_tx_queue_room(int index);
int raspberry_soft_uart_get_tx_queue_size(int index);
int raspberry_soft_uart_set_rx_callback(void (*callback)(unsigned char));

#endif
