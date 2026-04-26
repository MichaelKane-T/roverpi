#ifndef SERIAL_UART_H
#define SERIAL_UART_H

#include <stddef.h>

void serial_uart_init(void);

int serial_uart_send(const char *data, size_t len);
int serial_uart_send_line(const char *line);

int serial_uart_receive(char *buffer, size_t max_len);

#endif