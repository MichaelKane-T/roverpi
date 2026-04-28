#ifndef SERIAL_UART_H
#define SERIAL_UART_H
#include <stddef.h>

void serial_uart_init(void);

/* Returns number of bytes sent, or negative value on error. */
int serial_uart_send(const char *data, size_t len);

/* Sends string followed by '\n'. Returns total bytes sent. */
int serial_uart_send_line(const char *line);

/* Returns number of bytes received, 0 on timeout, or negative value on error. */
int serial_uart_receive(char *buffer, size_t max_len);

#endif // SERIAL_UART_H
