/*
 * serial_uart.c
 *
 * Created on: Jun 6, 2024
 * Author: Michael Kane
 * Description:
 * UART communication functions for RoverPi ESP32 controller.
 * Provides initialization, send, and receive functions for UART communication
 * between the Raspberry Pi and ESP32.
 */

#include "serial_uart.h"
#include "hardware_config.h"

#include <string.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "SERIAL_UART";

void serial_uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate  = UART_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));

    ESP_ERROR_CHECK(uart_set_pin(
        UART_PORT,
        UART_TX_PIN,
        UART_RX_PIN,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    ));

    ESP_ERROR_CHECK(uart_driver_install(
        UART_PORT,
        UART_BUF_SIZE,
        UART_BUF_SIZE,
        0,
        NULL,
        0
    ));

    ESP_LOGI(TAG, "UART ready: port=%d TX=%d RX=%d baud=%d",UART_PORT, UART_TX_PIN, UART_RX_PIN, UART_BAUD_RATE);
}

int serial_uart_send(const char *data, size_t len)
{
    if (data == NULL || len == 0) {
        return 0;
    }

    int bytes_written = uart_write_bytes(UART_PORT, data, len);

    if (bytes_written < 0) {
        ESP_LOGE(TAG, "UART write error: %d", bytes_written);
    }

    return bytes_written;
}

int serial_uart_send_line(const char *line)
{
    if (line == NULL) {
        return 0;
    }

    int total = 0;

    int written = serial_uart_send(line, strlen(line));
    if (written < 0) {
        return written;
    }
    total += written;

    written = serial_uart_send("\n", 1);
    if (written < 0) {
        return written;
    }
    total += written;

    return total;
}

int serial_uart_receive(char *buffer, size_t max_len)
{
    if (buffer == NULL || max_len < 2) {
        return -1;
    }

    int bytes_read = uart_read_bytes(
        UART_PORT,
        (uint8_t *)buffer,
        max_len - 1,
        pdMS_TO_TICKS(100)
    );

    if (bytes_read < 0) {
        ESP_LOGE(TAG, "UART read error: %d", bytes_read);
        return bytes_read;
    }

    buffer[bytes_read] = '\0';
    return bytes_read;
}