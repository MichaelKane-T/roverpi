/******************************************************************************
 * roverpi_main.c
 *
 * ESP32 motor/sensor controller for RoverPi autonomous rover.
 *
 * Architecture:
 *   Pi (brain) --UART--> ESP32 (body)
 *
 * UART protocol:
 *   Pi -> ESP32:
 *     FORWARD / BACKWARD / LEFT / RIGHT / STOP
 *     PING        -> PONG
 *     STATUS      -> STATUS dist=XX.X path=X dir=X fault=X
 *     SCAN        -> trigger full sweep
 *     SCAN_AT <d> -> single reading at angle d
 *     FAULT_CLEAR -> clears fault state
 *
 *   ESP32 -> Pi:
 *     OK <CMD>
 *     ERR FAULT
 *     ERR UNKNOWN
 *     ERR HEARTBEAT LOST
 *     OBSTACLE STOP
 *     SCAN angle=<d> dist=<cm>
 *     PONG
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "driver/gpio.h"

#include "hardware_config.h"
#include "motor_control.h"
#include "distance_sensor.h"
#include "roverpi_fsm.h"
#include "serial_uart.h"

static const char *TAG = "ROVERPI";

/* ---------------- Configuration ---------------- */

#define UART_TASK_PERIOD_MS     10
#define FSM_TASK_PERIOD_MS      50
#define GUARD_PERIOD_MS         80
#define SCAN_SETTLE_MS          200
#define HEARTBEAT_TIMEOUT_MS    2000

#define STACK_SIZE              4096
#define PRIORITY_SCAN           7
#define PRIORITY_UART           6
#define PRIORITY_FSM            5

/* ---------------- Shared State ---------------- */

typedef struct {
    bool path_clear;
    drive_direction_t direction;
    bool fault_detected;
    float distance_cm;
    TickType_t last_heartbeat;
} rover_shared_state_t;

static rover_shared_state_t g_state = {
    .path_clear = false,
    .direction = DIR_STOP,
    .fault_detected = false,
    .distance_cm = -1.0f,
    .last_heartbeat = 0,
};

static SemaphoreHandle_t g_state_mutex = NULL;

#define STATE_LOCK()   xSemaphoreTake(g_state_mutex, portMAX_DELAY)
#define STATE_UNLOCK() xSemaphoreGive(g_state_mutex)

/* ---------------- Scan Trigger Flags ---------------- */

static volatile bool scan_requested = false;
static volatile bool targeted_scan = false;
static volatile uint8_t targeted_angle_deg = SCANNER_SERVO_CENTER_DEG;

static void request_scan(void)
{
    scan_requested = true;
}

static void request_targeted_scan(uint8_t angle_deg)
{
    targeted_angle_deg = angle_deg;
    targeted_scan = true;
}

/* ---------------- Optional Fault ISR ---------------- */

#ifdef MOTOR_FAULT_GPIO

static volatile bool motor_fault_isr_flag = false;

static void IRAM_ATTR motor_fault_isr(void *arg)
{
    motor_fault_isr_flag = true;
}

static void fault_isr_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << MOTOR_FAULT_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };

    ESP_ERROR_CHECK(gpio_config(&io));
    ESP_ERROR_CHECK(gpio_isr_handler_add(MOTOR_FAULT_GPIO, motor_fault_isr, NULL));
}

#endif

/* ---------------- Scan Task ---------------- */

static void scan_task(void *arg)
{
    scanner_servo_center();
    vTaskDelay(pdMS_TO_TICKS(SCANNER_SERVO_SETTLE_MS));

    while (1) {
        if (targeted_scan) {
            targeted_scan = false;

            uint8_t angle = targeted_angle_deg;

            scanner_servo_set_angle(angle);
            vTaskDelay(pdMS_TO_TICKS(SCAN_SETTLE_MS));

            float dist = distance_sensor_get_distance_cm();
            if (dist < 0.0f) {
                dist = 0.0f;
            }

            char buf[64];
            snprintf(buf, sizeof(buf), "SCAN angle=%d dist=%.1f", angle, dist);
            serial_uart_send_line(buf);

            scanner_servo_center();
            vTaskDelay(pdMS_TO_TICKS(SCAN_SETTLE_MS));

            continue;
        }

        if (scan_requested) {
            scan_requested = false;

            const uint8_t sweep_angles[] = {
                SCANNER_SERVO_LEFT_DEG,
                SCANNER_SERVO_CENTER_DEG,
                SCANNER_SERVO_RIGHT_DEG,
            };

            for (int i = 0; i < 3; i++) {
                uint8_t angle = sweep_angles[i];

                scanner_servo_set_angle(angle);
                vTaskDelay(pdMS_TO_TICKS(SCAN_SETTLE_MS));

                float dist = distance_sensor_get_distance_cm();
                if (dist < 0.0f) {
                    dist = 0.0f;
                }

                char buf[64];
                snprintf(buf, sizeof(buf), "SCAN angle=%d dist=%.1f", angle, dist);
                serial_uart_send_line(buf);
            }

            scanner_servo_center();
            vTaskDelay(pdMS_TO_TICKS(SCAN_SETTLE_MS));

            continue;
        }

        /* Guard mode: centered distance monitoring */
        float dist = distance_sensor_get_distance_cm();

        STATE_LOCK();

        g_state.distance_cm = dist;

        if (dist < 0.0f) {
            g_state.path_clear = false;
            ESP_LOGW(TAG, "Sensor timeout: assuming obstacle");
        } else {
            g_state.path_clear = (dist > OBSTACLE_STOP_CM);
            ESP_LOGD(TAG, "Guard distance: %.1f cm path_clear=%d",
                     dist,
                     g_state.path_clear);
        }

        STATE_UNLOCK();

        vTaskDelay(pdMS_TO_TICKS(GUARD_PERIOD_MS));
    }
}

/* ---------------- UART Task ---------------- */

static void uart_task(void *arg)
{
    char rx_buf[128];

    serial_uart_send_line("ESP32 READY");

    while (1) {
        memset(rx_buf, 0, sizeof(rx_buf));

        int len = serial_uart_receive(rx_buf, sizeof(rx_buf));

        if (len > 0) {
            rx_buf[sizeof(rx_buf) - 1] = '\0';
            rx_buf[strcspn(rx_buf, "\r\n")] = '\0';

            if (strlen(rx_buf) == 0) {
                vTaskDelay(pdMS_TO_TICKS(UART_TASK_PERIOD_MS));
                continue;
            }

            ESP_LOGI(TAG, "RX: \"%s\"", rx_buf);

            if (strcmp(rx_buf, "PING") == 0) {
                STATE_LOCK();
                g_state.last_heartbeat = xTaskGetTickCount();
                STATE_UNLOCK();

                serial_uart_send_line("PONG");
            }

            else if (strcmp(rx_buf, "FORWARD") == 0) {
                STATE_LOCK();
                bool blocked = !g_state.path_clear;
                bool faulted = g_state.fault_detected;

                if (faulted) {
                    STATE_UNLOCK();
                    serial_uart_send_line("ERR FAULT");
                } else if (blocked) {
                    STATE_UNLOCK();
                    serial_uart_send_line("OBSTACLE STOP");
                } else {
                    g_state.direction = DIR_FORWARD;
                    STATE_UNLOCK();
                    serial_uart_send_line("OK FORWARD");
                }
            }

            else if (strcmp(rx_buf, "BACKWARD") == 0) {
                STATE_LOCK();
                bool faulted = g_state.fault_detected;

                if (faulted) {
                    STATE_UNLOCK();
                    serial_uart_send_line("ERR FAULT");
                } else {
                    g_state.direction = DIR_BACKWARD;
                    STATE_UNLOCK();
                    serial_uart_send_line("OK BACKWARD");
                }
            }

            else if (strcmp(rx_buf, "LEFT") == 0) {
                STATE_LOCK();
                bool faulted = g_state.fault_detected;

                if (faulted) {
                    STATE_UNLOCK();
                    serial_uart_send_line("ERR FAULT");
                } else {
                    g_state.direction = DIR_LEFT;
                    STATE_UNLOCK();
                    serial_uart_send_line("OK LEFT");
                }
            }

            else if (strcmp(rx_buf, "RIGHT") == 0) {
                STATE_LOCK();
                bool faulted = g_state.fault_detected;

                if (faulted) {
                    STATE_UNLOCK();
                    serial_uart_send_line("ERR FAULT");
                } else {
                    g_state.direction = DIR_RIGHT;
                    STATE_UNLOCK();
                    serial_uart_send_line("OK RIGHT");
                }
            }

            else if (strcmp(rx_buf, "STOP") == 0) {
                STATE_LOCK();
                g_state.direction = DIR_STOP;
                STATE_UNLOCK();

                serial_uart_send_line("OK STOP");
            }

            else if (strcmp(rx_buf, "SCAN") == 0) {
                request_scan();
                serial_uart_send_line("OK SCAN");
            }

            else if (strncmp(rx_buf, "SCAN_AT ", 8) == 0) {
                int angle = atoi(rx_buf + 8);

                if (angle < 0) {
                    angle = 0;
                }

                if (angle > 180) {
                    angle = 180;
                }

                request_targeted_scan((uint8_t)angle);

                char ack[32];
                snprintf(ack, sizeof(ack), "OK SCAN_AT %d", angle);
                serial_uart_send_line(ack);
            }

            else if (strcmp(rx_buf, "FAULT_CLEAR") == 0) {
                STATE_LOCK();
                g_state.fault_detected = false;
                g_state.direction = DIR_STOP;
                STATE_UNLOCK();

                roverpi_fault_clear();

                serial_uart_send_line("OK FAULT_CLEAR");
            }

            else if (strcmp(rx_buf, "STATUS") == 0) {
                STATE_LOCK();
                float dist = g_state.distance_cm;
                bool clear = g_state.path_clear;
                int dir = (int)g_state.direction;
                bool fault = g_state.fault_detected;
                STATE_UNLOCK();

                char status[96];
                snprintf(status,
                         sizeof(status),
                         "STATUS dist=%.1f path=%d dir=%d fault=%d",
                         dist,
                         clear,
                         dir,
                         fault);

                serial_uart_send_line(status);
            }

            else {
                serial_uart_send_line("ERR UNKNOWN");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(UART_TASK_PERIOD_MS));
    }
}

/* ---------------- FSM Task ---------------- */

static void fsm_task(void *arg)
{
    bool prev_path_clear = true;
    bool prev_fault = false;
    bool heartbeat_warned = false;
    bool scan_triggered = false;

    while (1) {
#ifdef MOTOR_FAULT_GPIO
        if (motor_fault_isr_flag) {
            motor_fault_isr_flag = false;

            STATE_LOCK();
            g_state.fault_detected = true;
            STATE_UNLOCK();
        }
#endif

        STATE_LOCK();
        bool path_clear = g_state.path_clear;
        drive_direction_t direction = g_state.direction;
        bool fault_detected = g_state.fault_detected;
        TickType_t last_hb = g_state.last_heartbeat;
        STATE_UNLOCK();

        TickType_t now = xTaskGetTickCount();
        TickType_t elapsed_ms = (now - last_hb) * portTICK_PERIOD_MS;

        if (last_hb > 0 && elapsed_ms > HEARTBEAT_TIMEOUT_MS) {
            if (!heartbeat_warned) {
                ESP_LOGW(TAG, "Pi heartbeat lost: safe stop");
                serial_uart_send_line("ERR HEARTBEAT LOST");
                heartbeat_warned = true;
            }

            STATE_LOCK();
            g_state.direction = DIR_STOP;
            STATE_UNLOCK();

            direction = DIR_STOP;
        } else {
            heartbeat_warned = false;
        }

        if (fault_detected && !prev_fault) {
            ESP_LOGE(TAG, "Motor fault: safe stop");
            serial_uart_send_line("ERR FAULT");

            STATE_LOCK();
            g_state.direction = DIR_STOP;
            STATE_UNLOCK();

            direction = DIR_STOP;
        }

        prev_fault = fault_detected;

        if (direction == DIR_FORWARD && !path_clear) {
            if (prev_path_clear) {
                ESP_LOGW(TAG, "Obstacle detected: stopping and requesting scan");
                serial_uart_send_line("OBSTACLE STOP");

                STATE_LOCK();
                g_state.direction = DIR_STOP;
                STATE_UNLOCK();

                direction = DIR_STOP;

                if (!scan_triggered) {
                    request_scan();
                    scan_triggered = true;
                }
            }
        } else {
            scan_triggered = false;
        }

        prev_path_clear = path_clear;

        roverpi_tick(path_clear, direction, fault_detected);

        vTaskDelay(pdMS_TO_TICKS(FSM_TASK_PERIOD_MS));
    }
}

/* ---------------- Entry Point ---------------- */

void app_main(void)
{
    ESP_LOGI(TAG, "RoverPi ESP32 controller starting");

    g_state_mutex = xSemaphoreCreateMutex();
    configASSERT(g_state_mutex);

    g_state.last_heartbeat = xTaskGetTickCount();

    motor_control_init();
    distance_sensor_init();
    roverpi_fsm_init(); 
    serial_uart_init();

#ifdef MOTOR_FAULT_GPIO
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    fault_isr_init();
#endif

    scanner_servo_sweep_test();

    xTaskCreate(scan_task, "scan", STACK_SIZE, NULL, PRIORITY_SCAN, NULL);
    xTaskCreate(uart_task, "uart", STACK_SIZE, NULL, PRIORITY_UART, NULL);
    xTaskCreate(fsm_task, "fsm", STACK_SIZE, NULL, PRIORITY_FSM, NULL);

    ESP_LOGI(TAG, "All tasks running");
}