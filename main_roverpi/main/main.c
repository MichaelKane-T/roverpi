/*
 * roverpi_main.c
 * ESP32 motor/sensor controller for RoverPi autonomous rover.
 *
 * Pi (brain) ──UART──► ESP32 (body)
 *
 * UART protocol (Pi → ESP32):
 *   FORWARD / BACKWARD / LEFT / RIGHT / STOP
 *   PING    → PONG
 *   STATUS  → STATUS dist=XX.X path=X dir=X fault=X
 *   FAULT_CLEAR → clears fault state
 *
 * ESP32 → Pi:
 *   OK <CMD> / ERR FAULT / ERR UNKNOWN
 *   OBSTACLE STOP  (edge-triggered, not repeated)
 *   SCAN angle=<deg> dist=<cm>
 *   PONG / STATUS ...
 */

#include <stdio.h>
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

/* ─────────────────────────── Configuration ────────────────────────────── */

static const char *TAG = "ROVERPI";

#define UART_TASK_PERIOD_MS      10
#define SENSOR_TASK_PERIOD_MS    50
#define FSM_TASK_PERIOD_MS       50
#define SCAN_TASK_PERIOD_MS     600   /* full sweep period                 */
#define HEARTBEAT_TIMEOUT_MS    500

#define STACK_SIZE              4096
#define PRIORITY_SENSOR            7
#define PRIORITY_UART              6
#define PRIORITY_FSM               5
#define PRIORITY_SCAN              3   /* lowest — scan is best-effort     */

/* ─────────────────────────── Shared State ──────────────────────────────── */

typedef struct {
    bool              path_clear;
    drive_direction_t direction;
    bool              fault_detected;
    float             distance_cm;
    TickType_t        last_heartbeat;
} rover_state_t;

static rover_state_t g_state = {
    .path_clear      = false,
    .direction       = DIR_STOP,
    .fault_detected  = false,
    .distance_cm     = -1.0f,
    .last_heartbeat  = 0,
};

static SemaphoreHandle_t g_state_mutex = NULL;

#define STATE_LOCK()   xSemaphoreTake(g_state_mutex, portMAX_DELAY)
#define STATE_UNLOCK() xSemaphoreGive(g_state_mutex)

/* ─────────────────────────── Fault ISR ────────────────────────────────── */

#ifdef MOTOR_FAULT_GPIO
static void IRAM_ATTR motor_fault_isr(void *arg)
{
    g_state.fault_detected = true;
}

static void fault_isr_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << MOTOR_FAULT_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&io);
    gpio_isr_handler_add(MOTOR_FAULT_GPIO, motor_fault_isr, NULL);
}
#endif

/* ─────────────────────────── Sensor Task ──────────────────────────────── */

static void sensor_task(void *arg)
{
    while (1) {
        float dist = distance_sensor_get_distance_cm();

        STATE_LOCK();
        g_state.distance_cm = dist;
        if (dist < 0.0f) {
            g_state.path_clear = false;
            ESP_LOGW(TAG, "Sensor timeout — assuming obstacle");
        } else {
            g_state.path_clear = (dist > OBSTACLE_STOP_CM);
            ESP_LOGD(TAG, "Distance: %.1f cm  path_clear=%d",
                     dist, g_state.path_clear);
        }
        STATE_UNLOCK();

        vTaskDelay(pdMS_TO_TICKS(SENSOR_TASK_PERIOD_MS));
    }
}

/* ─────────────────────────── Scan Task ────────────────────────────────── */

/*
 * Sweeps scanner servo LEFT → CENTER → RIGHT, takes a reading at each
 * angle, and reports back to the Pi as:
 *   SCAN angle=<deg> dist=<cm>
 *
 * Pi occupancy_map.py parses these to build the exploration grid.
 */
static void scan_task(void *arg)
{
    static const uint8_t angles[] = {
        SCANNER_SERVO_LEFT_DEG,
        SCANNER_SERVO_CENTER_DEG,
        SCANNER_SERVO_RIGHT_DEG,
    };
    static const int N = sizeof(angles) / sizeof(angles[0]);

    while (1) {
        for (int i = 0; i < N; i++) {
            scanner_servo_set_angle(angles[i]);
            vTaskDelay(pdMS_TO_TICKS(SCANNER_SERVO_SETTLE_MS));

            float dist = distance_sensor_get_distance_cm();
            if (dist < 0.0f) dist = 0.0f;   /* report 0 on timeout */

            char buf[48];
            snprintf(buf, sizeof(buf), "SCAN angle=%d dist=%.1f",
                     angles[i], dist);
            serial_uart_send_line(buf);
        }

        scanner_servo_center();
        vTaskDelay(pdMS_TO_TICKS(SCAN_TASK_PERIOD_MS));
    }
}

/* ─────────────────────────── UART Task ────────────────────────────────── */

static void uart_task(void *arg)
{
    char rx_buf[128];

    serial_uart_send_line("ESP32 READY");

    while (1) {
        memset(rx_buf, 0, sizeof(rx_buf));
        int len = serial_uart_receive(rx_buf, sizeof(rx_buf));

        if (len > 0) {
            if (len < (int)sizeof(rx_buf))
                rx_buf[len] = '\0';
            else
                rx_buf[sizeof(rx_buf) - 1] = '\0';

            rx_buf[strcspn(rx_buf, "\r\n")] = '\0';

            if (strlen(rx_buf) == 0) {
                vTaskDelay(pdMS_TO_TICKS(UART_TASK_PERIOD_MS));
                continue;
            }

            ESP_LOGI(TAG, "RX: \"%s\"", rx_buf);

            /* ── Heartbeat ──────────────────────────────────────────── */
            if (strcmp(rx_buf, "PING") == 0) {
                STATE_LOCK();
                g_state.last_heartbeat = xTaskGetTickCount();
                STATE_UNLOCK();
                serial_uart_send_line("PONG");
            }

            /* ── Motion commands ────────────────────────────────────── */
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

            /* ── Fault clear ────────────────────────────────────────── */
            else if (strcmp(rx_buf, "FAULT_CLEAR") == 0) {
                STATE_LOCK();
                g_state.fault_detected = false;
                g_state.direction      = DIR_STOP;
                STATE_UNLOCK();
                roverpi_fault_clear();
                serial_uart_send_line("OK FAULT_CLEAR");
            }

            /* ── Status query ───────────────────────────────────────── */
            else if (strcmp(rx_buf, "STATUS") == 0) {
                STATE_LOCK();
                float dist    = g_state.distance_cm;
                bool  clear   = g_state.path_clear;
                int   dir     = (int)g_state.direction;
                bool  fault   = g_state.fault_detected;
                STATE_UNLOCK();

                char status[72];
                snprintf(status, sizeof(status),
                         "STATUS dist=%.1f path=%d dir=%d fault=%d",
                         dist, clear, dir, fault);
                serial_uart_send_line(status);
            }

            else {
                serial_uart_send_line("ERR UNKNOWN");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(UART_TASK_PERIOD_MS));
    }
}

/* ─────────────────────────── FSM Task ─────────────────────────────────── */

static void fsm_task(void *arg)
{
    bool      prev_path_clear  = true;
    bool      prev_fault       = false;
    bool      heartbeat_warned = false;

    while (1) {
        STATE_LOCK();
        bool              path_clear     = g_state.path_clear;
        drive_direction_t direction      = g_state.direction;
        bool              fault_detected = g_state.fault_detected;
        TickType_t        last_hb        = g_state.last_heartbeat;
        STATE_UNLOCK();

        /* ── Heartbeat watchdog ──────────────────────────────────────── */
        TickType_t now     = xTaskGetTickCount();
        TickType_t elapsed = (now - last_hb) * portTICK_PERIOD_MS;

        if (last_hb > 0 && elapsed > HEARTBEAT_TIMEOUT_MS) {
            if (!heartbeat_warned) {
                ESP_LOGW(TAG, "Pi heartbeat lost — safe stop");
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

        /* ── Fault edge notification ─────────────────────────────────── */
        if (fault_detected && !prev_fault) {
            ESP_LOGE(TAG, "Motor fault — safe stop");
            serial_uart_send_line("ERR FAULT");
            STATE_LOCK();
            g_state.direction = DIR_STOP;
            STATE_UNLOCK();
            direction = DIR_STOP;
        }
        prev_fault = fault_detected;

        /* ── Obstacle edge notification ─────────────────────────────── */
        if (direction == DIR_FORWARD && !path_clear && prev_path_clear) {
            ESP_LOGW(TAG, "Obstacle detected — stopping");
            serial_uart_send_line("OBSTACLE STOP");
        }
        prev_path_clear = path_clear;

        /* ── Advance FSM ─────────────────────────────────────────────── */
        roverpi_tick(path_clear, direction, fault_detected);

        vTaskDelay(pdMS_TO_TICKS(FSM_TASK_PERIOD_MS));
    }
}

/* ─────────────────────────── Entry Point ──────────────────────────────── */

void app_main(void)
{
    ESP_LOGI(TAG, "RoverPi ESP32 controller starting");

    motor_control_init();
    distance_sensor_init();
    roverpi_fsm_init();
    serial_uart_init();

#ifdef MOTOR_FAULT_GPIO
    gpio_install_isr_service(0);
    fault_isr_init();
#endif

    g_state_mutex = xSemaphoreCreateMutex();
    configASSERT(g_state_mutex);

    /* Seed heartbeat so watchdog doesn't fire before Pi connects */
    g_state.last_heartbeat = xTaskGetTickCount();

    /* Self-test sweep */
    scanner_servo_sweep_test();

    xTaskCreate(sensor_task, "sensor", STACK_SIZE, NULL, PRIORITY_SENSOR, NULL);
    xTaskCreate(uart_task,   "uart",   STACK_SIZE, NULL, PRIORITY_UART,   NULL);
    xTaskCreate(fsm_task,    "fsm",    STACK_SIZE, NULL, PRIORITY_FSM,    NULL);
    xTaskCreate(scan_task,   "scan",   STACK_SIZE, NULL, PRIORITY_SCAN,   NULL);

    ESP_LOGI(TAG, "All tasks running");
}