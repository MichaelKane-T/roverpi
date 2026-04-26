/*
 * roverpi_main.c
 * ESP32 motor/sensor controller for RoverPi autonomous rover.
 *
 * Architecture:
 *   Pi (brain) ──UART──► ESP32 (body)
 *
 *   sensor_task  (P7) : polls distance sensor, updates shared state
 *   uart_task    (P6) : receives Pi commands, sends status reports
 *   fsm_task     (P5) : advances roverpi FSM, enforces safety rules
 *
 * Shared state is protected by a single mutex.
 * One-shot edge notifications prevent UART spam on sustained conditions.
 */

#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

#include "esp_log.h"
#include "driver/gpio.h"

#include "hardware_config.h"
#include "motor_control.h"
#include "distance_sensor.h"
#include "roverpi_fsm.h"
#include "serial_uart.h"

/* ─────────────────────────── Configuration ────────────────────────────── */

static const char *TAG = "ROVERPI";

#define UART_TASK_PERIOD_MS       10
#define SENSOR_TASK_PERIOD_MS     50
#define FSM_TASK_PERIOD_MS        50
#define HEARTBEAT_TIMEOUT_MS     500   /* Pi silence → safe stop           */
#define FAULT_DEBOUNCE_MS         20   /* debounce window for fault GPIO   */

#define STACK_SIZE               4096
#define PRIORITY_SENSOR             7  /* highest: data must be fresh      */
#define PRIORITY_UART               6
#define PRIORITY_FSM                5

/* ─────────────────────────── Shared State ──────────────────────────────── */

typedef struct {
    bool path_clear;
    bool drive_cmd;
    bool fault_detected;
    float distance_cm;
    TickType_t last_heartbeat;   /* tick of last PING from Pi            */
} rover_state_t;

static rover_state_t g_state = {
    .path_clear      = false,
    .drive_cmd       = false,
    .fault_detected  = false,
    .distance_cm     = -1.0f,
    .last_heartbeat  = 0,
};

static SemaphoreHandle_t g_state_mutex = NULL;

/* Convenience macros — always pair Take/Give */
#define STATE_LOCK()   xSemaphoreTake(g_state_mutex, portMAX_DELAY)
#define STATE_UNLOCK() xSemaphoreGive(g_state_mutex)

/* ─────────────────────────── Fault ISR ────────────────────────────────── */

/*
 * Called from GPIO ISR when motor driver signals a fault.
 * Writes fault flag from interrupt context — intentionally no mutex
 * (bool write is atomic on LX6; mutex would be unsafe in ISR).
 */
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
        .intr_type    = GPIO_INTR_NEGEDGE,   /* active-low fault pin     */
    };
    gpio_config(&io);
    gpio_isr_handler_add(MOTOR_FAULT_GPIO, motor_fault_isr, NULL);
}

/* ─────────────────────────── Sensor Task ──────────────────────────────── */

static void sensor_task(void *arg)
{
    while (1) {
        float dist = distance_sensor_get_distance_cm();

        STATE_LOCK();
        g_state.distance_cm = dist;
        if (dist < 0.0f) {
            /* Sensor timeout — treat as blocked, log warning */
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

/* ─────────────────────────── UART Task ────────────────────────────────── */

static void uart_task(void *arg)
{
    char rx_buf[128];

    serial_uart_send_line("ESP32 READY");

    while (1) {
        memset(rx_buf, 0, sizeof(rx_buf));
        int len = serial_uart_receive(rx_buf, sizeof(rx_buf));

        if (len > 0) {
            /* Safe null-terminate then strip line endings */
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

            /* ── Heartbeat ── */
            if (strcmp(rx_buf, "PING") == 0) {
                STATE_LOCK();
                g_state.last_heartbeat = xTaskGetTickCount();
                STATE_UNLOCK();
                serial_uart_send_line("PONG");
            }

            /* ── Motion commands ── */
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
                    g_state.drive_cmd = true;
                    STATE_UNLOCK();
                    serial_uart_send_line("OK FORWARD");
                }
            }

            else if (strcmp(rx_buf, "STOP") == 0) {
                STATE_LOCK();
                g_state.drive_cmd = false;
                STATE_UNLOCK();
                serial_uart_send_line("OK STOP");
            }

            /* ── Diagnostic query ── */
            else if (strcmp(rx_buf, "STATUS") == 0) {
                STATE_LOCK();
                float  dist    = g_state.distance_cm;
                bool   clear   = g_state.path_clear;
                bool   driving = g_state.drive_cmd;
                bool   fault   = g_state.fault_detected;
                STATE_UNLOCK();

                char status[64];
                snprintf(status, sizeof(status),
                         "STATUS dist=%.1f path=%d drive=%d fault=%d",
                         dist, clear, driving, fault);
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
    bool prev_path_clear  = true;
    bool prev_fault       = false;
    bool heartbeat_warned = false;

    while (1) {
        STATE_LOCK();
        bool path_clear     = g_state.path_clear;
        bool drive_cmd      = g_state.drive_cmd;
        bool fault_detected = g_state.fault_detected;
        TickType_t last_hb  = g_state.last_heartbeat;
        STATE_UNLOCK();

        /* ── Heartbeat watchdog ────────────────────────────────────── */
        TickType_t now     = xTaskGetTickCount();
        TickType_t elapsed = (now - last_hb) * portTICK_PERIOD_MS;

        if (last_hb > 0 && elapsed > HEARTBEAT_TIMEOUT_MS) {
            if (!heartbeat_warned) {
                ESP_LOGW(TAG, "Pi heartbeat lost — safe stop");
                serial_uart_send_line("ERR HEARTBEAT LOST");
                heartbeat_warned = true;
            }
            STATE_LOCK();
            g_state.drive_cmd = false;
            STATE_UNLOCK();
            drive_cmd = false;
        } else {
            heartbeat_warned = false;
        }

        /* ── Fault handling (rising edge only) ─────────────────────── */
        if (fault_detected && !prev_fault) {
            ESP_LOGE(TAG, "Motor fault detected — safe stop");
            serial_uart_send_line("ERR FAULT");
            STATE_LOCK();
            g_state.drive_cmd = false;
            STATE_UNLOCK();
            drive_cmd = false;
        }
        prev_fault = fault_detected;

        /* ── Obstacle edge notification ─────────────────────────────── */
        if (drive_cmd && !path_clear && prev_path_clear) {
            ESP_LOGW(TAG, "Obstacle detected — stopping");
            serial_uart_send_line("OBSTACLE STOP");
        }
        prev_path_clear = path_clear;

        /* ── Advance FSM ─────────────────────────────────────────────── */
        roverpi_tick(path_clear, drive_cmd, fault_detected);

        vTaskDelay(pdMS_TO_TICKS(FSM_TASK_PERIOD_MS));
    }
}

/* ─────────────────────────── Entry Point ──────────────────────────────── */

void app_main(void)
{
    ESP_LOGI(TAG, "RoverPi ESP32 controller starting");

    /* Peripherals */
    motor_control_init();
    distance_sensor_init();
    roverpi_fsm_init();
    serial_uart_init();

    /* Fault GPIO ISR */
    gpio_install_isr_service(0);
    fault_isr_init();

    /* Shared state mutex */
    g_state_mutex = xSemaphoreCreateMutex();
    configASSERT(g_state_mutex);

    /* Initialise heartbeat so watchdog doesn't fire before Pi connects */
    g_state.last_heartbeat = xTaskGetTickCount();

    /* Servo sweep self-test */
    scanner_servo_sweep_test();

    /* Tasks — sensor highest so FSM always has fresh data */
    xTaskCreate(sensor_task, "sensor", STACK_SIZE, NULL, PRIORITY_SENSOR, NULL);
    xTaskCreate(uart_task,   "uart",   STACK_SIZE, NULL, PRIORITY_UART,   NULL);
    xTaskCreate(fsm_task,    "fsm",    STACK_SIZE, NULL, PRIORITY_FSM,    NULL);

    ESP_LOGI(TAG, "Tasks running");
}