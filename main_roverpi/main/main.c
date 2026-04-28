/*=========================================================================
* roverpi_main.c
* ESP32 motor/sensor controller for RoverPi autonomous rover.
* Architecture:Raspberry Pi --UART--> ESP32
* Authors: Michael Kane
*
* RoverPi ESP32 Controller — Clean Multi-File Version

* Raspberry Pi = brain
* ESP32 = body controller
* UART between Pi and ESP32
* ESP32 owns motors, ultrasonic sensor, scanner servo, safety stop, and FSM
* Pi sends commands like "FORWARD", "BACKWARD", "LEFT", "RIGHT", "STOP", "SCAN", etc.
* roverpi_main.c
*
* ESP32 motor/sensor controller for RoverPi autonomous rover.
*
* Architecture:
*   Raspberry Pi --UART--> ESP32
*
* Pi -> ESP32 commands:
*   TICK
*   FORWARD
*   BACKWARD
*   LEFT
*   RIGHT
*   STOP
*   STATUS
*   SCAN
*   SCAN_AT <angle>
*   FAULT_CLEAR
*
* ESP32 -> Pi responses:
*   TOCK
*   OK <CMD>
*   ERR FAULT
*   ERR UNKNOWN
*   ERR HEARTBEAT LOST
*   OBSTACLE STOP
*   STATUS dist=XX.X path=X dir=X fault=X
*   SCAN angle=<d> dist=<cm>
=========================================================================*/

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

/*====================================================================
 * Task timing and priorities.
 *====================================================================*/
#define UART_TASK_PERIOD_MS     10
#define FSM_TASK_PERIOD_MS      50
#define GUARD_PERIOD_MS         80
#define SCAN_SETTLE_MS          200
#define HEARTBEAT_TIMEOUT_MS    2000

#define STACK_SIZE              4096
#define PRIORITY_UART           7
#define PRIORITY_SCAN           6
#define PRIORITY_FSM            5

#define TIMEOUT_STRIKE_MAX      3

/*====================================================================
 * Shared rover status.
 *
 * This is the state shared between UART, scan, and FSM tasks.
 * Access it only through STATE_LOCK / STATE_UNLOCK.
 *====================================================================*/
typedef struct {
    bool              path_clear;
    drive_direction_t direction;
    bool              fault_detected;
    float             distance_cm;
    TickType_t        last_heartbeat;
} rover_shared_state_t;

static rover_shared_state_t rover_state = {
    .path_clear      = false,
    .direction       = DIR_STOP,
    .fault_detected  = false,
    .distance_cm     = -1.0f,
    .last_heartbeat  = 0,
};

static SemaphoreHandle_t state_mutex = NULL;

#define STATE_LOCK()    xSemaphoreTake(state_mutex, portMAX_DELAY)
#define STATE_UNLOCK()  xSemaphoreGive(state_mutex)

/*====================================================================
 * Scan request flags.
 *
 * These are simple task-to-task request flags.
 * For a larger robot, queues/event groups would be better.
 *====================================================================*/
static volatile bool    scan_requested     = false;
static volatile bool    targeted_scan      = false;
static volatile uint8_t targeted_angle_deg = SCANNER_SERVO_CENTER_DEG;

static uint8_t timeout_strikes = 0;

static void request_scan(void)
{
    scan_requested = true;
}

static void request_targeted_scan(uint8_t angle_deg)
{
    targeted_angle_deg = angle_deg;
    targeted_scan = true;
}

/*====================================================================
 * scan_task
 *
 * Owns the HC-SR04 sensor and scanner servo.
 * No other task should directly call get_distance_cm() or move the servo.
 *====================================================================*/
static void scan_task(void *arg)
{
    (void)arg;

    scanner_servo_center();
    vTaskDelay(pdMS_TO_TICKS(SCANNER_SERVO_SETTLE_MS));

    while (1) {
        /*------------------------------------------------------------
         * Targeted scan requested by Pi: SCAN_AT <angle>
         *------------------------------------------------------------*/
        if (targeted_scan) {
            targeted_scan = false;

            uint8_t angle = targeted_angle_deg;

            scanner_servo_set_angle(angle);
            vTaskDelay(pdMS_TO_TICKS(SCAN_SETTLE_MS));

            float dist = get_distance_cm();
            if (dist < 0.0f) {
                dist = 0.0f;
            }

            char buf[UART_BUF_TXRX];
            snprintf(buf, sizeof(buf), "SCAN angle=%d dist=%.1f", angle, dist);
            serial_uart_send_line(buf);

            scanner_servo_center();
            vTaskDelay(pdMS_TO_TICKS(SCAN_SETTLE_MS));
            continue;
        }

        /*------------------------------------------------------------
         * Full sweep requested by Pi or FSM.
         *------------------------------------------------------------*/
        if (scan_requested) {
            scan_requested = false;

            static const uint8_t sweep[] = {
                SCANNER_SERVO_LEFT_DEG,
                SCANNER_SERVO_CENTER_DEG,
                SCANNER_SERVO_RIGHT_DEG,
            };

            for (int i = 0; i < 3; i++) {
                scanner_servo_set_angle(sweep[i]);
                vTaskDelay(pdMS_TO_TICKS(SCAN_SETTLE_MS));

                float dist = get_distance_cm();
                if (dist < 0.0f) {
                    dist = 0.0f;
                }

                char buf[UART_BUF_TXRX];
                snprintf(buf, sizeof(buf), "SCAN angle=%d dist=%.1f", sweep[i], dist);
                serial_uart_send_line(buf);
            }

            scanner_servo_center();
            vTaskDelay(pdMS_TO_TICKS(SCAN_SETTLE_MS));
            continue;
        }

        /*------------------------------------------------------------
         * Guard mode: keep sensor centered and continuously check path.
         *------------------------------------------------------------*/
        scanner_servo_center();

        float dist = get_distance_cm();

        STATE_LOCK();
        rover_state.distance_cm = dist;

        if (dist < 0.0f) {
            timeout_strikes++;

            if (timeout_strikes >= TIMEOUT_STRIKE_MAX) {
                timeout_strikes = TIMEOUT_STRIKE_MAX;
                rover_state.path_clear = false;
            }
        } else {
            timeout_strikes = 0;
            rover_state.path_clear = (dist > OBSTACLE_STOP_CM);
        }
        STATE_UNLOCK();

        if (dist < 0.0f) {
            ESP_LOGD(TAG, "Distance timeout strike %d/%d", timeout_strikes, TIMEOUT_STRIKE_MAX);
        } else {
            ESP_LOGD(TAG, "Distance %.1f cm", dist);
        }

        vTaskDelay(pdMS_TO_TICKS(GUARD_PERIOD_MS));
    }
}

/*====================================================================
 * uart_task
 *
 * Receives commands from Raspberry Pi and updates shared desired state.
 *====================================================================*/
static void uart_task(void *arg)
{
    (void)arg;

    char rx_buf[UART_BUF_RX];

    serial_uart_send_line("ESP32 READY");

    while (1) {
        memset(rx_buf, 0, sizeof(rx_buf));

        int len = serial_uart_receive(rx_buf, sizeof(rx_buf));

        if (len > 0) {
            /* Remove newline characters so strcmp() works cleanly. */
            rx_buf[strcspn(rx_buf, "\r\n")] = '\0';

            if (strlen(rx_buf) == 0) {
                vTaskDelay(pdMS_TO_TICKS(UART_TASK_PERIOD_MS));
                continue;
            }

            ESP_LOGI(TAG, "RX: \"%s\"", rx_buf);

            /*--------------------------------------------------------
            * Startup handshake
            *--------------------------------------------------------*/
            if (strcmp(rx_buf, "HELLO") == 0) {
                serial_uart_send_line("ESP32 READY");
            }

            /*--------------------------------------------------------
            * Heartbeat
            *--------------------------------------------------------*/
            else if (strcmp(rx_buf, "PING") == 0) {
                STATE_LOCK();
                rover_state.last_heartbeat = xTaskGetTickCount();
                STATE_UNLOCK();

                serial_uart_send_line("PONG");
            }

            /*--------------------------------------------------------
             * FORWARD: only allowed when path is clear and no fault.
             *--------------------------------------------------------*/
            else if (strcmp(rx_buf, "FORWARD") == 0) {
                                    // FIXED
                    STATE_LOCK();
                    bool blocked = !rover_state.path_clear;
                    bool faulted = rover_state.fault_detected;
                    if (faulted)       rover_state.direction = DIR_STOP;
                    else if (blocked)  rover_state.direction = DIR_STOP;
                    else               rover_state.direction = DIR_FORWARD;
                    STATE_UNLOCK();

                    // Act on snapshot — no lock held during UART TX
                    if (faulted)       serial_uart_send_line("ERR FAULT");
                    else if (blocked)  serial_uart_send_line("OBSTACLE STOP");
                    else               serial_uart_send_line("OK FORWARD");
            }

            else if (strcmp(rx_buf, "BACKWARD") == 0) {
                STATE_LOCK();
                bool faulted = rover_state.fault_detected;
                if (faulted) rover_state.direction = DIR_STOP;
                else         rover_state.direction = DIR_BACKWARD;
                STATE_UNLOCK();

                if (faulted) serial_uart_send_line("ERR FAULT");
                else         serial_uart_send_line("OK BACKWARD");
                
            }

            else if (strcmp(rx_buf, "LEFT") == 0) {
                STATE_LOCK();
                bool faulted = rover_state.fault_detected;
                if (faulted) rover_state.direction = DIR_STOP;
                else         rover_state.direction = DIR_LEFT;
                STATE_UNLOCK();

                if (faulted) serial_uart_send_line("ERR FAULT");
                else         serial_uart_send_line("OK LEFT");

            }

            else if (strcmp(rx_buf, "RIGHT") == 0) {
                STATE_LOCK();
                bool faulted = rover_state.fault_detected;
                if (faulted) rover_state.direction = DIR_STOP;
                else         rover_state.direction = DIR_RIGHT;
                STATE_UNLOCK();

                if (faulted) serial_uart_send_line("ERR FAULT");
                else         serial_uart_send_line("OK RIGHT");

            }

            else if (strcmp(rx_buf, "STOP") == 0) {
                STATE_LOCK();
                rover_state.direction = DIR_STOP;
                STATE_UNLOCK();

                serial_uart_send_line("OK STOP");
            }

            /*--------------------------------------------------------
             * Scan commands
             *--------------------------------------------------------*/
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

            /*--------------------------------------------------------
             * Fault handling
             *--------------------------------------------------------*/
            else if (strcmp(rx_buf, "FAULT_CLEAR") == 0) {
                STATE_LOCK();
                rover_state.fault_detected = false;
                rover_state.direction = DIR_STOP;
                STATE_UNLOCK();

                roverpi_fault_clear();
                serial_uart_send_line("OK FAULT_CLEAR");
            }

            /*--------------------------------------------------------
             * Status
             *--------------------------------------------------------*/
            else if (strcmp(rx_buf, "STATUS") == 0) {
                STATE_LOCK();
                float dist = rover_state.distance_cm;
                bool clear = rover_state.path_clear;
                int dir = (int)rover_state.direction;
                bool fault = rover_state.fault_detected;
                STATE_UNLOCK();

                char status[96];
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

/*====================================================================
 * fsm_task
 *
 * Applies safety logic and advances the rover FSM.
 *====================================================================*/
static void fsm_task(void *arg)
{
    (void)arg;

    bool prev_path_clear = true;
    bool prev_fault = false;
    bool heartbeat_warned = false;
    bool scan_triggered = false;

    while (1) {

        STATE_LOCK();
        bool path_clear = rover_state.path_clear;
        drive_direction_t direction = rover_state.direction;
        bool fault_detected = rover_state.fault_detected;
        TickType_t last_hb = rover_state.last_heartbeat;
        STATE_UNLOCK();

       /*------------------------------------------------------------
         * Heartbeat watchdog.
         * If Pi stops sending PING/TICK messages, ESP32 safely stops.
         * We ignore the watchdog while a SCAN is in progress to prevent 
         * timing jitter from stopping the rover mid-sweep.
         *------------------------------------------------------------*/
        TickType_t now = xTaskGetTickCount();
        uint32_t elapsed_ms = (uint32_t)((now - last_hb) * portTICK_PERIOD_MS);

        // Check if heartbeat is lost (and we aren't busy scanning)
        if ((last_hb > 0) && (elapsed_ms > HEARTBEAT_TIMEOUT_MS)) {
            
            // Only force STOP if a scan isn't currently occupying the CPU/UART
            if (!scan_requested && !targeted_scan) {
                
                if (!heartbeat_warned) {
                    ESP_LOGW(TAG, "Watchdog: Pi heartbeat lost (%u ms) — safe stop", elapsed_ms);
                    serial_uart_send_line("ERR HEARTBEAT LOST");
                    heartbeat_warned = true;
                }

                STATE_LOCK();
                rover_state.direction = DIR_STOP;
                STATE_UNLOCK();
                
                direction = DIR_STOP; // Local variable update for the FSM tick below
            }
        } else {
            // Heartbeat is healthy or hasn't timed out yet
            heartbeat_warned = false;
        }

        /*------------------------------------------------------------
         * Fault edge detection.
         *------------------------------------------------------------*/
        if (fault_detected && !prev_fault) {
            ESP_LOGE(TAG, "Motor fault — safe stop");
            serial_uart_send_line("ERR FAULT");

            STATE_LOCK();
            rover_state.direction = DIR_STOP;
            STATE_UNLOCK();

            direction = DIR_STOP;
        }
        prev_fault = fault_detected;

        /*------------------------------------------------------------
         * Obstacle handling.
         * If driving forward and path becomes blocked, stop and scan.
         *------------------------------------------------------------*/
        if ((direction == DIR_FORWARD) && !path_clear) {
            if (prev_path_clear) {
                ESP_LOGW(TAG, "Obstacle — stopping and triggering scan");
                serial_uart_send_line("OBSTACLE STOP");

                STATE_LOCK();
                rover_state.direction = DIR_STOP;
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

        /* Advance motor FSM. */
        roverpi_tick(path_clear, direction, fault_detected);

        vTaskDelay(pdMS_TO_TICKS(FSM_TASK_PERIOD_MS));
    }
}

/*====================================================================
 * app_main
 *====================================================================*/
void app_main(void)
{
    ESP_LOGI(TAG, "RoverPi ESP32 controller starting");

    state_mutex = xSemaphoreCreateMutex();
    configASSERT(state_mutex != NULL);

    motor_control_init();
    distance_sensor_init();
    roverpi_fsm_init();
    serial_uart_init();


    STATE_LOCK();
    rover_state.last_heartbeat = xTaskGetTickCount();
    STATE_UNLOCK();

    scanner_servo_sweep_test();

    xTaskCreate(uart_task, "uart", STACK_SIZE, NULL, PRIORITY_UART, NULL);
    xTaskCreate(scan_task, "scan", STACK_SIZE, NULL, PRIORITY_SCAN, NULL);
    xTaskCreate(fsm_task,  "fsm",  STACK_SIZE, NULL, PRIORITY_FSM,  NULL);

    ESP_LOGI(TAG, "All RoverPi tasks running");
}