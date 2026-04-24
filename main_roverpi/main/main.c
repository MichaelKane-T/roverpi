/******************************************************************************
 * main.c
 * Created: 2024-06-17
 * Author: Michael Kane
 * Description:
 *   Main application entry point for RoverPi ESP32 controller.
 ******************************************************************************/

#include <stdio.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "hardware_config.h"
#include "motor_control.h"
#include "distance_sensor.h"
#include "roverpi_fsm.h"

static const char *TAG = "ROVERPI_MAIN";

/* ---------------- Task Timing ---------------- */

#define FSM_TASK_PERIOD_MS      50
#define SENSOR_TASK_PERIOD_MS   1000

/* ---------------- Shared State ---------------- */

static volatile bool path_clear = false;
static volatile bool drive_cmd = true;          // temporary test command
static volatile bool fault_detected = false;

/* ---------------- Sensor Task ---------------- */

static void sensor_task(void *arg)
{
    while (1) {
        float distance_cm = distance_sensor_get_distance_cm();

        if (distance_cm < 0.0f) {
            ESP_LOGW(TAG, "Sensor timeout");
            path_clear = false;
        } else {
            path_clear = distance_cm > OBSTACLE_STOP_CM;
            ESP_LOGI(TAG, "Distance: %.1f cm | path_clear=%d",
                     distance_cm, path_clear);
        }

        vTaskDelay(pdMS_TO_TICKS(SENSOR_TASK_PERIOD_MS));
    }
}

/* ---------------- FSM Task ---------------- */

static void fsm_task(void *arg)
{
    while (1) {
        roverpi_tick(path_clear, drive_cmd, fault_detected);

        vTaskDelay(pdMS_TO_TICKS(FSM_TASK_PERIOD_MS));
    }
}

/* ---------------- Main Application ---------------- */

void app_main(void)
{
    ESP_LOGI(TAG, "Starting RoverPi ESP32 controller...");

    motor_control_init();
    distance_sensor_init();
    roverpi_fsm_init();

    scanner_servo_sweep_test();

    xTaskCreate(
        sensor_task,
        "sensor_task",
        4096,
        NULL,
        5,
        NULL
    );

    xTaskCreate(
        fsm_task,
        "fsm_task",
        4096,
        NULL,
        6,
        NULL
    );

    ESP_LOGI(TAG, "Initialization complete. Tasks started.");
}