/******************************************************************************
 * roverpi_fsm.c
 * Created: 2024-06-17
 * Author: Michael Kane
 * Description: Finite state machine for the ESP32 side of RoverPi.
 ******************************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "roverpi_fsm.h"
#include "hardware_config.h"

#include "esp_log.h"

static const char *TAG = "ROVERPI_FSM";

static rover_state_t current_state = STATE_INIT;
static control_mode_t control_mode = MODE_AUTO;

static uint32_t cnt = 0;

void roverpi_fsm_init(void)
{
    ESP_LOGI(TAG, "Initializing RoverPi FSM...");

    current_state = STATE_INIT;
    control_mode = MODE_AUTO;
    cnt = 0;
}

void roverpi_set_mode(control_mode_t mode)
{
    if (mode != control_mode) {
        ESP_LOGI(TAG, "Mode change: %d -> %d", control_mode, mode);

        // Always pass through STOP when changing control ownership.
        current_state = STATE_STOP;
        control_mode = mode;
    }
}

rover_state_t roverpi_get_state(void)
{
    return current_state;
}

control_mode_t roverpi_get_mode(void)
{
    return control_mode;
}

void roverpi_tick(bool path_clear, bool drive_cmd, bool fault_detected)
{
    // Transition logic
    switch (current_state) {
        case STATE_INIT:
            current_state = STATE_STOP;
            break;

        case STATE_STOP:
            if (fault_detected) {
                current_state = STATE_FAULT;
            } else if (path_clear && drive_cmd) {
                current_state = STATE_DRIVE;
            }
            break;

        case STATE_DRIVE:
            if (fault_detected) {
                current_state = STATE_FAULT;
            } else if (!path_clear || !drive_cmd) {
                current_state = STATE_STOP;
            }
            break;

        case STATE_FAULT:
            // Stay in fault until reset or explicit clear function later.
            break;

        default:
            current_state = STATE_STOP;
            break;
    }

    // Action logic
    switch (current_state) {
        case STATE_INIT:
            break;

        case STATE_STOP:
            ESP_LOGI(TAG, "STATE_STOP");
            // motors_stop();
            break;

        case STATE_DRIVE:
            ESP_LOGI(TAG, "STATE_DRIVE");
            // motors_forward(180);
            cnt++;
            break;

        case STATE_FAULT:
            ESP_LOGE(TAG, "STATE_FAULT");
            // motors_stop();
            break;

        default:
            break;
    }
}