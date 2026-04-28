/******************************************************************************
 * roverpi_fsm.c
 * Created:  2026-04-22
 * Author:   Michael Kane
 * Description:
 *   Finite state machine for the ESP32 side of RoverPi.
 *
 *   States
 *   ──────
 *   INIT            → STOP on first tick
 *   STOP            → DRIVE_FORWARD / DRIVE_BACKWARD / TURN_LEFT / TURN_RIGHT
 *                     on direction command (if path clear where required)
 *   DRIVE_FORWARD   → STOP if path blocked, direction changes, or fault
 *   DRIVE_BACKWARD  → STOP if direction changes or fault
 *   TURN_LEFT       → STOP if direction changes or fault
 *   TURN_RIGHT      → STOP if direction changes or fault
 *   FAULT           → stays until roverpi_fault_clear() called
 *
 *   The tick now takes drive_direction_t instead of a raw bool so all
 *   directions are first-class citizens of the FSM.
 ******************************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "roverpi_fsm.h"
#include "hardware_config.h"
#include "motor_control.h"
#include "esp_log.h"

static const char *TAG = "ROVER_FSM";

static rover_state_t current_state = STATE_INIT;
static control_mode_t current_mode = MODE_MANUAL;
static drive_direction_t current_direction = DIR_STOP;

/*====================================================================
 * Convert enum state to readable text for debugging logs.
 *====================================================================*/
static const char *state_name(rover_state_t s)
{
    switch (s) {
        case STATE_INIT:           return "INIT";
        case STATE_STOP:           return "STOP";
        case STATE_DRIVE_FORWARD:  return "DRIVE_FORWARD";
        case STATE_DRIVE_BACKWARD: return "DRIVE_BACKWARD";
        case STATE_TURN_LEFT:      return "TURN_LEFT";
        case STATE_TURN_RIGHT:     return "TURN_RIGHT";
        case STATE_FAULT:          return "FAULT";
        default:                   return "UNKNOWN";
    }
}

/*====================================================================
 * Change FSM state and log only when the state actually changes.
 *====================================================================*/
static void transition_to(rover_state_t next)
{
    if (next != current_state) {
        ESP_LOGI(TAG, "%s -> %s", state_name(current_state), state_name(next));
        current_state = next;
    }
}

/*====================================================================
 * Convert requested direction into target FSM state.
 *====================================================================*/
static rover_state_t dir_to_state(drive_direction_t dir)
{
    switch (dir) {
        case DIR_FORWARD:  return STATE_DRIVE_FORWARD;
        case DIR_BACKWARD: return STATE_DRIVE_BACKWARD;
        case DIR_LEFT:     return STATE_TURN_LEFT;
        case DIR_RIGHT:    return STATE_TURN_RIGHT;
        case DIR_STOP:
        default:           return STATE_STOP;
    }
}

/*=======================Public API=======================*/

void roverpi_fsm_init(void)
{
    current_state = STATE_INIT;
    current_mode = MODE_MANUAL;
    current_direction = DIR_STOP;

    motors_stop();
    transition_to(STATE_STOP);
}

void roverpi_set_mode(control_mode_t mode)
{   /* Always stop motors when handing over control */
    transition_to(STATE_STOP);
    current_dir = DIR_STOP;
    current_mode = mode;
}

void roverpi_fault_clear(void)
{
    if (current_state == STATE_FAULT) {
        transition_to(STATE_STOP);
    }

    current_direction = DIR_STOP;
    motors_stop();
}

rover_state_t roverpi_get_state(void)
{
    return current_state;
}

rover_state_t roverpi_set_state(rover_state_t next)
{
    transition_to(next);
    return current_state;
}

control_mode_t roverpi_get_mode(void)
{
    return control_mode;
}

drive_direction_t roverpi_get_direction(void)
{
    return current_dir;
}

/* The main FSM tick function. Call this periodically (e.g. every 100ms) with
 * the latest sensor inputs and commands from the Pi.
 *
 * path_clear: true if no obstacle detected in front (for forward command)
 * direction:   desired drive direction from Pi (DIR_STOP if no command)
 * fault_detected: true if any fault condition detected (e.g. motor stall)
 */
void roverpi_tick(bool path_clear, drive_direction_t direction, bool fault_detected)
{
    current_dir = direction;

    /*==================Transition Logic=======================*/
    switch (current_state) {

        case STATE_INIT:
            transition_to(STATE_STOP);
            break;

        case STATE_STOP:
            if (fault_detected) {
                transition_to(STATE_FAULT);
            } else if (direction == DIR_STOP) {
                /* stay stopped */
                transition_to(STATE_STOP);
            } else if (direction == DIR_FORWARD && !path_clear) {
                /* Pi asked to go forward but path is blocked — stay stopped */
                ESP_LOGW(TAG, "FORWARD blocked by obstacle");
            } else {
                transition_to(dir_to_state(direction));
            }
            break;

        case STATE_DRIVE_FORWARD:
            if (fault_detected) {
                transition_to(STATE_FAULT);
            } else if (!path_clear) {
                /* Obstacle appeared mid-drive */
                transition_to(STATE_STOP);
            } else if (direction != DIR_FORWARD) {
                /* Pi changed direction */
                transition_to(dir_to_state(direction));
            }
            break;

        case STATE_DRIVE_BACKWARD:
            if (fault_detected) {
                transition_to(STATE_FAULT);
            } else if (direction != DIR_BACKWARD) {
                transition_to(dir_to_state(direction));
            }
            break;

        case STATE_TURN_LEFT:
            if (fault_detected) {
                transition_to(STATE_FAULT);
            } else if (direction != DIR_LEFT) {
                transition_to(dir_to_state(direction));
            }
            break;

        case STATE_TURN_RIGHT:
            if (fault_detected) {
                transition_to(STATE_FAULT);
            } else if (direction != DIR_RIGHT) {
                transition_to(dir_to_state(direction));
            }
            break;

        case STATE_FAULT:
            /* Stays in FAULT until roverpi_fault_clear() called explicitly */
            transition_to(STATE_FAULT);
            break;

        default:
            transition_to(STATE_STOP);
            break;
    }

    /*==================Action Logic=======================*/
    switch (current_state) {

        case STATE_INIT:
            motors_stop();
            break;

        case STATE_STOP:
            motors_stop();
            break;

        case STATE_DRIVE_FORWARD:
            motors_forward(MOTOR_SPEED_CRUISE);
            break;

        case STATE_DRIVE_BACKWARD:
            motors_backward(MOTOR_SPEED_CRUISE);
            break;

        case STATE_TURN_LEFT:
            motors_turn_left(MOTOR_SPEED_CRUISE);
            break;

        case STATE_TURN_RIGHT:
            motors_turn_right(MOTOR_SPEED_CRUISE);
            break;

        case STATE_FAULT:
            motors_stop();
            break;

        default:
            motors_stop();
            break;
    }
}