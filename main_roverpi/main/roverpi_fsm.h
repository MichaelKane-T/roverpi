#ifndef ROVERPI_FSM_H
#define ROVERPI_FSM_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    MODE_AUTO,
    MODE_MANUAL
} control_mode_t;

typedef enum {
    STATE_INIT,
    STATE_STOP,
    STATE_DRIVE_FORWARD,
    STATE_DRIVE_BACKWARD,
    STATE_TURN_LEFT,
    STATE_TURN_RIGHT,
    STATE_FAULT
} rover_state_t;

typedef enum {
    DIR_FORWARD,
    DIR_BACKWARD,
    DIR_LEFT,
    DIR_RIGHT,
    DIR_STOP
} drive_direction_t;

void roverpi_fsm_init(void);
void roverpi_tick(bool path_clear, drive_direction_t direction, bool fault_detected);
void roverpi_set_mode(control_mode_t mode);
void roverpi_fault_clear(void);
rover_state_t      roverpi_get_state(void);
control_mode_t     roverpi_get_mode(void);
drive_direction_t  roverpi_get_direction(void);

#endif // ROVERPI_FSM_H