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
    STATE_DRIVE,
    STATE_FAULT
} rover_state_t;

void roverpi_fsm_init(void);
void roverpi_tick(bool path_clear, bool drive_cmd, bool fault_detected);

void roverpi_set_mode(control_mode_t mode);
rover_state_t roverpi_get_state(void);
control_mode_t roverpi_get_mode(void);

#endif // ROVERPI_FSM_H