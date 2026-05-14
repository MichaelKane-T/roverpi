#ifndef IR_OBSTACLE_H
#define IR_OBSTACLE_H

#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    bool front_blocked;
    bool left_blocked;
    bool right_blocked;
    bool front_up_blocked;
} ir_obstacle_state_t;

/***************** Function prototypes *****************/
/**
 * @brief Initializes the IR obstacle sensors.
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t ir_obstacle_init(void);

/**
 * @brief Returns true when obstacle is detected.
 */
bool ir_obstacle_detected(void);

/**
 * @brief Reads the state of all IR sensors.
 * @return The state of all IR sensors.
 */
ir_obstacle_state_t ir_obstacle_read_all(void);

#endif