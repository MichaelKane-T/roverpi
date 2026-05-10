#ifndef IR_OBSTACLE_H
#define IR_OBSTACLE_H

#include <stdbool.h>
#include "esp_err.h"

esp_err_t ir_obstacle_init(void);

/**
 * @brief Returns true when obstacle is detected.
 */
bool ir_obstacle_detected(void);

#endif