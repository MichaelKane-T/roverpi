#ifndef DISTANCE_SENSOR_H
#define DISTANCE_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize HC-SR04 ultrasonic sensor and scanner servo.
 */
void distance_sensor_init(void);

/**
 * @brief Get one distance measurement from the HC-SR04.
 *
 * @return Distance in centimeters, or -1.0f on timeout/failure.
 */
float get_distance_cm(void);

/**
 * @brief Returns true if an obstacle is closer than threshold_cm.
 */
bool is_path_blocked(void);

/**
 * @brief Initialize the scanner servo PWM.
 */
void scanner_servo_init(void);

/**
 * @brief Set the scanner servo to a specific angle (0-180 degrees).
 */
void scanner_servo_set_angle(uint8_t angle);

/**
 * @brief Center the scanner servo.
 */
void scanner_servo_center(void);

/**
 * @brief Move the scanner servo to the left.
 */
void scanner_servo_left(void);

/**
 * @brief Move the scanner servo to the right.
 */
void scanner_servo_right(void);

/**
 * @brief Sweep scanner servo left-center-right-center for test.
 */
void scanner_servo_sweep_test(void);

#endif