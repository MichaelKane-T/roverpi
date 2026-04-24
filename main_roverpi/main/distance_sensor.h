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
float distance_sensor_get_distance_cm(void);

/**
 * @brief Returns true if an obstacle is closer than threshold_cm.
 */
bool distance_sensor_obstacle_detected(float threshold_cm);

/**
 * @brief Initialize the scanner servo PWM.
 */
void scanner_servo_init(void);

/**
 * @brief Set scanner servo angle from 0 to 180 degrees.
 */
void scanner_servo_set_angle(uint8_t angle_deg);

/**
 * @brief Center scanner servo.
 */
void scanner_servo_center(void);

/**
 * @brief Sweep scanner servo left-center-right-center for test.
 */
void scanner_servo_sweep_test(void);

#endif