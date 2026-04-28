/*
 * motor_control.c
 *
 * Motor control functions for RoverPi.
 *
 * This module abstracts away the low-level GPIO and PWM details of controlling
 * the TB6612FNG motor driver, providing a simple API for higher-level logic to
 * set motor speeds and directions without worrying about the underlying hardware
 * specifics. This separation of concerns makes it easier to maintain and modify
 * the motor control logic in one place without affecting other parts of the code.
 *
 * Note: The actual GPIO pin numbers and PWM configuration should be defined in
 * hardware_config.h, and this module should use those definitions to ensure
 * consistency across the codebase. Avoid hardcoding any pin numbers or PWM
 * parameters directly in this file; instead, rely on the abstractions provided
 * by hardware_config.h to allow for easy reconfiguration if needed.
 */
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

void motor_control_init(void);

void motor_set_speed(uint8_t left_speed, uint8_t right_speed);

void motors_stop(void);
void motors_forward(uint8_t speed);
void motors_backward(uint8_t speed);
void motors_turn_left(uint8_t speed);
void motors_turn_right(uint8_t speed);

#endif // MOTOR_CONTROL_H