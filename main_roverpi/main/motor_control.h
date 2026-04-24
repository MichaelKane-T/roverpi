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

#endif