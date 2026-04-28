/*
 * hardware_config.h - Pin and peripheral configuration for RoverPi
 *
 * This file defines the GPIO pins, UART settings, PWM parameters, and other
 * hardware-related constants used throughout the RoverPi firmware.
 *
 * It serves as a single source of truth for all hardware configurations, making
 * it easier to maintain and update pin assignments or peripheral settings in
 * one place.
 */


#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"

/*====================================================================
 * UART Configuration
 *
 * ESP32 UART2:
 *   TX -> Raspberry Pi RX
 *   RX -> Raspberry Pi TX
 *   GND must be shared between ESP32 and Raspberry Pi.
 *====================================================================*/
#define UART_PORT       UART_NUM_2
#define UART_TX_PIN     GPIO_NUM_17
#define UART_RX_PIN     GPIO_NUM_16
#define UART_BAUD_RATE  115200

#define UART_BUF_SIZE   1024
#define UART_BUF_RX     128
#define UART_BUF_TXRX   64

/*====================================================================
 * Scanner Servo Configuration
 *
 * For SM-S2309S-style micro servo:
 *   50 Hz PWM
 *   20 ms period
 *   safe pulse range around 900 us to 2100 us
 *
 * With 14-bit LEDC:
 *   full scale = 16384 counts
 *   900 us  -> about 737 counts
 *   2100 us -> about 1720 counts
 *====================================================================*/
#define SERVO_GPIO          GPIO_NUM_18
#define SERVO_PWM_TIMER     LEDC_TIMER_1
#define SERVO_PWM_CHANNEL   LEDC_CHANNEL_2
#define SERVO_FREQ_HZ       50
#define SERVO_RES           LEDC_TIMER_14_BIT

#define SERVO_MIN_DUTY      737
#define SERVO_MAX_DUTY      1720

#define SCANNER_SERVO_LEFT_DEG     30
#define SCANNER_SERVO_CENTER_DEG   90
#define SCANNER_SERVO_RIGHT_DEG    150
#define SCANNER_SERVO_SETTLE_MS    250

/*====================================================================
 * HC-SR04 Ultrasonic Sensor
 *====================================================================*/
#define TRIGGER_PIN                 GPIO_NUM_4
#define ECHO_PIN                    GPIO_NUM_5
#define TRIG_PULSE_US               10
#define HCSR04_TIMEOUT_US           30000
#define HCSR04_SOUND_CM_PER_US      0.0343f
#define OBSTACLE_STOP_CM            15.0f

/*====================================================================
 * TB6612FNG Motor Driver Pins
 *====================================================================*/
#define TB_STBY GPIO_NUM_14

#define TB_AIN1 GPIO_NUM_26
#define TB_AIN2 GPIO_NUM_27
#define TB_PWMA GPIO_NUM_25

#define TB_BIN1 GPIO_NUM_32
#define TB_BIN2 GPIO_NUM_33
#define TB_PWMB GPIO_NUM_13

/*====================================================================
 * Motor PWM
 *====================================================================*/
#define PWM_MODE         LEDC_LOW_SPEED_MODE
#define PWM_TIMER        LEDC_TIMER_0
#define PWM_FREQ_HZ      1000
#define PWM_RESOLUTION   LEDC_TIMER_8_BIT
#define PWM_DUTY_MAX     255

#define PWM_CHANNEL_A    LEDC_CHANNEL_0
#define PWM_CHANNEL_B    LEDC_CHANNEL_1

/*====================================================================
 * Motor Speeds
 *====================================================================*/
#define MOTOR_SPEED_MAX      255
#define MOTOR_SPEED_CRUISE   128
#define MOTOR_SPEED_STOP     0

#endif // HARDWARE_CONFIG_H