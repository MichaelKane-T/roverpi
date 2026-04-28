#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "driver/rtc_io.h" // rtc_gpio_is_valid_gpio

/*------------------- UART -------------------------*/
#define UART_PORT       UART_NUM_2
#define UART_TX_PIN     GPIO_NUM_17
#define UART_RX_PIN     GPIO_NUM_16
#define UART_BAUD_RATE  115200
#define UART_BUF_SIZE   1024

/*------------------- Servo ------------------------*/
#define SERVO_GPIO          GPIO_NUM_18
#define SERVO_PWM_TIMER     LEDC_TIMER_1
#define SERVO_PWM_CHANNEL   LEDC_CHANNEL_2
#define SERVO_FREQ_HZ       50
#define SERVO_RES           LEDC_TIMER_14_BIT

#define SERVO_MIN_DUTY      737
#define SERVO_MAX_DUTY      1720

/*------------------- HC-SR04 ----------------------*/
#define TRIGGER_PIN GPIO_NUM_4
#define ECHO_PIN GPIO_NUM_5
#define TRIG_PULSE_US 10
#define SPEED_OF_SOUND 343.0 // m/s

/*------------------- TB6612FNG --------------------*/
#define TB_STBY GPIO_NUM_14

#define TB_AIN1 GPIO_NUM_26
#define TB_AIN2 GPIO_NUM_27
#define TB_PWMA GPIO_NUM_25

#define TB_BIN1 GPIO_NUM_32
#define TB_BIN2 GPIO_NUM_33
#define TB_PWMB GPIO_NUM_13

/*------------------- Motor PWM --------------------*/
#define PWM_FREQ_HZ      1000
#define PWM_RESOLUTION   LEDC_TIMER_8_BIT
#define PWM_DUTY_MAX     255

#define PWM_MODE         LEDC_LOW_SPEED_MODE
#define PWM_TIMER        LEDC_TIMER_0
#define PWM_CHANNEL_A    LEDC_CHANNEL_0
#define PWM_CHANNEL_B    LEDC_CHANNEL_1

/*------------------- Motor Speed -------------------------*/
#define MOTOR_SPEED_MAX  255
#define MOTOR_SPEED_CRUISE  128
#define MOTOR_SPEED_STOP 0

/*------------------- Ultrasonic Sensor --------------------*/
#define HCSR04_TIMEOUT_US          30000
#define HCSR04_SOUND_CM_PER_US     0.0343f
#define OBSTACLE_STOP_CM           15.0f

/*------------------- Scanner Servo Angles -----------------*/
#define SCANNER_SERVO_LEFT_DEG     30
#define SCANNER_SERVO_CENTER_DEG   90
#define SCANNER_SERVO_RIGHT_DEG    150
#define SCANNER_SERVO_STEP_DEG     5
#define SCANNER_SERVO_SETTLE_MS    250
#define SERVO_PERIOD_US            20000

#endif // HARDWARE_CONFIG_H