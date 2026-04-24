/* motor_control.c 
 * Created: 2024-06-17
 * Author: Michael Kane
 * Description: Motor control functions for RoverPi.
*/

#include "motor_control.h"
#include "hardware_config.h"
#include <stdbool.h>
#include "esp_log.h"

static const char *TAG = "MOTOR_CONTROL";


/* ---------------- Motor Helpers ---------------- */
static void motor_gpio_init(void)
{   
    // Configure motor control pins as outputs
    gpio_config_t io_conf = {
        .pin_bit_mask =
            (1ULL << TB_STBY) |
            (1ULL << TB_AIN1) |
            (1ULL << TB_AIN2) |
            (1ULL << TB_BIN1) |
            (1ULL << TB_BIN2) ,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };gpio_config(&io_conf);

    // Set initial states
    gpio_set_level(TB_STBY, 1);   // Wake up TB6612FNG
    gpio_set_level(TB_AIN1, 0);
    gpio_set_level(TB_AIN2, 0);
    gpio_set_level(TB_BIN1, 0);
    gpio_set_level(TB_BIN2, 0);
}

// Initialize LEDC for motor PWM control
static void motor_pwm_init(void)
{
    // Configure LEDC timer
    ledc_timer_config_t timer_conf = {
        .speed_mode       = PWM_MODE,
        .timer_num        = PWM_TIMER,
        .duty_resolution  = PWM_RESOLUTION,
        .freq_hz          = PWM_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    // Configure LEDC channels for both motors
    ledc_channel_config_t channel_a = {
        .gpio_num   = TB_PWMA,
        .speed_mode = PWM_MODE,
        .channel    = PWM_CHANNEL_A,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = PWM_TIMER,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&channel_a);

    // Configure channel for motor B
    ledc_channel_config_t channel_b = {
        .gpio_num   = TB_PWMB,
        .speed_mode = PWM_MODE,
        .channel    = PWM_CHANNEL_B,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = PWM_TIMER,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&channel_b);
}

void motor_control_init(void)
{
    ESP_LOGI(TAG, "Initializing motor control...");
    motor_gpio_init();
    motor_pwm_init();
    ESP_LOGI(TAG, "Motor control initialization complete.");
}
void motor_set_speed(uint8_t left_speed, uint8_t right_speed)
{
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_A, left_speed);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_A);

    ledc_set_duty(PWM_MODE, PWM_CHANNEL_B, right_speed);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_B);
}

void motors_stop(void)
{
    gpio_set_level(TB_AIN1, 0);
    gpio_set_level(TB_AIN2, 0);
    gpio_set_level(TB_BIN1, 0);
    gpio_set_level(TB_BIN2, 0);

    motor_set_speed(0, 0);
    ESP_LOGI(TAG, "Motors stopped");
}

void motors_forward(uint8_t speed)
{
    gpio_set_level(TB_AIN1, 1);
    gpio_set_level(TB_AIN2, 0);
    gpio_set_level(TB_BIN1, 1);
    gpio_set_level(TB_BIN2, 0);

    motor_set_speed(speed, speed);
}

void motors_backward(uint8_t speed)
{
    gpio_set_level(TB_AIN1, 0);
    gpio_set_level(TB_AIN2, 1);
    gpio_set_level(TB_BIN1, 0);
    gpio_set_level(TB_BIN2, 1);

    motor_set_speed(speed, speed);
}

void motors_turn_left(uint8_t speed)
{
    gpio_set_level(TB_AIN1, 0);
    gpio_set_level(TB_AIN2, 1);
    gpio_set_level(TB_BIN1, 1);
    gpio_set_level(TB_BIN2, 0);

    motor_set_speed(speed, speed);
}

void motors_turn_right(uint8_t speed)
{
    gpio_set_level(TB_AIN1, 1);
    gpio_set_level(TB_AIN2, 0);
    gpio_set_level(TB_BIN1, 0);
    gpio_set_level(TB_BIN2, 1);

    motor_set_speed(speed, speed);
}