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

void motor_control_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL<<TB_STBY) | 
                        (1ULL<<TB_AIN1) | 
                        (1ULL<<TB_AIN2) | 
                        (1ULL<<TB_BIN1) | 
                        (1ULL<<TB_BIN2),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_conf);
    gpio_set_level(TB_STBY, 1); // Enable driver

    ledc_timer_config_t timer_conf = {
        .speed_mode = PWM_MODE,
        .timer_num = PWM_TIMER,
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t ch_a = { .gpio_num = TB_PWMA, .channel = PWM_CHANNEL_A, .speed_mode = PWM_MODE, .timer_sel = PWM_TIMER, .duty = 0 };
    ledc_channel_config(&ch_a);

    ledc_channel_config_t ch_b = { .gpio_num = TB_PWMB, .channel = PWM_CHANNEL_B, .speed_mode = PWM_MODE, .timer_sel = PWM_TIMER, .duty = 0 };
    ledc_channel_config(&ch_b);
}

void motors_forward(uint8_t speed) {
    gpio_set_level(TB_AIN1, 1); gpio_set_level(TB_AIN2, 0);
    gpio_set_level(TB_BIN1, 1); gpio_set_level(TB_BIN2, 0);
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_A, speed);
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_B, speed);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_A);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_B);
}

void motors_stop(void) {
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_A, 0);
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_B, 0);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_A);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_B);

    ESP_LOGI(TAG, "Motors stopped");
}

void motor_set_speed(uint8_t left_speed, uint8_t right_speed)
{
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_A, left_speed);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_A);

    ledc_set_duty(PWM_MODE, PWM_CHANNEL_B, right_speed);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_B);
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

void motors_turn_right(uint8_t speed)
{
    gpio_set_level(TB_AIN1, 0);
    gpio_set_level(TB_AIN2, 1);
    gpio_set_level(TB_BIN1, 1);
    gpio_set_level(TB_BIN2, 0);

    motor_set_speed(speed, speed);
}

void motors_turn_left(uint8_t speed)
{
    gpio_set_level(TB_AIN1, 1);
    gpio_set_level(TB_AIN2, 0);
    gpio_set_level(TB_BIN1, 0);
    gpio_set_level(TB_BIN2, 1);

    motor_set_speed(speed, speed);
}