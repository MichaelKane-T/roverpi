/******************************************************************************
 * distance_sensor.c
 * Created: 2024-06-17
 * Author: Michael Kane
 * Description:
 *   HC-SR04 ultrasonic distance sensor and scanner servo support for RoverPi.
 ******************************************************************************/
#include "distance_sensor.h"
#include "hardware_config.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "DIST_SENSOR";

/*====================================================================
 * Convert angle in degrees to LEDC duty count.
 *
 * angle = 0   -> SERVO_MIN_DUTY
 * angle = 180 -> SERVO_MAX_DUTY
 *====================================================================*/
static uint32_t angle_to_duty(uint8_t angle_deg)
{
    if (angle_deg > 180) {
        angle_deg = 180;
    }

    return ((uint32_t)angle_deg * (SERVO_MAX_DUTY - SERVO_MIN_DUTY) / 180U)
           + SERVO_MIN_DUTY;
}

void scanner_servo_init(void)
{
    ledc_timer_config_t timer_conf = {
        .speed_mode      = PWM_MODE,
        .timer_num       = SERVO_PWM_TIMER,
        .duty_resolution = SERVO_RES,
        .freq_hz         = SERVO_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    ledc_channel_config_t ch_conf = {
        .gpio_num   = SERVO_GPIO,
        .speed_mode = PWM_MODE,
        .channel    = SERVO_PWM_CHANNEL,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = SERVO_PWM_TIMER,
        .duty       = angle_to_duty(SCANNER_SERVO_CENTER_DEG),
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_conf));

    vTaskDelay(pdMS_TO_TICKS(SCANNER_SERVO_SETTLE_MS));
    ESP_LOGI(TAG, "Scanner servo ready on GPIO %d", SERVO_GPIO);
}

void scanner_servo_set_angle(uint8_t angle_deg)
{
    uint32_t duty = angle_to_duty(angle_deg);

    ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, SERVO_PWM_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, SERVO_PWM_CHANNEL));
}

void scanner_servo_center(void)
{
    scanner_servo_set_angle(SCANNER_SERVO_CENTER_DEG);
}

void scanner_servo_left(void)
{
    scanner_servo_set_angle(SCANNER_SERVO_LEFT_DEG);
}

void scanner_servo_right(void)
{
    scanner_servo_set_angle(SCANNER_SERVO_RIGHT_DEG);
}

void scanner_servo_sweep_test(void)
{
    ESP_LOGI(TAG, "Scanner servo sweep test starting");

    ESP_LOGI(TAG, "Moving left");
    scanner_servo_left();
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "Moving center");
    scanner_servo_center();
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "Moving right");
    scanner_servo_right();
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "Moving center");
    scanner_servo_center();
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "Scanner servo sweep test complete");
}

void distance_sensor_init(void)
{
    ESP_LOGI(TAG, "Initializing HC-SR04 distance sensor");

    gpio_config_t trig_conf = {
        .pin_bit_mask = (1ULL << TRIGGER_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&trig_conf));
    gpio_set_level(TRIGGER_PIN, 0);

    gpio_config_t echo_conf = {
        .pin_bit_mask = (1ULL << ECHO_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&echo_conf));

    scanner_servo_init();

    ESP_LOGI(TAG, "Distance sensor ready: TRIG=%d ECHO=%d",
             TRIGGER_PIN, ECHO_PIN);
}

float get_distance_cm(void)
{
    /* Make sure trigger starts low. */
    gpio_set_level(TRIGGER_PIN, 0);
    esp_rom_delay_us(5);

    /* Send 10 us trigger pulse. */
    gpio_set_level(TRIGGER_PIN, 1);
    esp_rom_delay_us(TRIG_PULSE_US);
    gpio_set_level(TRIGGER_PIN, 0);

    /* Wait for echo to go high. */
    uint64_t wait_start = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) == 0) {
        if ((esp_timer_get_time() - wait_start) > HCSR04_TIMEOUT_US) {
            return -1.0f;
        }
    }

    /* Measure how long echo stays high. */
    uint64_t echo_start = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) == 1) {
        if ((esp_timer_get_time() - echo_start) > HCSR04_TIMEOUT_US) {
            return -1.0f;
        }
    }

    uint64_t echo_end = esp_timer_get_time();
    uint64_t duration_us = echo_end - echo_start;

    /* Distance = time * speed of sound / 2 because sound travels out and back. */
    float distance_cm = (duration_us * HCSR04_SOUND_CM_PER_US) / 2.0f;

    return distance_cm;
}

// Abstraction for the FSM to use
bool is_path_blocked(void) {
    float dist = get_distance_cm();
    return (dist > 0 && dist < OBSTACLE_STOP_CM);
}
