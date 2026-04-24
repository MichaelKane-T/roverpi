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

static uint32_t servo_us_to_duty(uint32_t pulse_us)
{
    return (pulse_us * SERVO_MAX_DUTY) / SERVO_PERIOD_US;
}

static void scanner_servo_write_us(uint32_t pulse_us)
{
    if (pulse_us < SERVO_MIN_US) {
        pulse_us = SERVO_MIN_US;
    }

    if (pulse_us > SERVO_MAX_US) {
        pulse_us = SERVO_MAX_US;
    }

    uint32_t duty = servo_us_to_duty(pulse_us);

    ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, SERVO_PWM_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, SERVO_PWM_CHANNEL));
}

void scanner_servo_set_angle(uint8_t angle_deg)
{
    if (angle_deg > 180) {
        angle_deg = 180;
    }

    uint32_t pulse_us = SERVO_MIN_US +
        ((SERVO_MAX_US - SERVO_MIN_US) * angle_deg) / 180;

    scanner_servo_write_us(pulse_us);
}

void scanner_servo_center(void)
{
    scanner_servo_set_angle(SCANNER_SERVO_CENTER_DEG);
}

void scanner_servo_init(void)
{
    ledc_timer_config_t timer_conf = {
        .speed_mode       = PWM_MODE,
        .timer_num        = SERVO_PWM_TIMER,
        .duty_resolution  = SERVO_RES,
        .freq_hz          = SERVO_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };

    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    ledc_channel_config_t channel_conf = {
        .gpio_num   = SERVO_GPIO,
        .speed_mode = PWM_MODE,
        .channel    = SERVO_PWM_CHANNEL,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = SERVO_PWM_TIMER,
        .duty       = 0,
        .hpoint     = 0
    };

    ESP_ERROR_CHECK(ledc_channel_config(&channel_conf));

    scanner_servo_center();
    vTaskDelay(pdMS_TO_TICKS(SCANNER_SERVO_SETTLE_MS));

    ESP_LOGI(TAG, "Scanner servo initialized on GPIO %d", SERVO_GPIO);
}

void scanner_servo_sweep_test(void)
{
    ESP_LOGI(TAG, "Running scanner servo sweep test");

    scanner_servo_set_angle(SCANNER_SERVO_CENTER_DEG);
    vTaskDelay(pdMS_TO_TICKS(SCANNER_SERVO_SETTLE_MS));

    scanner_servo_set_angle(SCANNER_SERVO_LEFT_DEG);
    vTaskDelay(pdMS_TO_TICKS(SCANNER_SERVO_SETTLE_MS));

    scanner_servo_set_angle(SCANNER_SERVO_RIGHT_DEG);
    vTaskDelay(pdMS_TO_TICKS(SCANNER_SERVO_SETTLE_MS));

    scanner_servo_set_angle(SCANNER_SERVO_CENTER_DEG);
    vTaskDelay(pdMS_TO_TICKS(SCANNER_SERVO_SETTLE_MS));

    ESP_LOGI(TAG, "Scanner servo sweep test complete");
}

static void hcsr04_gpio_init(void)
{
    gpio_config_t trig_conf = {
        .pin_bit_mask = (1ULL << TRIG_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&trig_conf));

    gpio_config_t echo_conf = {
        .pin_bit_mask = (1ULL << ECHO_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&echo_conf));

    gpio_set_level(TRIG_GPIO, 0);

    ESP_LOGI(TAG, "HC-SR04 initialized: TRIG=%d ECHO=%d", TRIG_GPIO, ECHO_GPIO);
}

void distance_sensor_init(void)
{
    ESP_LOGI(TAG, "Initializing distance sensor system");

    hcsr04_gpio_init();
    scanner_servo_init();

    ESP_LOGI(TAG, "Distance sensor system initialized");
}

float distance_sensor_get_distance_cm(void)
{
    gpio_set_level(TRIG_GPIO, 0);
    esp_rom_delay_us(2);

    gpio_set_level(TRIG_GPIO, 1);
    esp_rom_delay_us(10);
    gpio_set_level(TRIG_GPIO, 0);

    int64_t wait_start = esp_timer_get_time();

    while (gpio_get_level(ECHO_GPIO) == 0) {
        if ((esp_timer_get_time() - wait_start) > HCSR04_TIMEOUT_US) {
            ESP_LOGW(TAG, "HC-SR04 timeout waiting for echo HIGH");
            return -1.0f;
        }
    }

    int64_t echo_start = esp_timer_get_time();

    while (gpio_get_level(ECHO_GPIO) == 1) {
        if ((esp_timer_get_time() - echo_start) > HCSR04_TIMEOUT_US) {
            ESP_LOGW(TAG, "HC-SR04 timeout waiting for echo LOW");
            return -1.0f;
        }
    }

    int64_t echo_end = esp_timer_get_time();
    int64_t pulse_us = echo_end - echo_start;

    float distance_cm = (pulse_us * HCSR04_SOUND_CM_PER_US) / 2.0f;

    ESP_LOGI(TAG, "Distance: %.1f cm", distance_cm);

    return distance_cm;
}

bool distance_sensor_obstacle_detected(float threshold_cm)
{
    float distance_cm = distance_sensor_get_distance_cm();

    if (distance_cm < 0.0f) {
        return false;
    }

    return distance_cm <= threshold_cm;
}