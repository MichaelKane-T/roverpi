/******************************************************************************
 * distance_sensor.c
 * Created: 2024-06-17
 * Author: Michael Kane
 * Description:
 *   HC-SR04 ultrasonic distance sensor and scanner servo support for RoverPi.
 ******************************************************************************/

#include <stdbool.h>
#include <string.h>
 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
 
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
 
#include "hardware_config.h"
#include "distance_sensor.h"
 
static const char *TAG = "DIST_SENSOR";

/*========Scanner Servo Initialization===========
*
*
*/
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
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_conf));
    vTaskDelay(pdMS_TO_TICKS(SCANNER_SERVO_SETTLE_MS));
    ESP_LOGI(TAG, "Scanner servo initialised on GPIO %d", SERVO_GPIO);
}

// Helper to convert angle (0-180) to duty cycle
uint32_t angle_to_duty(int angle) {
    return (angle * (SERVO_MAX_DUTY - SERVO_MIN_DUTY) / 180) + SERVO_MIN_DUTY;
}

/*========Servo Set Angle===========
*
*
*/
void scanner_servo_set_angle(uint8_t angle_deg)
{
    if (angle_deg > 180) angle_deg = 180;
    uint32_t duty = angle_to_duty(angle_deg);
    ledc_set_duty(PWM_MODE, SERVO_PWM_CHANNEL, duty);
    ledc_update_duty(PWM_MODE, SERVO_PWM_CHANNEL);
}

/*========Servo Center===========
*
*
*/
void scanner_servo_center(void)
{
    scanner_servo_set_angle(SCANNER_SERVO_CENTER_DEG);
}

/*========Servo Left===========
*
*
*/
void scanner_servo_left(void)
{
    scanner_servo_set_angle(SCANNER_SERVO_LEFT_DEG);
}

/*========Servo Right===========
*
*
*/
void scanner_servo_right(void)
{
    scanner_servo_set_angle(SCANNER_SERVO_RIGHT_DEG);
}



/*========Servo Sweep Test===========
*
*
*/
void scanner_servo_sweep_test(void)
{
    ESP_LOGI(TAG, "Scanner servo sweep test starting...");

    // Use your named abstractions!
    ESP_LOGI(TAG, "Moving to Left");
    scanner_servo_left();
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "Moving to Center");
    scanner_servo_center();
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "Moving to Right");
    scanner_servo_right();
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "Scanner servo sweep test complete");
}


/*=========== Distance Sensor Initialization ===========*/
void distance_sensor_init(void)
{
    ESP_LOGI(TAG, "Initialising distance sensor (interrupt-driven)");
    /*
     * TRIG — output, push-pull.
     * Start low to avoid accidentally triggering a measurement during setup.
     */
    gpio_config_t trig_conf = {
        .pin_bit_mask = (1ULL << TRIGGER_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&trig_conf));
    gpio_set_level(TRIGGER_PIN, 0);
 
    /*
     * ECHO — input, pull-down enabled.
     * Pull-down prevents ghost interrupts from a floating line when
     * the sensor is not actively driving the pin high.
     */
    gpio_config_t echo_conf = {
        .pin_bit_mask = (1ULL << ECHO_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_ANYEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&echo_conf));
 
    scanner_servo_init();
 
    ESP_LOGI(TAG, "Distance sensor initialised: TRIG=%d ECHO=%d",
             TRIGGER_PIN, ECHO_PIN);
}


/*========Get Distance Measurement===========
*
*
*/
float get_distance_cm(void)
{
    // 1. Ensure trigger is low
    gpio_set_level(TRIGGER_PIN, 0);
    esp_rom_delay_us(5);

    // 2. Trigger pulse
    gpio_set_level(TRIGGER_PIN, 1);
    esp_rom_delay_us(TRIG_PULSE_US);
    gpio_set_level(TRIGGER_PIN, 0);

    // 3. Wait for Echo to start (with timeout)
    uint64_t start = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) == 0) {
        if (esp_timer_get_time() - start > 10000) return -1.0f; 
    }
    
    uint64_t echo_start = esp_timer_get_time();
    
    // 4. Wait for Echo to end
    while (gpio_get_level(ECHO_PIN) == 1) {
        if (esp_timer_get_time() - echo_start > HCSR04_TIMEOUT_US) return -1.0f;
    }
    uint64_t echo_end = esp_timer_get_time();
    
    // 5. Calculate
    uint64_t duration = echo_end - echo_start;
    float distance = (duration * HCSR04_SOUND_CM_PER_US) / 2.0f;
    
    return distance;
}
/*========== Obstacle Detection Utility ==========
*
*
*/
bool obstacle_detected(void)
{
    float distance = get_distance_cm();
    if (distance < 0.0f) return false;

    return distance <= OBSTACLE_STOP_CM;

}