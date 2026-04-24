#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "driver/gpio.h" // gpio_dump_io_configuration
#include "driver/rtc_io.h" // rtc_gpio_is_valid_gpio
#include "driver/ledc.h"
#include "esp_timer.h"

#include <string.h>

#include "driver/uart.h"

static const char *TAG = "ROVERPI";

/*-------------------UART Configuration-------------------------*/
#define UART_PORT      UART_NUM_2
#define UART_TX_PIN    GPIO_NUM_17
#define UART_RX_PIN    GPIO_NUM_16
#define UART_BAUD_RATE 115200
#define UART_BUF_SIZE  1024

/*-------------------Servo Configuration-------------------------*/
#define SERVO_GPIO GPIO_NUM_18
#define SERVO_PWM_TIMER   LEDC_TIMER_1
#define SERVO_PWM_CHANNEL LEDC_CHANNEL_2
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500
#define SERVO_FREQ_HZ 50

#define SERVO_RES LEDC_TIMER_14_BIT
#define SERVO_MAX_DUTY ((1 << 14) - 1)

/*-------------------HC-SR04-------------------------*/
#define TRIG_GPIO GPIO_NUM_4
#define ECHO_GPIO GPIO_NUM_5

/* ---------------- Pin Definitions ---------------- */
#define TB_STBY   GPIO_NUM_14

#define TB_AIN1   GPIO_NUM_26
#define TB_AIN2   GPIO_NUM_27
#define TB_PWMA   GPIO_NUM_25

#define TB_BIN1   GPIO_NUM_32
#define TB_BIN2   GPIO_NUM_33
#define TB_PWMB   GPIO_NUM_13

/* ---------------- PWM Configuration ---------------- */
#define PWM_FREQ_HZ         1000
#define PWM_RESOLUTION      LEDC_TIMER_8_BIT
#define PWM_DUTY_MAX        255

#define PWM_MODE            LEDC_LOW_SPEED_MODE
#define PWM_TIMER           LEDC_TIMER_0
#define PWM_CHANNEL_A       LEDC_CHANNEL_0
#define PWM_CHANNEL_B       LEDC_CHANNEL_1

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

//HC-SR04 Initialization
static void hcsr04_init(void)
{
    // Configure TRIG pin as output
    gpio_config_t trig_conf = {
        .pin_bit_mask = (1ULL << TRIG_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&trig_conf);

    // Configure ECHO pin as input
    gpio_config_t echo_conf = {
        .pin_bit_mask = (1ULL << ECHO_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&echo_conf);
}

// Function to measure distance using HC-SR04
static float measure_distance(void)
{
    // Make sure trigger starts low
    gpio_set_level(TRIG_GPIO, 0);
    esp_rom_delay_us(2);

    // Send 10 us trigger pulse
    gpio_set_level(TRIG_GPIO, 1);
    esp_rom_delay_us(10);
    gpio_set_level(TRIG_GPIO, 0);

    // Wait for echo to go high with timeout
    int64_t t0 = esp_timer_get_time();
    while (gpio_get_level(ECHO_GPIO) == 0) {
        if (esp_timer_get_time() - t0 > 30000) {
            return -1.0f; // timeout
        }
    }

    int64_t start_time = esp_timer_get_time();

    // Wait for echo to go low with timeout
    while (gpio_get_level(ECHO_GPIO) == 1) {
        if (esp_timer_get_time() - start_time > 30000) {
            return -1.0f; // timeout
        }
    }

    int64_t end_time = esp_timer_get_time();
    int64_t pulse_us = end_time - start_time;

    float distance_cm = pulse_us * 0.0343f / 2.0f;
    return distance_cm;
}


// Set motor speeds (0-255)
static void motor_set_speed(uint8_t left_speed, uint8_t right_speed)
{
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_A, left_speed);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_A);

    ledc_set_duty(PWM_MODE, PWM_CHANNEL_B, right_speed);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_B);
}
                                                                                
// Stop both motors
static void motors_stop(void)
{
    gpio_set_level(TB_AIN1, 0);
    gpio_set_level(TB_AIN2, 0);
    gpio_set_level(TB_BIN1, 0);
    gpio_set_level(TB_BIN2, 0);

    motor_set_speed(0, 0);
    ESP_LOGI(TAG, "Motors stopped");
}

// Set motors to move forward at specified speed
static void motors_forward(uint8_t speed)
{
    gpio_set_level(TB_AIN1, 1);
    gpio_set_level(TB_AIN2, 0);

    gpio_set_level(TB_BIN1, 1);
    gpio_set_level(TB_BIN2, 0);

    motor_set_speed(speed, speed);
    ESP_LOGI(TAG, "Forward, speed=%d", speed);
}

// Set motors to move backward at specified speed
static void motors_backward(uint8_t speed)
{
    gpio_set_level(TB_AIN1, 0);
    gpio_set_level(TB_AIN2, 1);

    gpio_set_level(TB_BIN1, 0);
    gpio_set_level(TB_BIN2, 1);

    motor_set_speed(speed, speed);
    ESP_LOGI(TAG, "Backward, speed=%d", speed);
}

// Set motors to turn left at specified speed
static void motors_turn_left(uint8_t speed)
{
    gpio_set_level(TB_AIN1, 0);
    gpio_set_level(TB_AIN2, 1);

    gpio_set_level(TB_BIN1, 1);
    gpio_set_level(TB_BIN2, 0);

    motor_set_speed(speed, speed);
    ESP_LOGI(TAG, "Turning left, speed=%d", speed);
}

// Set motors to turn right at specified speed
static void motors_turn_right(uint8_t speed)
{
    gpio_set_level(TB_AIN1, 1);
    gpio_set_level(TB_AIN2, 0);

    gpio_set_level(TB_BIN1, 0);
    gpio_set_level(TB_BIN2, 1);

    motor_set_speed(speed, speed);
    ESP_LOGI(TAG, "Turning right, speed=%d", speed);
}

static void uart_init_pi(void)
{
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_BUF_SIZE, UART_BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "UART2 initialized on TX=%d RX=%d", UART_TX_PIN, UART_RX_PIN);
}

//servo init and sweep test
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500
#define SERVO_FREQ_HZ 50

#define SERVO_RES LEDC_TIMER_14_BIT
#define SERVO_MAX_DUTY ((1 << 14) - 1)

static uint32_t servo_us_to_duty(uint32_t pulse_us)
{
    // 50 Hz = 20,000 us period
    return (pulse_us * SERVO_MAX_DUTY) / 20000;
}

static void servo_write_us(uint32_t pulse_us)
{
    uint32_t duty = servo_us_to_duty(pulse_us);

    ledc_set_duty(PWM_MODE, SERVO_PWM_CHANNEL, duty);
    ledc_update_duty(PWM_MODE, SERVO_PWM_CHANNEL);
}

static void servo_init(void)
{
    ledc_timer_config_t timer_conf = {
        .speed_mode       = PWM_MODE,
        .timer_num        = SERVO_PWM_TIMER,
        .duty_resolution  = SERVO_RES,
        .freq_hz          = SERVO_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t channel_conf = {
        .gpio_num   = SERVO_GPIO,
        .speed_mode = PWM_MODE,
        .channel    = SERVO_PWM_CHANNEL,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = SERVO_PWM_TIMER,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&channel_conf);

    ESP_LOGI(TAG, "Panning servo left and right");

    // Center
    servo_write_us(1500);
    vTaskDelay(pdMS_TO_TICKS(500));

    // Left
    servo_write_us(500);
    vTaskDelay(pdMS_TO_TICKS(700));

    // Right
    servo_write_us(2500);
    vTaskDelay(pdMS_TO_TICKS(700));

    // Center again
    servo_write_us(1500);
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "Servo pan complete");
}

/* ---------------- Main application ---------------- */
void app_main(void)
{
    ESP_LOGI(TAG, "Starting initialization...");

    motor_gpio_init();
    motor_pwm_init();
    uart_init_pi();
    hcsr04_init();
    servo_init();
    ESP_LOGI(TAG, "Initialization complete. ");
    const char *startup_msg = "ESP32 READY\n";
    uart_write_bytes(UART_PORT, startup_msg, strlen(startup_msg));

    uint8_t data[UART_BUF_SIZE];

    while (1) {
        float distance = measure_distance();

        if (distance < 0) {
            ESP_LOGW(TAG, "No echo / timeout");
            motors_stop();
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (distance < 10.0f) {
            ESP_LOGW(TAG, "Obstacle too close: %.1f cm", distance);
            motors_stop();
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        int len = uart_read_bytes(UART_PORT, data, UART_BUF_SIZE - 1, pdMS_TO_TICKS(100));

        if (len > 0) {
                data[len] = '\0';

                if (len <= 1) {
                    continue;
                }

                ESP_LOGI(TAG, "Received from Pi: %s", (char *)data);

                char msg[64];
                snprintf(msg, sizeof(msg), "OK DIST:%.1f\n", distance);
                uart_write_bytes(UART_PORT, msg, strlen(msg));
            }else {
                ESP_LOGI(TAG, "No data received from Pi");
            }

        vTaskDelay(pdMS_TO_TICKS(10));


        motors_forward(180);
        vTaskDelay(pdMS_TO_TICKS(2000));

        motors_stop();
        vTaskDelay(pdMS_TO_TICKS(1000));

        motors_backward(180);
        vTaskDelay(pdMS_TO_TICKS(2000));

        motors_stop();
        vTaskDelay(pdMS_TO_TICKS(1000));

        motors_turn_left(180);
        vTaskDelay(pdMS_TO_TICKS(1500));

        motors_stop();
        vTaskDelay(pdMS_TO_TICKS(1000));

        motors_turn_right(180);
        vTaskDelay(pdMS_TO_TICKS(1500));

        motors_stop();
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
