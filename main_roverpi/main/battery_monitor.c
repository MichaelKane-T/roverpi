#include "battery_monitor.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hardware_config.h"

static const char *TAG = "BATT_DRIVER";

// Pin Definitions
#define BATT_EN_GPIO     GPIO_NUM_23
#define BATT_ADC_CHAN    ADC_CHANNEL_7   // GPIO35

// Hardware Math
#define R1               100000.0f    // 100k
#define R2               47000.0f     // 47k
#define DIVIDER_RATIO    ((R1 + R2) / R2)
#define ADC_VREF         3100         // 3.1V (approx range with 12dB attenuation)

static adc_oneshot_unit_handle_t adc_handle;

esp_err_t battery_monitor_init(void) {
    // 1. Setup Enable Pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BATT_EN_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 1, // Stay off
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(BATT_EN_GPIO, 0);

    // 2. Setup ADC Unit
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    // 3. Setup ADC Channel
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12, // Necessary for voltages > 1.1V
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, BATT_ADC_CHAN, &config));

    ESP_LOGI(TAG, "Battery Monitor Initialized.");
    return ESP_OK;
}

static float battery_percent_from_voltage(float batt_v)
{
    float full  = BATTERY_CELLS * CELL_FULL_V;
    float empty = BATTERY_CELLS * CELL_EMPTY_V;

    float pct = ((batt_v - empty) / (full - empty)) * 100.0f;

    if (pct < 0.0f) pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;

    return pct;
}

static battery_state_t battery_state_from_voltage(float batt_v)
{
    float cell_v = batt_v / BATTERY_CELLS;

    if (cell_v <= CELL_CRITICAL_V)
        return BATTERY_CRITICAL;

    if (cell_v <= CELL_LOW_V)
        return BATTERY_LOW;

    return BATTERY_OK;
}

float battery_monitor_get_voltage(void) {
    int adc_raw = 0;
    
    // Turn on the BJT to enable the voltage divider
    gpio_set_level(BATT_EN_GPIO, 1);
    
    // Wait for the MOSFET to fully switch and voltage to settle
    vTaskDelay(pdMS_TO_TICKS(15));

    // Read the ADC
    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, BATT_ADC_CHAN, &adc_raw));

    // Turn off the BJT immediately to save battery
    gpio_set_level(BATT_EN_GPIO, 0);

    // Convert raw to voltage
    // (adc_raw / 4095.0) * 3.1V * Divider
    float v_out = ((float)adc_raw / 4095.0f) * (ADC_VREF / 1000.0f);
    float v_batt = v_out * DIVIDER_RATIO;

    return v_batt;
}
float battery_monitor_get_percent(float batt_v)
{
    return battery_percent_from_voltage(batt_v);
}

battery_state_t battery_monitor_get_state(float batt_v)
{
    return battery_state_from_voltage(batt_v);
}

static float battery_monitor_read_adc_voltage_once(void)
{
    int adc_raw = 0;

    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, BATT_ADC_CHAN, &adc_raw));

    float v_out = ((float)adc_raw / 4095.0f) * (ADC_VREF / 1000.0f);
    return v_out * DIVIDER_RATIO;
}

battery_info_t battery_monitor_read(void)
{
    battery_info_t info;
    float sum = 0.0f;

    gpio_set_level(BATT_EN_GPIO, 1);

    // 1uF cap across 47k needs much longer than 15ms
    vTaskDelay(pdMS_TO_TICKS(200));

    // Optional: throw away first read
    (void)battery_monitor_read_adc_voltage_once();

    for (int i = 0; i < 8; i++) {
        sum += battery_monitor_read_adc_voltage_once();
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    gpio_set_level(BATT_EN_GPIO, 0);

    info.voltage = sum / 8.0f;
    info.percent = battery_percent_from_voltage(info.voltage);
    info.state   = battery_state_from_voltage(info.voltage);

    return info;
}