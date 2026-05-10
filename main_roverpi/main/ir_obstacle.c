#include "ir_obstacle.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "IR_OBSTACLE";

#define IR_OBSTACLE_GPIO GPIO_NUM_15

/*
 * Most HW-201 / FC-51 style IR obstacle modules output:
 * LOW  = obstacle detected
 * HIGH = no obstacle
 */
#define IR_ACTIVE_LEVEL 0

esp_err_t ir_obstacle_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << IR_OBSTACLE_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t ret = gpio_config(&io_conf);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "IR obstacle sensor initialized on GPIO%d", IR_OBSTACLE_GPIO);
    }

    return ret;
}

bool ir_obstacle_detected(void)
{
    int level = gpio_get_level(IR_OBSTACLE_GPIO);
    return level == IR_ACTIVE_LEVEL;
}