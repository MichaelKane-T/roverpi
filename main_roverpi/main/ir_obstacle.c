#include "ir_obstacle.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "hardware_config.h"

static const char *TAG = "IR_OBSTACLE";

#define IR_ACTIVE_LEVEL 0

esp_err_t ir_obstacle_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask =
            (1ULL << IR_OBSTACLE_FRONT) |
            (1ULL << IR_OBSTACLE_LEFT)  |
            (1ULL << IR_OBSTACLE_RIGHT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t ret = gpio_config(&io_conf);

    gpio_config_t up_conf = {
        .pin_bit_mask = (1ULL << IR_OBSTACLE_FRONT_UP),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ret |= gpio_config(&up_conf);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "IR sensors initialized: front=%d left=%d right=%d",
                 IR_OBSTACLE_FRONT, IR_OBSTACLE_LEFT, IR_OBSTACLE_RIGHT);
    }

    return ret;
}

bool ir_obstacle_detected(void)
{
    return gpio_get_level(IR_OBSTACLE_FRONT) == IR_ACTIVE_LEVEL;
}

ir_obstacle_state_t ir_obstacle_read_all(void)
{
    ir_obstacle_state_t state = {
        .front_blocked = gpio_get_level(IR_OBSTACLE_FRONT) == IR_ACTIVE_LEVEL,
        .front_up_blocked = gpio_get_level(IR_OBSTACLE_FRONT_UP) == IR_ACTIVE_LEVEL,
        .left_blocked  = gpio_get_level(IR_OBSTACLE_LEFT) == IR_ACTIVE_LEVEL,
        .right_blocked = gpio_get_level(IR_OBSTACLE_RIGHT) == IR_ACTIVE_LEVEL,
    };

    return state;
}