#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include "esp_err.h"

typedef enum {
    BATTERY_OK = 0,
    BATTERY_LOW,
    BATTERY_CRITICAL
} battery_state_t;

typedef struct {
    float voltage;
    float percent;
    battery_state_t state;
} battery_info_t;

esp_err_t battery_monitor_init(void);

float battery_monitor_get_voltage(void);

float battery_monitor_get_percent(float batt_v);

battery_state_t battery_monitor_get_state(float batt_v);

battery_info_t battery_monitor_read(void);

#endif