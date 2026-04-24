#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "hardware_config.h"

static const char *TAG = "ROVERPI";
struct task
{
    uint32_t period;      // Task’s period
    uint32_t elapsedTime; // Time elapsed since last task tick
    int (*initFct)(int);  // Task’s init function
    void (*tickFct)(int); // Task’s tick function
};
struct task tasks[N]

    /* ---------------- Main application ---------------- */
    void
    app_main(void)
{
    ESP_LOGI(TAG, "Starting initialization...");

    for (i = 0; i < N; i++)
    {
        tasks[i].initFcn();
    }
    timer_setPeriod(timerPeriod);
    timer_start();
    while (1)
    {
        for (i = 0; i < N; i++)
        {
            if (tasks[i].elapsedTime >= tasks[i].period)
            {
                tasks[i].tickFct();
                tasks[i].elapsedTime = 0;
            }
            tasks[i].elapsedTime += timerPeriod;
        }
        while (!TimerFlag)
        {
        }
        TimerFlag = 0;
    }
}
