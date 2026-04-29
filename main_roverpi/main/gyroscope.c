/*========================= GYROSCOPE DRIVER ============================
 * Created By: Michael Kane
 * Date Created: 04/29/2026
 * Description:
 *   MPU6050 driver for ESP32 using ESP-IDF I2C.
 *   Includes:
 *     - I2C init
 *     - MPU6050 wake/config
 *     - raw gyro read
 *     - startup bias calibration
 *     - yaw integration from Z gyro
 *======================================================================*/

#include "gyroscope.h"

#include <stdint.h>

#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* I2C config */
#define I2C_MASTER_SCL_IO       22
#define I2C_MASTER_SDA_IO       19
#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_FREQ_HZ      400000

/* MPU6050 config */
#define MPU6050_ADDR            0x68
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_GYRO_XOUT_H     0x43

/* Gyro conversion */
#define GYRO_SCALE_250DPS       131.0f
#define CALIBRATION_SAMPLES     300

static const char *GYRO_TAG = "GYRO";

static float gyro_bias_x = 0.0f;
static float gyro_bias_y = 0.0f;
static float gyro_bias_z = 0.0f;

static float yaw_deg = 0.0f;
static float gz_dps_latest = 0.0f;

static TickType_t last_update_tick = 0;

/*====================================================================
 * I2C INIT
 *====================================================================*/
esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        return ret;
    }

    return i2c_driver_install(
        I2C_MASTER_NUM,
        conf.mode,
        0,
        0,
        0
    );
}

/*====================================================================
 * MPU6050 INIT
 *====================================================================*/
esp_err_t mpu6050_init(void)
{
    esp_err_t ret;

    /* Wake MPU6050: write 0x00 to PWR_MGMT_1 register */
    uint8_t wake_cmd[2] = {
        MPU6050_PWR_MGMT_1,
        0x00
    };

    ret = i2c_master_write_to_device(
        I2C_MASTER_NUM,
        MPU6050_ADDR,
        wake_cmd,
        sizeof(wake_cmd),
        pdMS_TO_TICKS(1000)
    );

    if (ret != ESP_OK) {
        return ret;
    }

    /* Set gyro range to ±250 deg/s */
    uint8_t gyro_cfg[2] = {
        MPU6050_GYRO_CONFIG,
        0x00
    };

    return i2c_master_write_to_device(
        I2C_MASTER_NUM,
        MPU6050_ADDR,
        gyro_cfg,
        sizeof(gyro_cfg),
        pdMS_TO_TICKS(1000)
    );
}

/*====================================================================
 * RAW GYRO READ
 *====================================================================*/
esp_err_t mpu6050_read_gyro(
    int16_t *gyro_x,
    int16_t *gyro_y,
    int16_t *gyro_z
)
{
    uint8_t reg = MPU6050_GYRO_XOUT_H;
    uint8_t data[6];

    esp_err_t ret = i2c_master_write_read_device(
        I2C_MASTER_NUM,
        MPU6050_ADDR,
        &reg,
        1,
        data,
        6,
        pdMS_TO_TICKS(1000)
    );

    if (ret == ESP_OK) {
        *gyro_x = (int16_t)((data[0] << 8) | data[1]);
        *gyro_y = (int16_t)((data[2] << 8) | data[3]);
        *gyro_z = (int16_t)((data[4] << 8) | data[5]);
    }

    return ret;
}

/*====================================================================
 * GYRO CALIBRATION
 *====================================================================*/
esp_err_t mpu6050_calibrate_gyro(void)
{
    int64_t sum_x = 0;
    int64_t sum_y = 0;
    int64_t sum_z = 0;

    ESP_LOGI(GYRO_TAG, "Calibrating gyro — keep rover still");

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        int16_t gx = 0;
        int16_t gy = 0;
        int16_t gz = 0;

        esp_err_t ret = mpu6050_read_gyro(&gx, &gy, &gz);
        if (ret != ESP_OK) {
            ESP_LOGE(
                GYRO_TAG,
                "Calibration read failed: %s",
                esp_err_to_name(ret)
            );
            return ret;
        }

        sum_x += gx;
        sum_y += gy;
        sum_z += gz;

        vTaskDelay(pdMS_TO_TICKS(5));
    }

    gyro_bias_x = (float)sum_x / CALIBRATION_SAMPLES;
    gyro_bias_y = (float)sum_y / CALIBRATION_SAMPLES;
    gyro_bias_z = (float)sum_z / CALIBRATION_SAMPLES;

    yaw_deg = 0.0f;
    gz_dps_latest = 0.0f;
    last_update_tick = xTaskGetTickCount();

    ESP_LOGI(
        GYRO_TAG,
        "Gyro bias calibrated: X=%.2f Y=%.2f Z=%.2f",
        gyro_bias_x,
        gyro_bias_y,
        gyro_bias_z
    );

    return ESP_OK;
}

/*====================================================================
 * YAW UPDATE
 *====================================================================*/
void mpu6050_update_yaw(void)
{
    int16_t gx = 0;
    int16_t gy = 0;
    int16_t gz = 0;

    esp_err_t ret = mpu6050_read_gyro(&gx, &gy, &gz);
    if (ret != ESP_OK) {
        ESP_LOGW(
            GYRO_TAG,
            "Gyro read failed: %s",
            esp_err_to_name(ret)
        );
        return;
    }

    TickType_t now = xTaskGetTickCount();

    if (last_update_tick == 0) {
        last_update_tick = now;
        return;
    }

    float dt = ((float)(now - last_update_tick) * portTICK_PERIOD_MS) / 1000.0f;
    last_update_tick = now;

    float corrected_gz = (float)gz - gyro_bias_z;

    gz_dps_latest = corrected_gz / GYRO_SCALE_250DPS;

    if (gz_dps_latest > -0.3f && gz_dps_latest < 0.3f) {

        gz_dps_latest = 0.0f;

    }

    yaw_deg += gz_dps_latest * dt;

    if (yaw_deg >= 360.0f) {
        yaw_deg -= 360.0f;
    } else if (yaw_deg < 0.0f) {
        yaw_deg += 360.0f;
    }
}

/*====================================================================
 * GETTERS
 *====================================================================*/
float mpu6050_get_yaw_deg(void)
{
    return yaw_deg;
}

float mpu6050_get_gz_dps(void)
{
    return gz_dps_latest;
}