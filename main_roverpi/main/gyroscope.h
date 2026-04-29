#ifndef GYROSCOPE_H
#define GYROSCOPE_H

#include <stdint.h>
#include "esp_err.h"

esp_err_t i2c_master_init(void);
esp_err_t mpu6050_init(void);

esp_err_t mpu6050_read_gyro(
    int16_t *gyro_x,
    int16_t *gyro_y,
    int16_t *gyro_z
);

esp_err_t mpu6050_calibrate_gyro(void);

void mpu6050_update_yaw(void);

float mpu6050_get_yaw_deg(void);
float mpu6050_get_gz_dps(void);

#endif