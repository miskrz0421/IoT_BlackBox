#include "mpu6050_config.h"
#include "cJSON.h"
#include "esp_log.h"
#include "driver/i2c.h"

static const char *TAG = "BlackBox Config";

esp_err_t blackbox_configure_mpu6050(mpu6050_handle_t sensor, const blackbox_config_t *config)
{
    if (sensor == NULL || config == NULL)
    {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;

    ret = mpu6050_wake_up(sensor);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to wake up sensor");
        return ret;
    }

    mpu6050_acce_fs_t acce_fs = config->acce_range;
    mpu6050_gyro_fs_t gyro_fs = config->gyro_range;
    ret = mpu6050_config(sensor, acce_fs, gyro_fs);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure sensor ranges");
        return ret;
    }

    uint8_t data;
    uint8_t dlpf_cfg = config->dlpf_threshold;

    ret = mpu6050_read(sensor, MPU6050_CONFIG, &data, 1);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read config register");
        return ret;
    }

    data = (data & 0xF8) | (dlpf_cfg & 0x07);
    ret = mpu6050_write(sensor, MPU6050_CONFIG, &data, 1);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure DLPF");
        return ret;
    }

    ESP_LOGI(TAG, "MPU6050 configured: ACCE=%d, GYRO=%d, DLPF=%d",
             acce_fs, gyro_fs, dlpf_cfg);
    return ESP_OK;
}