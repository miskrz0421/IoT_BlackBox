#include "sensor_storage.h"
#include <string.h>
#include "esp_system.h"
#include "esp_log.h"
#include <time.h>

static const char *TAG = "SENSOR_STORAGE";

static uint32_t base_timestamp = 0;
static uint64_t start_time_us = 0;
static bool is_sntp_initialized = false;

static uint32_t get_current_timestamp(void)
{
    uint64_t elapsed_time_us = esp_timer_get_time() - start_time_us;
    uint32_t elapsed_seconds = (uint32_t)(elapsed_time_us / 1000000);
    return base_timestamp + elapsed_seconds;
}

esp_err_t init_time_once(void)
{
    if (is_sntp_initialized)
    {
        ESP_LOGI(TAG, "SNTP already initialized");
        return ESP_OK;
    }

    setenv("TZ", "CET-1", 1);
    tzset();

    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();

    is_sntp_initialized = true;
    ESP_LOGI(TAG, "SNTP server initialized");
    return ESP_OK;
}

esp_err_t init_storage_time(void)
{
    if (!is_sntp_initialized)
    {
        ESP_LOGE(TAG, "SNTP not initialized. Call init_time_once first");
        return ESP_ERR_INVALID_STATE;
    }

    esp_sntp_restart();

    int retry = 0;
    const int max_retry = 30;

    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < max_retry)
    {
        ESP_LOGI(TAG, "Waiting for SNTP sync... (%d/%d)", retry, max_retry);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    if (retry >= max_retry)
    {
        ESP_LOGE(TAG, "SNTP sync failed");
        return ESP_FAIL;
    }

    time_t now;
    time(&now);
    base_timestamp = (uint32_t)now;

    start_time_us = esp_timer_get_time();

    ESP_LOGI(TAG, "Time synchronized. Base timestamp: %lu", base_timestamp);
    return ESP_OK;
}

mpu6050_storage_t mpu6050_storage = {0};
bmp280_storage_t bmp280_storage = {0};
ky026_storage_t ky026_storage = {0};

esp_err_t init_sensor_storage(void)
{
    size_t mpu6050_count = MPU6050_STORAGE_SIZE / sizeof(mpu6050_reading_t);
    size_t bmp280_count = BMP280_STORAGE_SIZE / sizeof(bmp280_reading_t);
    size_t ky026_count = FLAME_STORAGE_SIZE / sizeof(ky026_reading_t);

    mpu6050_storage.buffer = heap_caps_malloc(MPU6050_STORAGE_SIZE, MALLOC_CAP_SPIRAM);
    if (mpu6050_storage.buffer == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate PSRAM for MPU6050");
        return ESP_ERR_NO_MEM;
    }
    mpu6050_storage.capacity = mpu6050_count;
    mpu6050_storage.count = 0;
    mpu6050_storage.write_index = 0;

    bmp280_storage.buffer = heap_caps_malloc(BMP280_STORAGE_SIZE, MALLOC_CAP_SPIRAM);
    if (bmp280_storage.buffer == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate PSRAM for BMP280");
        free_sensor_storage();
        return ESP_ERR_NO_MEM;
    }
    bmp280_storage.capacity = bmp280_count;
    bmp280_storage.count = 0;
    bmp280_storage.write_index = 0;

    ky026_storage.buffer = heap_caps_malloc(FLAME_STORAGE_SIZE, MALLOC_CAP_SPIRAM);
    if (ky026_storage.buffer == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate PSRAM for KY026");
        free_sensor_storage();
        return ESP_ERR_NO_MEM;
    }
    ky026_storage.capacity = ky026_count;
    ky026_storage.count = 0;
    ky026_storage.write_index = 0;

    ESP_LOGI(TAG, "Sensor storage initialized - MPU6050: %zu readings, BMP280: %zu readings, KY026: %zu readings",
             mpu6050_count, bmp280_count, ky026_count);

    return ESP_OK;
}

void free_sensor_storage(void)
{
    if (mpu6050_storage.buffer)
    {
        free(mpu6050_storage.buffer);
        mpu6050_storage.buffer = NULL;
    }
    if (bmp280_storage.buffer)
    {
        free(bmp280_storage.buffer);
        bmp280_storage.buffer = NULL;
    }
    if (ky026_storage.buffer)
    {
        free(ky026_storage.buffer);
        ky026_storage.buffer = NULL;
    }
}

esp_err_t store_mpu6050_reading(float accel_x, float accel_y, float accel_z,
                                float gyro_x, float gyro_y, float gyro_z)
{
    if (!mpu6050_storage.buffer)
    {
        return ESP_ERR_INVALID_STATE;
    }

    mpu6050_reading_t reading = {
        .timestamp = get_current_timestamp(),
        .accel_x = accel_x,
        .accel_y = accel_y,
        .accel_z = accel_z,
        .gyro_x = gyro_x,
        .gyro_y = gyro_y,
        .gyro_z = gyro_z};

    mpu6050_storage.buffer[mpu6050_storage.write_index] = reading;
    mpu6050_storage.write_index = (mpu6050_storage.write_index + 1) % mpu6050_storage.capacity;

    if (mpu6050_storage.count < mpu6050_storage.capacity)
    {
        mpu6050_storage.count++;
    }

    return ESP_OK;
}

esp_err_t store_bmp280_reading(float temperature, float pressure, float normalized_pressure)
{
    if (!bmp280_storage.buffer)
    {
        return ESP_ERR_INVALID_STATE;
    }

    bmp280_reading_t reading = {
        .timestamp = get_current_timestamp(),
        .temperature = temperature,
        .pressure = pressure,
        .normalized_pressure = normalized_pressure};

    bmp280_storage.buffer[bmp280_storage.write_index] = reading;
    bmp280_storage.write_index = (bmp280_storage.write_index + 1) % bmp280_storage.capacity;

    if (bmp280_storage.count < bmp280_storage.capacity)
    {
        bmp280_storage.count++;
    }

    return ESP_OK;
}

esp_err_t store_ky026_reading(uint8_t digital_value, uint16_t analog_value, uint16_t voltage)
{
    if (!ky026_storage.buffer)
    {
        return ESP_ERR_INVALID_STATE;
    }

    ky026_reading_t reading = {
        .timestamp = get_current_timestamp(),
        .digital_value = digital_value,
        .analog_value = analog_value,
        .voltage = voltage};

    ky026_storage.buffer[ky026_storage.write_index] = reading;
    ky026_storage.write_index = (ky026_storage.write_index + 1) % ky026_storage.capacity;

    if (ky026_storage.count < ky026_storage.capacity)
    {
        ky026_storage.count++;
    }

    return ESP_OK;
}

const mpu6050_reading_t *get_mpu6050_readings(size_t *count)
{
    if (count)
    {
        *count = mpu6050_storage.count;
    }
    return mpu6050_storage.buffer;
}

const bmp280_reading_t *get_bmp280_readings(size_t *count)
{
    if (count)
    {
        *count = bmp280_storage.count;
    }
    return bmp280_storage.buffer;
}

const ky026_reading_t *get_ky026_readings(size_t *count)
{
    if (count)
    {
        *count = ky026_storage.count;
    }
    return ky026_storage.buffer;
}