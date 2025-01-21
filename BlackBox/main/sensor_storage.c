#include "sensor_storage.h"
#include <string.h>
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_partition.h"
#include <time.h>

static const char *TAG = "SENSOR_STORAGE";

static const uint32_t BUFFER_FLUSH_THRESHOLD_PERCENT = 90;

mpu6050_storage_t mpu6050_storage = {0};
bmp280_storage_t bmp280_storage = {0};
ky026_storage_t ky026_storage = {0};

static const esp_partition_t *storage_partition = NULL;

static uint32_t base_timestamp = 0;
static uint64_t start_time_us = 0;
static bool is_sntp_initialized = false;
static uint64_t last_flush_time = 0;

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

static esp_err_t write_data_block(const void *data, size_t record_size,
                                  uint32_t record_count, uint32_t block_type,
                                  uint32_t block_number)
{
    size_t offset = (block_number * 3 + (block_type - 1)) * BLOCK_SIZE;

    block_header_t header = {
        .magic_number = MAGIC_NUMBER,
        .block_type = block_type,
        .record_count = record_count,

    };

    if (offset + BLOCK_SIZE > storage_partition->size)
    {
        ESP_LOGE(TAG, "Not enough space in partition");
        return ESP_ERR_NO_MEM;
    }

    ESP_ERROR_CHECK(esp_partition_erase_range(storage_partition, offset, BLOCK_SIZE));

    ESP_ERROR_CHECK(esp_partition_write(storage_partition, offset, &header, sizeof(header)));

    ESP_ERROR_CHECK(esp_partition_write(storage_partition,
                                        offset + sizeof(header),
                                        data,
                                        record_size * record_count));
    ESP_LOGI(TAG, "Written data size: %lu",
             record_size * record_count);

    ESP_LOGI(TAG, "Written block %lu, type %lu, records: %lu",
             block_number, block_type, record_count);

    return ESP_OK;
}

static bool should_flush_buffer(size_t count, size_t capacity, uint32_t *last_flush_time_sensor)
{
    ESP_LOGI(TAG, "Checking - count: %u, capacity: %u, percent filled: %u%%, threshold: %u%%",
             (unsigned int)count,
             (unsigned int)capacity,
             (unsigned int)(count * 100 / capacity),
             (unsigned int)BUFFER_FLUSH_THRESHOLD_PERCENT);

    if (count * 100 / capacity >= BUFFER_FLUSH_THRESHOLD_PERCENT)
    {
        ESP_LOGI(TAG, "Flushing due to buffer threshold");
        return true;
    }
    return false;
}

static esp_err_t flush_buffer(void *buffer, size_t record_size,
                              size_t *count, size_t *write_index,
                              uint32_t block_type, uint32_t *block_count,
                              uint32_t *last_flush_time_sensor)
{
    if (*count > 0)
    {
        esp_err_t err = write_data_block(buffer,
                                         record_size,
                                         *count,
                                         block_type,
                                         (*block_count)++);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to flush buffer type %lu", block_type);
            return err;
        }
        *count = 0;
        *write_index = 0;
        *last_flush_time_sensor = esp_timer_get_time() / 1000000;
    }
    return ESP_OK;
}
static uint32_t last_flush_time_mpu6050 = 0;
static uint32_t last_flush_time_bmp280 = 0;
static uint32_t last_flush_time_ky026 = 0;

esp_err_t init_sensor_storage(void)
{
    storage_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, 0x99, "storage");
    if (storage_partition == NULL)
    {
        ESP_LOGE(TAG, "Failed to find storage partition");
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGW(TAG, "PARTITION SIZE: %lu", storage_partition->size);
    ESP_LOGW(TAG, "PARTITION ERASE SIZE: %lu", storage_partition->erase_size);

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
    mpu6050_storage.block_count = 0;

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
    bmp280_storage.block_count = 0;

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
    ky026_storage.block_count = 0;

    last_flush_time = esp_timer_get_time() / 1000000;

    ESP_LOGI(TAG, "Sensor storage initialized - MPU6050: %u readings, BMP280: %u readings, KY026: %u readings",
             (unsigned int)mpu6050_count, (unsigned int)bmp280_count, (unsigned int)ky026_count);

    return ESP_OK;
}
esp_err_t clear_sensor_storage(void)
{
    if (storage_partition == NULL)
    {
        ESP_LOGE(TAG, "Storage not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Erasing storage partition...");
    esp_err_t err = esp_partition_erase_range(storage_partition, 0, storage_partition->size);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to erase storage partition");
        return err;
    }

    mpu6050_storage.block_count = 0;
    bmp280_storage.block_count = 0;
    ky026_storage.block_count = 0;

    ESP_LOGI(TAG, "Storage cleared successfully");
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

    if (should_flush_buffer(mpu6050_storage.count, mpu6050_storage.capacity, &last_flush_time_mpu6050))
    {
        flush_buffer(mpu6050_storage.buffer,
                     sizeof(mpu6050_reading_t),
                     &mpu6050_storage.count,
                     &mpu6050_storage.write_index,
                     BLOCK_TYPE_MPU6050,
                     &mpu6050_storage.block_count,
                     &last_flush_time_mpu6050);
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

    if (should_flush_buffer(bmp280_storage.count, bmp280_storage.capacity, &last_flush_time_bmp280))
    {
        flush_buffer(bmp280_storage.buffer,
                     sizeof(bmp280_reading_t),
                     &bmp280_storage.count,
                     &bmp280_storage.write_index,
                     BLOCK_TYPE_BMP280,
                     &bmp280_storage.block_count,
                     &last_flush_time_bmp280);
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

    if (should_flush_buffer(ky026_storage.count, ky026_storage.capacity, &last_flush_time_ky026))
    {
        flush_buffer(ky026_storage.buffer,
                     sizeof(ky026_reading_t),
                     &ky026_storage.count,
                     &ky026_storage.write_index,
                     BLOCK_TYPE_KY026,
                     &ky026_storage.block_count,
                     &last_flush_time_ky026);
    }
    return ESP_OK;
}
static esp_err_t read_blocks_from_flash(uint32_t block_type, void *buffer,
                                        size_t record_size, uint32_t *count)
{
    block_header_t header;
    uint32_t block_count = 0;
    size_t total_count = 0;
    uint8_t *write_ptr = buffer;

    bool counting_only = (buffer == NULL);

    while (1)
    {
        size_t offset = (block_count * 3 + (block_type - 1)) * BLOCK_SIZE;

        if (offset >= storage_partition->size)
        {
            ESP_LOGI(TAG, "Reached end of partition at offset %u", (unsigned int)offset);
            break;
        }

        esp_err_t err = esp_partition_read(storage_partition, offset, &header, sizeof(header));
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to read block header at offset %u", (unsigned int)offset);
            break;
        }

        if (header.magic_number != MAGIC_NUMBER || header.block_type != block_type)
        {
            ESP_LOGD(TAG, "Invalid block at offset %zu (magic=%lu, type=%lu)",
                     (unsigned int)offset, header.magic_number, header.block_type);
            break;
        }

        if (!counting_only)
        {
            err = esp_partition_read(storage_partition,
                                     offset + sizeof(header),
                                     write_ptr,
                                     header.record_count * record_size);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to read block data at offset %u", (unsigned int)offset);
                return err;
            }
            write_ptr += header.record_count * record_size;
        }

        total_count += header.record_count;
        block_count++;

        ESP_LOGI(TAG, "Read block %lu: %lu records at offset %u",
                 block_count - 1, header.record_count, (unsigned int)offset);
    }

    *count = total_count;
    ESP_LOGI(TAG, "Total blocks read: %lu, total records: %u",
             block_count, (unsigned int)total_count);
    return ESP_OK;
}

const mpu6050_reading_t *get_mpu6050_readings(size_t *count)
{
    if (!count)
        return NULL;

    size_t flash_count = 0;
    size_t total_count;

    read_blocks_from_flash(BLOCK_TYPE_MPU6050, NULL, sizeof(mpu6050_reading_t), &flash_count);
    total_count = flash_count + mpu6050_storage.count;

    if (total_count == 0)
    {
        *count = 0;
        return NULL;
    }

    mpu6050_reading_t *all_readings = heap_caps_malloc(total_count * sizeof(mpu6050_reading_t), MALLOC_CAP_SPIRAM);
    if (!all_readings)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for combined MPU6050 readings");
        *count = 0;
        return NULL;
    }

    if (flash_count > 0)
    {
        esp_err_t err = read_blocks_from_flash(BLOCK_TYPE_MPU6050,
                                               all_readings,
                                               sizeof(mpu6050_reading_t),
                                               &flash_count);
        if (err != ESP_OK)
        {
            free(all_readings);
            *count = 0;
            return NULL;
        }
    }

    if (mpu6050_storage.count > 0)
    {
        memcpy(&all_readings[flash_count],
               mpu6050_storage.buffer,
               mpu6050_storage.count * sizeof(mpu6050_reading_t));
    }

    *count = total_count;

    ESP_LOGI(TAG, "Retrieved %u MPU6050 readings (Flash: %u, PSRAM: %u)",
             (unsigned int)total_count, (unsigned int)flash_count, (unsigned int)mpu6050_storage.count);
    if (mpu6050_storage.buffer)
    {
        free(mpu6050_storage.buffer);
        mpu6050_storage.buffer = NULL;
    }
    return all_readings;
}

const bmp280_reading_t *get_bmp280_readings(size_t *count)
{
    if (!count)
        return NULL;

    size_t flash_count = 0;
    size_t total_count;

    read_blocks_from_flash(BLOCK_TYPE_BMP280, NULL, sizeof(bmp280_reading_t), &flash_count);
    total_count = flash_count + bmp280_storage.count;

    if (total_count == 0)
    {
        *count = 0;
        return NULL;
    }

    bmp280_reading_t *all_readings = heap_caps_malloc(total_count * sizeof(bmp280_reading_t), MALLOC_CAP_SPIRAM);
    if (!all_readings)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for combined BMP280 readings");
        *count = 0;
        return NULL;
    }

    if (flash_count > 0)
    {
        esp_err_t err = read_blocks_from_flash(BLOCK_TYPE_BMP280,
                                               all_readings,
                                               sizeof(bmp280_reading_t),
                                               &flash_count);
        if (err != ESP_OK)
        {
            free(all_readings);
            *count = 0;
            return NULL;
        }
    }

    if (bmp280_storage.count > 0)
    {
        memcpy(&all_readings[flash_count],
               bmp280_storage.buffer,
               bmp280_storage.count * sizeof(bmp280_reading_t));
    }

    *count = total_count;

    ESP_LOGI(TAG, "Retrieved %u BMP280 readings (Flash: %u, PSRAM: %u)",
             (unsigned int)total_count, (unsigned int)flash_count, (unsigned int)bmp280_storage.count);
    if (bmp280_storage.buffer)
    {
        free(bmp280_storage.buffer);
        bmp280_storage.buffer = NULL;
    }
    return all_readings;
}

const ky026_reading_t *get_ky026_readings(size_t *count)
{
    if (!count)
        return NULL;

    size_t flash_count = 0;
    size_t total_count;

    read_blocks_from_flash(BLOCK_TYPE_KY026, NULL, sizeof(ky026_reading_t), &flash_count);
    total_count = flash_count + ky026_storage.count;

    if (total_count == 0)
    {
        *count = 0;
        return NULL;
    }

    ky026_reading_t *all_readings = heap_caps_malloc(total_count * sizeof(ky026_reading_t), MALLOC_CAP_SPIRAM);
    if (!all_readings)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for combined KY026 readings");
        *count = 0;
        return NULL;
    }

    if (flash_count > 0)
    {
        esp_err_t err = read_blocks_from_flash(BLOCK_TYPE_KY026,
                                               all_readings,
                                               sizeof(ky026_reading_t),
                                               &flash_count);
        if (err != ESP_OK)
        {
            free(all_readings);
            *count = 0;
            return NULL;
        }
    }

    if (ky026_storage.count > 0)
    {
        memcpy(&all_readings[flash_count],
               ky026_storage.buffer,
               ky026_storage.count * sizeof(ky026_reading_t));
    }

    *count = total_count;
    ESP_LOGI(TAG, "Retrieved %u KY026 readings (Flash: %u, PSRAM: %u)",
             (unsigned int)total_count, (unsigned int)flash_count, (unsigned int)ky026_storage.count);
    if (ky026_storage.buffer)
    {
        free(ky026_storage.buffer);
        ky026_storage.buffer = NULL;
    }
    return all_readings;
}