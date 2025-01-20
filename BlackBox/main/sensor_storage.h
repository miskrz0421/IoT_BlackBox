#ifndef SENSOR_STORAGE_H
#define SENSOR_STORAGE_H

#include <stdint.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include "esp_log.h"
#include "esp_sntp.h"
#include "esp_timer.h"

#define FLAME_STORAGE_SIZE (256 * 1024)
#define MPU6050_STORAGE_SIZE (1 * 1024 * 1024)
#define BMP280_STORAGE_SIZE (1 * 1024 * 1024)

#define MAGIC_NUMBER 0xABCD1234
#define BLOCK_SIZE 4096
#define MAX_FLASH_STORAGE_SIZE (8 * 1024 * 1024)

#define BLOCK_TYPE_MPU6050 1
#define BLOCK_TYPE_BMP280 2
#define BLOCK_TYPE_KY026 3

typedef struct
{
    uint32_t magic_number;
    uint32_t block_type;
    uint32_t record_count;
    // uint32_t timestamp;
} block_header_t;

typedef struct
{
    uint32_t timestamp;
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} mpu6050_reading_t;

typedef struct
{
    uint32_t timestamp;
    float temperature;
    float pressure;
    float normalized_pressure;
} bmp280_reading_t;

typedef struct
{
    uint32_t timestamp;
    uint8_t digital_value;
    uint16_t analog_value;
    uint16_t voltage;
} ky026_reading_t;

typedef struct
{
    mpu6050_reading_t *buffer;
    size_t capacity;
    size_t count;
    size_t write_index;
    uint32_t block_count;
} mpu6050_storage_t;

typedef struct
{
    bmp280_reading_t *buffer;
    size_t capacity;
    size_t count;
    size_t write_index;
    uint32_t block_count;
} bmp280_storage_t;

typedef struct
{
    ky026_reading_t *buffer;
    size_t capacity;
    size_t count;
    size_t write_index;
    uint32_t block_count;
} ky026_storage_t;

extern volatile uint32_t BUFFER_FLUSH_THRESHOLD_PERCENT;
extern volatile uint32_t BUFFER_FLUSH_INTERVAL_SEC;

extern mpu6050_storage_t mpu6050_storage;
extern bmp280_storage_t bmp280_storage;
extern ky026_storage_t ky026_storage;

esp_err_t init_time_once(void);
esp_err_t init_storage_time(void);
esp_err_t init_sensor_storage(void);
void free_sensor_storage(void);

esp_err_t store_mpu6050_reading(float accel_x, float accel_y, float accel_z,
                                float gyro_x, float gyro_y, float gyro_z);
esp_err_t store_bmp280_reading(float temperature, float pressure, float normalized_pressure);
esp_err_t store_ky026_reading(uint8_t digital_value, uint16_t analog_value, uint16_t voltage);

const mpu6050_reading_t *get_mpu6050_readings(size_t *count);
const bmp280_reading_t *get_bmp280_readings(size_t *count);
const ky026_reading_t *get_ky026_readings(size_t *count);
esp_err_t clear_sensor_storage(void);

#endif