#ifndef SENSOR_STORAGE_H
#define SENSOR_STORAGE_H

#include <stdint.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include "esp_log.h"
// Na początku pliku, z innymi #include
#include "esp_sntp.h"
#include "esp_timer.h"

// Przed deklaracjami funkcji
esp_err_t init_time_once(void);
esp_err_t init_storage_time(void); // nowa funkcja do inicjalizacji czasu

#define FLAME_STORAGE_SIZE (512 * 1024)        // 0.5MB for flame sensor
#define MPU6050_STORAGE_SIZE (5 * 1024 * 1024) // 5MB for accelerometer
#define BMP280_STORAGE_SIZE (2 * 1024 * 1024)  // 2MB for pressure/temp

// Structure for MPU6050 readings
typedef struct
{
    uint32_t timestamp; // Unix timestamp
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} mpu6050_reading_t;

// Structure for BMP280 readings
typedef struct
{
    uint32_t timestamp;
    float temperature;
    float pressure;
    float normalized_pressure;
} bmp280_reading_t;

// Structure for KY026 (flame sensor) readings
typedef struct
{
    uint32_t timestamp;
    uint8_t digital_value;
    uint16_t analog_value;
    uint16_t voltage;
} ky026_reading_t;

// Storage management structures
typedef struct
{
    mpu6050_reading_t *buffer;
    size_t capacity;    // Maximum number of readings
    size_t count;       // Current number of readings
    size_t write_index; // Current write position
} mpu6050_storage_t;

typedef struct
{
    bmp280_reading_t *buffer;
    size_t capacity;
    size_t count;
    size_t write_index;
} bmp280_storage_t;

typedef struct
{
    ky026_reading_t *buffer;
    size_t capacity;
    size_t count;
    size_t write_index;
} ky026_storage_t;

// Global storage instances
extern mpu6050_storage_t mpu6050_storage;
extern bmp280_storage_t bmp280_storage;
extern ky026_storage_t ky026_storage;

// Function declarations
esp_err_t init_sensor_storage(void);
void free_sensor_storage(void);

// MPU6050 functions
esp_err_t store_mpu6050_reading(float accel_x, float accel_y, float accel_z,
                                float gyro_x, float gyro_y, float gyro_z);
const mpu6050_reading_t *get_mpu6050_readings(size_t *count);

// BMP280 functions
esp_err_t store_bmp280_reading(float temperature, float pressure, float normalized_pressure);
const bmp280_reading_t *get_bmp280_readings(size_t *count);

// KY026 functions
esp_err_t store_ky026_reading(uint8_t digital_value, uint16_t analog_value, uint16_t voltage);
const ky026_reading_t *get_ky026_readings(size_t *count);

#endif // SENSOR_STORAGE_H