// mpu6050_config.h

#ifndef MPU6050_CONFIG_H
#define MPU6050_CONFIG_H

#include "mpu6050.h"
#include "esp_err.h"

#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_CONFIG 0x1A
#define MPU6050_PWR_MGMT_1 0x6B

typedef enum
{
    DLPF_PASSES_UP_TO_260HZ = 0, // Accelerometer: 260Hz, Gyroscope: 256Hz
    DLPF_PASSES_UP_TO_184HZ = 1, // Accelerometer: 184Hz, Gyroscope: 188Hz
    DLPF_PASSES_UP_TO_94HZ = 2,  // Accelerometer: 94Hz, Gyroscope: 98Hz
    DLPF_PASSES_UP_TO_44HZ = 3,  // Accelerometer: 44Hz, Gyroscope: 42Hz
    DLPF_PASSES_UP_TO_21HZ = 4,  // Accelerometer: 21Hz, Gyroscope: 20H
    DLPF_PASSES_UP_TO_10HZ = 5,  // Accelerometer: 10Hz, Gyroscope: 10Hz
    DLPF_PASSES_UP_TO_5HZ = 6    // Accelerometer: 5Hz, Gyroscope: 5Hz
} dlpf_config_t;

typedef struct
{
    mpu6050_gyro_fs_t gyro_range;
    mpu6050_acce_fs_t acce_range;
    dlpf_config_t dlpf_threshold;
} blackbox_config_t;

esp_err_t blackbox_parse_config_json(const char *json_string, blackbox_config_t *config);

esp_err_t blackbox_configure_mpu6050(mpu6050_handle_t sensor, const blackbox_config_t *config);

#endif