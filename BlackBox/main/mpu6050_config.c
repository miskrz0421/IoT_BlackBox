#include "mpu6050_config.h"
#include "cJSON.h"
#include "esp_log.h"
#include "driver/i2c.h"

static const char *TAG = "BlackBox Config";

// Funkcje pomocnicze do konwersji wartości
static mpu6050_acce_fs_t get_acce_fs(cargo_type_t cargo_type)
{
    switch (cargo_type)
    {
    case CARGO_VERY_DELICATE:
        return ACCE_FS_2G;
    case CARGO_STANDARD:
        return ACCE_FS_4G;
    case CARGO_DURABLE:
        return ACCE_FS_8G;
    case CARGO_VERY_DURABLE:
        return ACCE_FS_16G;
    default:
        return ACCE_FS_4G;
    }
}

static mpu6050_gyro_fs_t get_gyro_fs(position_requirements_t position)
{
    switch (position)
    {
    case POSITION_STRICT:
        return GYRO_FS_250DPS;
    case POSITION_PREFERRED:
        return GYRO_FS_500DPS;
    case POSITION_NO_THROWING:
        return GYRO_FS_1000DPS;
    case POSITION_ANY:
        return GYRO_FS_2000DPS;
    default:
        return GYRO_FS_500DPS;
    }
}

static uint8_t get_dlpf_cfg(transport_type_t transport, transport_duration_t duration)
{
    // Bazowa wartość filtra zależna od transportu
    uint8_t base_dlpf;
    switch (transport)
    {
    case TRANSPORT_AIR:
        base_dlpf = 3; // 44Hz
        break;
    case TRANSPORT_TRUCK_TRAIN:
        base_dlpf = 2; // 94Hz
        break;
    case TRANSPORT_COURIER:
        base_dlpf = 1; // 184Hz
        break;
    default:
        base_dlpf = 2;
        break;
    }

    // Dostosowanie filtra do czasu transportu
    switch (duration)
    {
    case DURATION_SHORT:
        return (base_dlpf > 0) ? base_dlpf - 1 : 0; // Mniejsza filtracja
    case DURATION_MEDIUM:
        return base_dlpf; // Bazowa filtracja
    case DURATION_LONG:
        return (base_dlpf + 1 > 6) ? 6 : base_dlpf + 1; // Większa filtracja
    default:
        return base_dlpf;
    }
}

esp_err_t blackbox_parse_config_json(const char *json_string, blackbox_config_t *config)
{
    if (json_string == NULL || config == NULL)
    {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *root = cJSON_Parse(json_string);
    if (root == NULL)
    {
        ESP_LOGE(TAG, "JSON parsing failed");
        return ESP_FAIL;
    }

    // Parsowanie pól
    cJSON *cargo = cJSON_GetObjectItem(root, "cargo_type");
    cJSON *position = cJSON_GetObjectItem(root, "position");
    cJSON *transport = cJSON_GetObjectItem(root, "transport");
    cJSON *duration = cJSON_GetObjectItem(root, "duration");

    // Sprawdzenie czy wszystkie pola istnieją i są liczbami
    if (!cJSON_IsNumber(cargo) || !cJSON_IsNumber(position) ||
        !cJSON_IsNumber(transport) || !cJSON_IsNumber(duration))
    {
        cJSON_Delete(root);
        ESP_LOGE(TAG, "Missing or invalid fields in JSON");
        return ESP_ERR_INVALID_ARG;
    }

    // Sprawdzenie zakresów wartości
    if (cargo->valueint < 0 || cargo->valueint > 3 ||
        position->valueint < 0 || position->valueint > 3 ||
        transport->valueint < 0 || transport->valueint > 2 ||
        duration->valueint < 0 || duration->valueint > 2)
    {
        cJSON_Delete(root);
        ESP_LOGE(TAG, "Values out of range");
        return ESP_ERR_INVALID_ARG;
    }

    // Zapisanie wartości do struktury
    config->cargo_type = (cargo_type_t)cargo->valueint;
    config->position = (position_requirements_t)position->valueint;
    config->transport = (transport_type_t)transport->valueint;
    config->duration = (transport_duration_t)duration->valueint;

    cJSON_Delete(root);
    ESP_LOGI(TAG, "Configuration parsed successfully");
    return ESP_OK;
}

esp_err_t blackbox_configure_mpu6050(mpu6050_handle_t sensor, const blackbox_config_t *config)
{
    if (sensor == NULL || config == NULL)
    {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;

    // 1. Wake up MPU6050
    ret = mpu6050_wake_up(sensor);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to wake up sensor");
        return ret;
    }

    // 2. Konfiguracja zakresów pomiarowych
    mpu6050_acce_fs_t acce_fs = get_acce_fs(config->cargo_type);
    mpu6050_gyro_fs_t gyro_fs = get_gyro_fs(config->position);
    ret = mpu6050_config(sensor, acce_fs, gyro_fs);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure sensor ranges");
        return ret;
    }

    // 3. Konfiguracja podstawowa czujnika
    uint8_t data[2];
    uint8_t dlpf_cfg = get_dlpf_cfg(config->transport, config->duration);

    // Read current config to modify only DLPF bits
    ret = mpu6050_read(sensor, MPU6050_CONFIG, &data[0], 1);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read config register");
        return ret;
    }

    // Modify only DLPF bits (0-2) and keep rest unchanged
    data[0] = (data[0] & 0xF8) | (dlpf_cfg & 0x07);
    ret = mpu6050_write(sensor, MPU6050_CONFIG, &data[0], 1);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure DLPF");
        return ret;
    }

    // Set sample rate divider
    data[0] = 0; // Sample rate divider = 0 for maximum frequency
    ret = mpu6050_write(sensor, MPU6050_SMPLRT_DIV, &data[0], 1);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set sample rate");
        return ret;
    }

    ESP_LOGI(TAG, "MPU6050 configured: ACCE=%d, GYRO=%d, DLPF=%d",
             acce_fs, gyro_fs, dlpf_cfg);
    return ESP_OK;
}