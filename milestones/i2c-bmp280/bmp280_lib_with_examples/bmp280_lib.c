#include "bmp280_lib.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_mac.h"
#include "esp_system.h"
#include <math.h>

static const char *TAG = "BMP280";
static i2c_port_t current_i2c_port;
static uint8_t current_i2c_addr;
static uint8_t current_mode = MODE_NORMAL;
static uint8_t current_ctrl_meas = 0b01010111;
static uint8_t current_config = 0b00110000;

typedef struct
{
    uint16_t dig_t1;
    int16_t dig_t2;
    int16_t dig_t3;
    uint16_t dig_p1;
    int16_t dig_p2;
    int16_t dig_p3;
    int16_t dig_p4;
    int16_t dig_p5;
    int16_t dig_p6;
    int16_t dig_p7;
    int16_t dig_p8;
    int16_t dig_p9;
} bmp280_calib_t;

static bmp280_calib_t calib_params;

static esp_err_t bmp280_read_reg(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(current_i2c_port, current_i2c_addr,
                                        &reg, 1, data, len, pdMS_TO_TICKS(100));
}

static esp_err_t bmp280_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t write_buf[2] = {reg, data};
    return i2c_master_write_to_device(current_i2c_port, current_i2c_addr,
                                      write_buf, sizeof(write_buf), pdMS_TO_TICKS(100));
}

static esp_err_t bmp280_read_calibration(void)
{
    uint8_t data[24];
    esp_err_t err = bmp280_read_reg(0x88, data, 24);
    if (err != ESP_OK)
        return err;

    calib_params.dig_t1 = (data[1] << 8) | data[0];
    calib_params.dig_t2 = (data[3] << 8) | data[2];
    calib_params.dig_t3 = (data[5] << 8) | data[4];
    calib_params.dig_p1 = (data[7] << 8) | data[6];
    calib_params.dig_p2 = (data[9] << 8) | data[8];
    calib_params.dig_p3 = (data[11] << 8) | data[10];
    calib_params.dig_p4 = (data[13] << 8) | data[12];
    calib_params.dig_p5 = (data[15] << 8) | data[14];
    calib_params.dig_p6 = (data[17] << 8) | data[16];
    calib_params.dig_p7 = (data[19] << 8) | data[18];
    calib_params.dig_p8 = (data[21] << 8) | data[20];
    calib_params.dig_p9 = (data[23] << 8) | data[22];

    return ESP_OK;
}

esp_err_t bmp280_set_mode(uint8_t mode)
{
    uint8_t current_config;
    esp_err_t err = bmp280_read_reg(BMP280_REG_CTRL_MEAS, &current_config, 1);
    if (err != ESP_OK)
        return err;

    current_config &= ~0b11;
    current_mode = mode;

    switch (mode)
    {
    case MODE_SLEEP:
        current_config |= 0b00;
        break;
    case MODE_FORCED:
        current_config |= 0b01;
        break;
    case MODE_NORMAL:
        current_config |= 0b11;
        break;
    default:
        return ESP_ERR_INVALID_ARG;
    }

    return bmp280_write_reg(BMP280_REG_CTRL_MEAS, current_config);
}

esp_err_t bmp280_check_connection(void)
{
    uint8_t id;
    esp_err_t err = bmp280_read_reg(BMP280_REG_ID, &id, 1);
    if (err != ESP_OK || id != 0x58)
        return ESP_ERR_INVALID_RESPONSE;
    return ESP_OK;
}

esp_err_t bmp280_init(bmp280_config_t *config)
{
    if (config == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    current_i2c_port = config->i2c_port;
    current_i2c_addr = config->i2c_addr;
    current_mode = config->mode;

    uint8_t id;
    esp_err_t err = bmp280_read_reg(BMP280_REG_ID, &id, 1);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error reading ID");
        return err;
    }

    if (id != 0x58)
    {
        ESP_LOGE(TAG, "Wrong ID: %02x", id);
        return ESP_ERR_INVALID_RESPONSE;
    }

    err = bmp280_write_reg(BMP280_REG_RESET, 0xB6);
    if (err != ESP_OK)
        return err;
    vTaskDelay(pdMS_TO_TICKS(100));

    err = bmp280_read_calibration();
    if (err != ESP_OK)
        return err;

    err = bmp280_write_reg(BMP280_REG_CTRL_MEAS, current_ctrl_meas);
    if (err != ESP_OK)
        return err;

    err = bmp280_write_reg(BMP280_REG_CONFIG, current_config);
    if (err != ESP_OK)
        return err;

    return bmp280_set_mode(current_mode);
}

esp_err_t bmp280_read_measurements(float *temperature, float *pressure)
{
    uint8_t data[6];
    esp_err_t err = bmp280_read_reg(BMP280_REG_PRESS_MSB, data, 6);
    if (err != ESP_OK)
        return err;

    int32_t press_raw = ((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | ((data[2] >> 4) & 0x0F);
    int32_t temp_raw = ((uint32_t)data[3] << 12) | ((uint32_t)data[4] << 4) | ((data[5] >> 4) & 0x0F);

    int32_t var1, var2, t_fine;
    var1 = ((((temp_raw >> 3) - ((int32_t)calib_params.dig_t1 << 1))) * (int32_t)calib_params.dig_t2) >> 11;
    var2 = (((((temp_raw >> 4) - (int32_t)calib_params.dig_t1) * ((temp_raw >> 4) - (int32_t)calib_params.dig_t1)) >> 12) * (int32_t)calib_params.dig_t3) >> 14;
    t_fine = var1 + var2;

    *temperature = (t_fine * 5 + 128) >> 8;
    *temperature = *temperature / 100.0f;

    int64_t var1_p, var2_p, p;
    var1_p = ((int64_t)t_fine) - 128000;
    var2_p = var1_p * var1_p * (int64_t)calib_params.dig_p6;
    var2_p = var2_p + ((var1_p * (int64_t)calib_params.dig_p5) << 17);
    var2_p = var2_p + (((int64_t)calib_params.dig_p4) << 35);
    var1_p = ((var1_p * var1_p * (int64_t)calib_params.dig_p3) >> 8) + ((var1_p * (int64_t)calib_params.dig_p2) << 12);
    var1_p = (((((int64_t)1) << 47) + var1_p)) * ((int64_t)calib_params.dig_p1) >> 33;

    if (var1_p == 0)
        return ESP_ERR_INVALID_STATE;

    p = 1048576 - press_raw;
    p = (((p << 31) - var2_p) * 3125) / var1_p;
    var1_p = (((int64_t)calib_params.dig_p9) * (p >> 13) * (p >> 13)) >> 25;
    var2_p = (((int64_t)calib_params.dig_p8) * p) >> 19;

    p = ((p + var1_p + var2_p) >> 8) + (((int64_t)calib_params.dig_p7) << 4);
    *pressure = (float)p / 256.0f;

    return ESP_OK;
}

esp_err_t bmp280_configure_regs(bmp280_osrs_p_t osrs_p, bmp280_osrs_t_t osrs_t,
                                bmp280_filter_t filter, bmp280_t_sb_t t_sb)
{

    current_ctrl_meas = (osrs_t << 5) | (osrs_p << 2) | (current_ctrl_meas & 0b11);

    current_config = (t_sb << 5) | (filter << 2) | (current_config & 0b11);

    esp_err_t err = bmp280_write_reg(BMP280_REG_CTRL_MEAS, current_ctrl_meas);
    if (err != ESP_OK)
        return err;

    return bmp280_write_reg(BMP280_REG_CONFIG, current_config);
}

double normalize_pressure(double pressure, double altitude)
{
    double exponent = (GRAVITY * MOLAR_MASS_AIR * altitude) /
                      (GAS_CONSTANT * TEMPERATURE_STANDARD);
    return pressure * exp(exponent);
}