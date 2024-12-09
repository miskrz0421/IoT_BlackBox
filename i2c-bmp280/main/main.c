#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_SCL_IO 9
#define I2C_MASTER_SDA_IO 8
#define I2C_MASTER_FREQ_HZ 400000
#define I2C_MASTER_NUM 0
#define BMP280_SENSOR_ADDR 0x76
#define BUTTON_GPIO 0

#define BMP280_REG_TEMP_XLSB 0xFC
#define BMP280_REG_TEMP_LSB 0xFB
#define BMP280_REG_TEMP_MSB 0xFA
#define BMP280_REG_PRESS_XLSB 0xF9
#define BMP280_REG_PRESS_LSB 0xF8
#define BMP280_REG_PRESS_MSB 0xF7
#define BMP280_REG_CONFIG 0xF5
#define BMP280_REG_CTRL_MEAS 0xF4
#define BMP280_REG_STATUS 0xF3
#define BMP280_REG_RESET 0xE0
#define BMP280_REG_ID 0xD0

#define MODE_NORMAL 0
#define MODE_FORCED 1
#define MODE_SLEEP 2

static const char *TAG = "BMP280";
static uint8_t current_mode = MODE_NORMAL;
static bool sensor_connected = false;
static const int RECONNECT_DELAY_MS = 5000;
static int first_connect = 0;

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

static void button_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&io_conf);
}

static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK)
        return err;

    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static esp_err_t bmp280_read_reg(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, BMP280_SENSOR_ADDR, &reg, 1, data, len, pdMS_TO_TICKS(100));
}

static esp_err_t bmp280_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t write_buf[2] = {reg, data};
    return i2c_master_write_to_device(I2C_MASTER_NUM, BMP280_SENSOR_ADDR, write_buf, sizeof(write_buf), pdMS_TO_TICKS(100));
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

static esp_err_t bmp280_set_mode(uint8_t mode)
{
    uint8_t current_config;
    esp_err_t err = bmp280_read_reg(BMP280_REG_CTRL_MEAS, &current_config, 1);
    if (err != ESP_OK)
        return err;

    current_config &= ~0b11;

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
    }

    return bmp280_write_reg(BMP280_REG_CTRL_MEAS, current_config);
}

static esp_err_t bmp280_check_connection(void)
{
    uint8_t id;
    esp_err_t err = bmp280_read_reg(BMP280_REG_ID, &id, 1);
    if (err != ESP_OK || id != 0x58)
        return ESP_ERR_INVALID_RESPONSE;
    return ESP_OK;
}

static esp_err_t bmp280_init(void)
{
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

    err = bmp280_write_reg(BMP280_REG_CTRL_MEAS, 0b01010111);
    if (err != ESP_OK)
        return err;

    return bmp280_write_reg(BMP280_REG_CONFIG, 0b00100000);
}

static esp_err_t bmp280_reconnect(void)
{
    ESP_LOGI(TAG, "Attempting to reconnect to BMP280...");

    esp_err_t err = bmp280_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Reconnection failed");
        return err;
    }

    err = bmp280_set_mode(current_mode);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set mode after reconnection");
        return err;
    }

    ESP_LOGI(TAG, "BMP280 reconnected successfully");
    sensor_connected = true;
    return ESP_OK;
}

static esp_err_t bmp280_read_measurements(float *temperature, float *pressure)
{
    uint8_t data[6];
    esp_err_t err = bmp280_read_reg(BMP280_REG_PRESS_MSB, data, 6);
    if (err != ESP_OK)
        return err;

    int32_t press_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    int32_t temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);

    int32_t var1_t, var2_t, t_fine;
    int64_t var1_p, var2_p, p;

    var1_t = ((((temp_raw >> 3) - ((int32_t)calib_params.dig_t1 << 1))) * ((int32_t)calib_params.dig_t2)) >> 11;
    var2_t = (((((temp_raw >> 4) - ((int32_t)calib_params.dig_t1)) * ((temp_raw >> 4) - ((int32_t)calib_params.dig_t1))) >> 12) * ((int32_t)calib_params.dig_t3)) >> 14;
    t_fine = var1_t + var2_t;
    *temperature = ((t_fine * 5 + 128) >> 8) / 100.0f;

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

static void button_task(void *pvParameters)
{
    uint8_t last_state = 1;

    while (1)
    {
        uint8_t current_state = gpio_get_level(BUTTON_GPIO);

        if (last_state == 1 && current_state == 0)
        {
            current_mode = (current_mode + 1) % 3;
            esp_err_t err = bmp280_set_mode(current_mode);
            if (err == ESP_OK)
            {
                const char *mode_str[] = {"Normal", "Forced", "Sleep"};
                ESP_LOGI(TAG, "Switched to %s mode", mode_str[current_mode]);
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        last_state = current_state;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    button_init();
    ESP_LOGI(TAG, "I2C initialized successfully");

    if (bmp280_init() == ESP_OK && bmp280_set_mode(MODE_NORMAL) == ESP_OK)
    {
        ESP_LOGI(TAG, "BMP280 initialized successfully in Normal mode");
        sensor_connected = true;
    }

    xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);

    while (1)
    {
        if (!sensor_connected || bmp280_check_connection() != ESP_OK)
        {
            if (sensor_connected)
            {
                ESP_LOGE(TAG, "Lost connection to BMP280");
                first_connect = 0;
                sensor_connected = false;
            }

            vTaskDelay(pdMS_TO_TICKS(RECONNECT_DELAY_MS));
            if (bmp280_reconnect() != ESP_OK)
                continue;
        }

        switch (current_mode)
        {
        case MODE_NORMAL:
            float temperature, pressure;
            if (bmp280_read_measurements(&temperature, &pressure) == ESP_OK)
            {
                if (first_connect != 0)
                {
                    ESP_LOGI(TAG, "Temperature: %.2f °C, Pressure: %.2f Pa", temperature, pressure);
                }
                else
                {
                    first_connect++;
                }
            }
            else
            {
                sensor_connected = false;
            }
            break;

        case MODE_FORCED:
            float temp_forced, press_forced;
            if (bmp280_read_measurements(&temp_forced, &press_forced) == ESP_OK)
            {
                if (first_connect != 0)
                {
                    ESP_LOGI(TAG, "[Forced] Temperature: %.2f °C, Pressure: %.2f Pa", temp_forced, press_forced);
                }
                else
                {
                    first_connect++;
                }
            }
            else
            {
                first_connect = 0;
                sensor_connected = false;
                break;
            }
            current_mode = MODE_SLEEP;
            bmp280_set_mode(MODE_SLEEP);
            ESP_LOGI(TAG, "Switching to Sleep mode after Forced measurement");
            break;

        case MODE_SLEEP:
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}