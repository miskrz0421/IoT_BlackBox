#include <string.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include "esp_system.h"
#include "nvs_flash.h"

#define TAG "BMP280_SPI_DRIVER"

// SPI Pin definitions
#define SPI_MOSI_PIN 11
#define SPI_MISO_PIN 13
#define SPI_SCLK_PIN 12
#define SPI_CS_PIN 10

// BMP280 Register Addresses
#define BMP280_REG_ID 0xD0
#define BMP280_REG_CTRL_MEAS 0xF4
#define BMP280_REG_CONFIG 0xF5
#define BMP280_REG_PRESS_MSB 0xF7
#define BMP280_REG_TEMP_MSB 0xFA
#define BMP280_RESET_REG 0xE0

// Operation modes
#define BMP280_SLEEP_MODE 0x00
#define BMP280_FORCED_MODE 0x01
#define BMP280_NORMAL_MODE 0x03

// Expected chip ID
#define BMP280_CHIP_ID 0x58

// Configuration structures
typedef struct
{
    uint8_t mode;
    uint8_t osrs_t;
    uint8_t osrs_p;
    uint8_t t_sb;
    uint8_t filter;
} bmp280_config_t;

typedef struct
{
    int32_t temperature;
    int32_t pressure;
} bmp280_raw_data_t;

// Global SPI handle
static spi_device_handle_t spi;

/**
 * @brief Initialize SPI bus and configure pins
 */
static bool bmp280_init(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = SPI_MOSI_PIN,
        .miso_io_num = SPI_MISO_PIN,
        .sclk_io_num = SPI_SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32};

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 250000, // Zmniejszamy prędkość do 500kHz
        .mode = 0,                // Tryb 3 (CPOL=1, CPHA=1)
        .spics_io_num = SPI_CS_PIN,
        .queue_size = 7,
        .flags = 0,
        .pre_cb = NULL,
        .cs_ena_pretrans = 3, // Dodajemy opóźnienie przed transakcją
        .cs_ena_posttrans = 3 // Dodajemy opóźnienie po transakcji
    };

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize SPI bus");
        return false;
    }

    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add SPI device");
        return false;
    }

    ESP_LOGI(TAG, "SPI initialized successfully");
    return true;
}

/**
 * @brief Read a single register from BMP280
 */
static uint8_t bmp280_read_register(uint8_t reg)
{
    // Dodajmy więcej debugowania
    ESP_LOGI(TAG, "Attempting to read register 0x%02X", reg);

    // Przygotujmy transakcję SPI z dodatkowym bitem
    static spi_transaction_t t = {
        .flags = 0,
        .cmd = 0,
        .addr = 0,
        .length = 16,
        .rxlength = 16};

    uint8_t tx_data[2] = {reg | 0x80, 0xFF}; // Zmiana 0xFF na 0x00
    uint8_t rx_data[2] = {0};

    t.tx_buffer = tx_data;
    t.rx_buffer = rx_data;

    // Dodajmy sprawdzenie przed transmisją
    ESP_LOGI(TAG, "TX buffer: [0x%02X, 0x%02X]", tx_data[0], tx_data[1]);

    vTaskDelay(pdMS_TO_TICKS(1000));

    esp_err_t ret = spi_device_transmit(spi, &t);
    vTaskDelay(pdMS_TO_TICKS(1000));
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI transmission failed: %s", esp_err_to_name(ret));
        return 0;
    }

    ESP_LOGI(TAG, "RX buffer: [0x%02X, 0x%02X]", rx_data[0], rx_data[1]);

    return rx_data[1];
}

/**
 * @brief Write to a single register on BMP280
 */
static void bmp280_write_register(uint8_t reg, uint8_t value)
{
    uint8_t tx_data[2] = {reg & 0x7F, value}; // Clear read bit (bit 7)

    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx_data,
        .flags = 0};

    esp_err_t ret = spi_device_transmit(spi, &t);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write register 0x%02X", reg);
    }
}

/**
 * @brief Verify BMP280 chip ID
 */
static bool bmp280_verify_id(void)
{
    uint8_t id = bmp280_read_register(BMP280_REG_ID);
    ESP_LOGI(TAG, "BMP280 ID: 0x%02X", id);
    if (id != BMP280_CHIP_ID)
    {
        ESP_LOGE(TAG, "Unexpected chip ID: 0x%02X", id);
        return false;
    }
    return true;
}

/**
 * @brief Reset BMP280
 */
static bool bmp280_reset(void)
{
    bmp280_write_register(BMP280_RESET_REG, 0xB6);
    vTaskDelay(pdMS_TO_TICKS(10)); // Wait for reset to complete
    ESP_LOGI(TAG, "BMP280 reset completed");
    return true;
}

/**
 * @brief Set BMP280 operation mode
 */
static bool bmp280_set_mode(uint8_t mode)
{
    uint8_t ctrl_meas = bmp280_read_register(BMP280_REG_CTRL_MEAS);
    ctrl_meas = (ctrl_meas & ~0x03) | (mode & 0x03);
    bmp280_write_register(BMP280_REG_CTRL_MEAS, ctrl_meas);

    // Verify the mode was set correctly
    uint8_t verify = bmp280_read_register(BMP280_REG_CTRL_MEAS);
    if ((verify & 0x03) != (mode & 0x03))
    {
        ESP_LOGE(TAG, "Failed to set mode: 0x%02X", mode);
        return false;
    }

    ESP_LOGI(TAG, "Mode set to: 0x%02X", mode);
    return true;
}

/**
 * @brief Read raw temperature and pressure data
 */
static bool bmp280_read_raw_data(bmp280_raw_data_t *data)
{
    if (data == NULL)
    {
        ESP_LOGE(TAG, "NULL pointer provided for raw data");
        return false;
    }

    uint8_t tx_buffer[7] = {BMP280_REG_PRESS_MSB | 0x80, 0, 0, 0, 0, 0, 0};
    uint8_t rx_buffer[7];

    spi_transaction_t t = {
        .length = 7 * 8, // 7 bytes total
        .tx_buffer = tx_buffer,
        .rx_buffer = rx_buffer};

    esp_err_t ret = spi_device_transmit(spi, &t);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read raw data");
        return false;
    }

    // Combine the bytes into pressure and temperature values
    data->pressure = ((uint32_t)rx_buffer[1] << 12) |
                     ((uint32_t)rx_buffer[2] << 4) |
                     (rx_buffer[3] >> 4);

    data->temperature = ((uint32_t)rx_buffer[4] << 12) |
                        ((uint32_t)rx_buffer[5] << 4) |
                        (rx_buffer[6] >> 4);

    ESP_LOGD(TAG, "Raw data - Temp: %ld, Press: %ld",
             data->temperature, data->pressure);
    return true;
}

/**
 * @brief Convert raw temperature to actual temperature in degrees Celsius
 */
static float bmp280_compensate_temperature(int32_t raw_temp)
{
    return raw_temp / 5120.0f;
}

void app_main(void)
{
    bmp280_raw_data_t raw_data;

    // Initialize hardware
    if (!bmp280_init())
    {
        ESP_LOGE(TAG, "Failed to initialize BMP280");
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    // Verify chip ID
    if (!bmp280_verify_id())
    {
        ESP_LOGE(TAG, "Failed to verify BMP280 ID");
        return;
    }

    // Reset and configure sensor
    if (!bmp280_reset() || !bmp280_set_mode(BMP280_NORMAL_MODE))
    {
        ESP_LOGE(TAG, "Failed to configure BMP280");
        return;
    }

    // Main loop
    while (1)
    {
        if (bmp280_read_raw_data(&raw_data))
        {
            float temperature = bmp280_compensate_temperature(raw_data.temperature);
            ESP_LOGI(TAG, "Temperature: %.2f °C", temperature);
        }
        else
        {
            ESP_LOGE(TAG, "Failed to read sensor data");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}