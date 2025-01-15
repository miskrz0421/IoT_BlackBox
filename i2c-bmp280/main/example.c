#include "bmp280_lib.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Configuration defines for I2C and sensor operation
#define I2C_MASTER_SCL_IO 9
#define I2C_MASTER_SDA_IO 8
#define I2C_MASTER_FREQ_HZ 400000
#define I2C_MASTER_NUM 0
#define ALTITUDE 220              // Current altitude for pressure normalization
#define RECONNECT_DELAY_MS 2000   // Delay between reconnection attempts
#define MEASUREMENT_DELAY_MS 2000 // Delay between measurements
#define MODE_SWITCH_INTERVAL 5    // Number of measurements before mode switch

static const char *TAG = "BMP280_APP";
static uint8_t current_mode = MODE_NORMAL;
static bool sensor_connected = false;
static int first_connect = 0;
static int measurement_count = 0;

/**
 * @brief Initializes I2C master communication
 * Configures I2C with specified pins and frequency
 */
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

void app_main(void)
{
    // Initialize I2C communication
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    // Configure BMP280 sensor with initial settings
    // Using bmp280_init() to set up the sensor with I2C parameters and normal mode
    bmp280_config_t config = {
        .i2c_port = I2C_MASTER_NUM,
        .i2c_addr = BMP280_SENSOR_ADDR,
        .mode = MODE_NORMAL};
    if (bmp280_init(&config) == ESP_OK)
    {
        ESP_LOGI(TAG, "BMP280 initialized successfully in Normal mode");
        sensor_connected = true;
    }

    while (1)
    {
        // Check sensor connection using bmp280_check_connection()
        // Attempts reconnection if connection is lost
        if (!sensor_connected || bmp280_check_connection() != ESP_OK)
        {
            if (sensor_connected)
            {
                ESP_LOGE(TAG, "Lost connection to BMP280");
                first_connect = 0;
                sensor_connected = false;
            }
            vTaskDelay(pdMS_TO_TICKS(RECONNECT_DELAY_MS));
            if (bmp280_init(&config) != ESP_OK)
                continue;
            sensor_connected = true;
        }

        // Read sensor measurements using bmp280_read_measurements()
        // Gets temperature in °C and pressure in hPa
        float temperature, pressure;
        if (bmp280_read_measurements(&temperature, &pressure) == ESP_OK)
        {
            if (first_connect != 0)
            {
                // Use normalize_pressure() to convert current pressure to sea level equivalent
                ESP_LOGI(TAG, "Temperature: %.2f °C, Air Pressure: %.2f hPa, Sea Level Pressure: %.2f hPa",
                         temperature, pressure / 100.0, normalize_pressure(pressure, ALTITUDE) / 100.0);
            }
            else
            {
                first_connect++;
            }

            // Switch measurement modes using bmp280_set_mode()
            // Alternates between Normal and Forced modes
            measurement_count++;
            if (measurement_count >= MODE_SWITCH_INTERVAL)
            {
                current_mode = (current_mode == MODE_NORMAL) ? MODE_FORCED : MODE_NORMAL;
                esp_err_t err = bmp280_set_mode(current_mode);
                if (err == ESP_OK)
                {
                    const char *mode_str[] = {"Normal", "Forced", "Sleep"};
                    ESP_LOGI(TAG, "Switched to %s mode", mode_str[current_mode]);
                }
                measurement_count = 0;
            }
        }
        else
        {
            sensor_connected = false;
        }

        vTaskDelay(pdMS_TO_TICKS(MEASUREMENT_DELAY_MS));
    }
}