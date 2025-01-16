#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "nvs_flash.h"
#include "esp_system.h"

#include "mpu6050.h"
#include "mpu6050_config.h"
#include "bmp280_lib.h"
#include "ky026.h"
#include "ble.h"
#include "wifi.h"
#include "sensor_storage.h"
#include "mqtt.h"
#include "led_strip.h"
#include <cJSON.h>
#include <math.h>

#define I2C_MASTER_SCL_IO 9
#define I2C_MASTER_SDA_IO 8
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000
#define BOOT_BUTTON GPIO_NUM_0

#define MPU6050_INTERVAL 1
#define KY026_INTERVAL 1

#define LOCATION_ALTITUDE 200.0

static const char *TAG = "SENSOR_CONFIG";
static volatile bool is_recording = false;
static volatile uint32_t button_press_start = 0;

typedef enum
{
    DEVICE_STATE_NOT_CONFIGURED = -1,
    DEVICE_STATE_CONFIGURING = 0,
    DEVICE_STATE_READY_TO_TRAVEL = 1,
    DEVICE_STATE_TRAVELLING = 2,
    DEVICE_STATE_AFTER_TRAVEL = 3
} device_state_t;
static device_state_t dev_state;

static const blackbox_config_t mock_mpu_config = {
    .gyro_range = GYRO_FS_500DPS,
    .acce_range = ACCE_FS_4G,
    .dlpf_threshold = DLPF_PASSES_UP_TO_44HZ};

static esp_err_t init_i2c(void);
static void configure_boot_button(void);
static void IRAM_ATTR button_isr_handler(void *arg);
static void mpu6050_task(void *pvParameters);
static void bmp280_task(void *pvParameters);
static void ky026_task(void *pvParameters);
extern int mqtt_connected;
extern char pin_value[7];
extern bool wifi_connected;
extern bool got_pin;
extern bool got_time = false;
bool verified = false;
bool yellow_led = false;
static esp_err_t init_i2c(void)
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

esp_err_t save_device_state(device_state_t new_state)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("config", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error opening NVS handle!");
        return err;
    }

    err = nvs_set_i32(nvs_handle, "state", (int32_t)new_state);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error setting state in NVS!");
        nvs_close(nvs_handle);
        return err;
    }

    err = nvs_commit(nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error committing NVS!");
        nvs_close(nvs_handle);
        return err;
    }

    dev_state = new_state;
    ESP_LOGI(TAG, "Successfully saved new state: %ld", (long)new_state);

    nvs_close(nvs_handle);
    return ESP_OK;
}

esp_err_t check_device_state()
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("config", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error opening NVS handle!");
        return err;
    }
    int32_t state;
    err = nvs_get_i32(nvs_handle, "state", &state);

    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGI(TAG, "State not found in NVS, setting to -1");
        err = nvs_set_i32(nvs_handle, "state", -1);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Error setting initial state!");
            nvs_close(nvs_handle);
            return err;
        }
        err = nvs_commit(nvs_handle);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Error committing NVS!");
            nvs_close(nvs_handle);
            return err;
        }
        dev_state = DEVICE_STATE_NOT_CONFIGURED;
    }
    else if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "Found state in NVS: %ld", (long)state);
        dev_state = (device_state_t)state;
    }
    else
    {
        ESP_LOGE(TAG, "Error reading state from NVS!");
    }

    nvs_close(nvs_handle);
    return err;
}
#define LED_PIN 7
#define BLINK_DELAY_MS 1000
static const char *LEDTAG = "blink_led";

static void configure_ledRed(void)
{
    ESP_LOGI(LEDTAG, "Konfiguracja pinu GPIO dla diody LED");

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    gpio_config(&io_conf);
}
#define LED_GREEN 17
static void configure_ledGreen(void)
{
    ESP_LOGI(LEDTAG, "Konfiguracja pinu GPIO dla diody LED");

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GREEN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    gpio_config(&io_conf);
}
#define LED_YELLOW 16
static void configure_ledYellow(void)
{
    ESP_LOGI(LEDTAG, "Konfiguracja pinu GPIO dla diody LED");

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_YELLOW),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    gpio_config(&io_conf);
}

static void led_blink_task(void *pvParameter)
{
    bool led_state = false;

    ESP_LOGI(TAG, "Start zadania migania LED");

    while (1)
    {
        if (dev_state == -1)
        {
            led_state = !led_state;
            gpio_set_level(LED_PIN, led_state);
        }

        vTaskDelay(pdMS_TO_TICKS(BLINK_DELAY_MS));
    }
}
static void led_yellow_task(void *pvParameter)
{
    bool led_state = false;

    ESP_LOGI(TAG, "Start zadania migania LED");

    while (1)
    {

        if (yellow_led)
        {
            led_state = !led_state;
            gpio_set_level(LED_YELLOW, led_state);
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        else
        {
            led_state = 0;
            gpio_set_level(LED_YELLOW, led_state);
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(BLINK_DELAY_MS));
    }
}

static void mpu6050_task(void *pvParameters)
{
    mpu6050_handle_t mpu = mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS);
    esp_err_t err = blackbox_configure_mpu6050(mpu, &mock_mpu_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure MPU6050");
        vTaskDelete(NULL);
        return;
    }

    bool sensor_configured = false;

    float prev_acce_x = 0.0f;
    float prev_acce_y = 0.0f;
    float prev_acce_z = 0.0f;
    float prev_gyro_x = 0.0f;
    float prev_gyro_y = 0.0f;
    float prev_gyro_z = 0.0f;
    bool first_reading = true;

    while (1)
    {
        if (dev_state == DEVICE_STATE_AFTER_TRAVEL)
        {
            sensor_configured = false;
        }
        if ((dev_state == DEVICE_STATE_READY_TO_TRAVEL || dev_state == DEVICE_STATE_TRAVELLING) && !sensor_configured)
        {
            nvs_handle_t nvs_handle;
            esp_err_t err = nvs_open("config", NVS_READONLY, &nvs_handle);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Error opening NVS handle");
                return;
            }

            blackbox_config_t mpu_config;
            int32_t value;

            err = nvs_get_i32(nvs_handle, "gyro", &value);
            if (err == ESP_OK)
            {
                mpu_config.gyro_range = (mpu6050_gyro_fs_t)value;
            }
            else
            {
                ESP_LOGE(TAG, "Error reading cargo_type from NVS");
                nvs_close(nvs_handle);
                return;
            }

            err = nvs_get_i32(nvs_handle, "acce", &value);
            if (err == ESP_OK)
            {
                mpu_config.acce_range = (mpu6050_acce_fs_t)value;
            }
            else
            {
                ESP_LOGE(TAG, "Error reading position_req from NVS");
                nvs_close(nvs_handle);
                return;
            }

            err = nvs_get_i32(nvs_handle, "dlpf", &value);
            if (err == ESP_OK)
            {
                mpu_config.dlpf_threshold = (dlpf_config_t)value;
            }
            else
            {
                ESP_LOGE(TAG, "Error reading transport_type from NVS");
                nvs_close(nvs_handle);
                return;
            }

            nvs_close(nvs_handle);

            err = blackbox_configure_mpu6050(mpu, &mpu_config);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to configure MPU6050");
                return;
            }

            ESP_LOGI(TAG, "MPU6050 configured successfully with:");
            ESP_LOGI(TAG, "Gyro range: %d", mpu_config.gyro_range);
            ESP_LOGI(TAG, "Acce range: %d", mpu_config.acce_range);
            ESP_LOGI(TAG, "DLPF Threshold: %d", mpu_config.dlpf_threshold);

            sensor_configured = true;
            first_reading = true;
        }
        if (is_recording && got_time)
        {
            mpu6050_acce_value_t acce;
            mpu6050_gyro_value_t gyro;
            mpu6050_get_acce(mpu, &acce);
            mpu6050_get_gyro(mpu, &gyro);

            bool should_store = first_reading;
            if (!first_reading)
            {
                float threshold = 0.90f;
                should_store = (fabsf(fabsf(acce.acce_x) - fabsf(prev_acce_x)) > fabsf(fabsf(prev_acce_x) * threshold)) ||
                               (fabsf(fabsf(acce.acce_y) - fabsf(prev_acce_y)) > fabsf(fabsf(prev_acce_y) * threshold)) ||
                               (fabsf(fabsf(acce.acce_z) - fabsf(prev_acce_z)) > fabsf(fabsf(prev_acce_z) * threshold));

                //    (fabsf(fabsf(gyro.gyro_x) - fabsf(prev_gyro_x)) > fabsf(fabsf(prev_gyro_x) * threshold)) ||
                //    (fabsf(fabsf(gyro.gyro_y) - fabsf(prev_gyro_y)) > fabsf(fabsf(prev_gyro_y) * threshold)) ||
                //    (fabsf(fabsf(gyro.gyro_z) - fabsf(prev_gyro_z)) > fabsf(fabsf(prev_gyro_z) * threshold))
                // printf("%f - %f > %f * %f\n", acce.acce_x, prev_acce_x, prev_acce_x, threshold);
                // printf("%f > %lf\n", fabsf(fabsf(acce.acce_x) - fabsf(prev_acce_x)), fabsf(fabsf(prev_acce_x) * threshold));
                // printf("%f - %f > %f * %f\n", acce.acce_y, prev_acce_y, prev_acce_y, threshold);
                // printf("%f > %lf\n", fabsf(fabsf(acce.acce_y) - fabsf(prev_acce_y)), fabsf(fabsf(prev_acce_y) * threshold));
                // printf("%f - %f > %f * %f\n", acce.acce_z, prev_acce_z, prev_acce_z, threshold);
                // printf("%f > %lf\n", fabsf(fabsf(acce.acce_z) - fabsf(prev_acce_z)), fabsf(fabsf(prev_acce_z) * threshold));
                // printf("%f - %f > %f * %f\n", gyro.gyro_x, prev_gyro_x, prev_gyro_x, threshold);
                // printf("%f > %lf\n", fabsf(fabsf(gyro.gyro_x) - fabsf(prev_gyro_x)), fabsf(fabsf(prev_gyro_x) * threshold));
                // printf("%f - %f > %f * %f\n", gyro.gyro_y, prev_gyro_y, prev_gyro_y, threshold);
                // printf("%f > %lf \n", fabsf(fabsf(gyro.gyro_y) - fabsf(prev_gyro_y)), fabsf(fabsf(prev_gyro_y) * threshold));
                // printf("%f - %f > %f * %f\n", gyro.gyro_z, prev_gyro_z, prev_gyro_z, threshold);
                // printf("%f > %lf\n", fabsf(fabsf(gyro.gyro_z) - fabsf(prev_gyro_z)), fabsf(fabsf(prev_gyro_z) * threshold));
            }

            if (should_store)
            {
                store_mpu6050_reading(acce.acce_x, acce.acce_y, acce.acce_z,
                                      gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);

                prev_acce_x = acce.acce_x;
                prev_acce_y = acce.acce_y;
                prev_acce_z = acce.acce_z;
                prev_gyro_x = gyro.gyro_x;
                prev_gyro_y = gyro.gyro_y;
                prev_gyro_z = gyro.gyro_z;
                first_reading = false;
                ESP_LOGI(TAG, "[MPU6050] Accel: X:%.2f Y:%.2f Z:%.2f, Gyro: X:%.2f Y:%.2f Z:%.2f",
                         acce.acce_x, acce.acce_y, acce.acce_z,
                         gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(MPU6050_INTERVAL * 1000));
    }
}

static void bmp280_task(void *pvParameters)
{
    bmp280_config_t init_config = {
        .i2c_port = I2C_NUM_0,
        .i2c_addr = BMP280_SENSOR_ADDR,
        .mode = MODE_NORMAL};

    esp_err_t err = bmp280_init(&init_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize BMP280");
        vTaskDelete(NULL);
        return;
    }

    bool sensor_configured = false;
    int measurement_interval = 5000;

    while (1)
    {
        if (dev_state == DEVICE_STATE_AFTER_TRAVEL)
        {
            sensor_configured = false;
        }
        if ((dev_state == DEVICE_STATE_READY_TO_TRAVEL || dev_state == DEVICE_STATE_TRAVELLING) && !sensor_configured)
        {
            nvs_handle_t nvs_handle;
            err = nvs_open("config", NVS_READONLY, &nvs_handle);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Error opening NVS handle");
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }

            int32_t value;
            bmp280_osrs_p_t osrs_p = OSRS_P_X1;
            bmp280_osrs_t_t osrs_t = OSRS_T_X1;
            bmp280_filter_t filter = FILTER_OFF;

            err = nvs_get_i32(nvs_handle, "osrs_p", &value);
            if (err == ESP_OK)
            {
                osrs_p = (bmp280_osrs_p_t)value;
            }

            err = nvs_get_i32(nvs_handle, "osrs_t", &value);
            if (err == ESP_OK)
            {
                osrs_t = (bmp280_osrs_t_t)value;
            }

            err = nvs_get_i32(nvs_handle, "filter", &value);
            if (err == ESP_OK)
            {
                filter = (bmp280_filter_t)value;
            }

            err = nvs_get_i32(nvs_handle, "interval", &value);
            if (err == ESP_OK)
            {
                measurement_interval = value * 1000;
            }

            nvs_close(nvs_handle);

            err = bmp280_configure_regs(
                osrs_p,
                osrs_t,
                filter,
                T_SB_1000);

            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to configure BMP280 registers");
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }

            ESP_LOGI(TAG, "BMP280 configured successfully with:");
            ESP_LOGI(TAG, "OSRS_P: %d", osrs_p);
            ESP_LOGI(TAG, "OSRS_T: %d", osrs_t);
            ESP_LOGI(TAG, "Filter: %d", filter);
            ESP_LOGI(TAG, "Interval: %d ms", measurement_interval);
            ESP_LOGI(TAG, "T_SB: T_SB_1000 (fixed)");

            sensor_configured = true;
        }

        if (is_recording && sensor_configured && got_time)
        {
            float temperature, pressure;
            err = bmp280_read_measurements(&temperature, &pressure);
            if (err == ESP_OK)
            {
                double normalized_pressure = normalize_pressure(pressure, LOCATION_ALTITUDE);
                double pressure_hpa = normalized_pressure / 100.0;

                ESP_LOGI(TAG, "[BMP280] Temp: %.2f°C, Pressure: %.2f hPa (normalized)",
                         temperature, pressure_hpa);
                store_bmp280_reading(temperature, pressure_hpa, normalized_pressure);
            }
            else
            {
                ESP_LOGE(TAG, "Failed to read BMP280 measurements");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(measurement_interval));
    }
}

static void ky026_task(void *pvParameters)
{
    esp_err_t err = ky026_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize KY026");
        vTaskDelete(NULL);
        return;
    }

    while (1)
    {
        if (is_recording && got_time)
        {
            int digital_val = ky026_read_digital();
            int voltage = ky026_read_voltage();
            int raw = ky026_read_raw_analog();

            ESP_LOGI(TAG, "[KY026] Digital: %d,Raw: %d, Analog: %d mV, Flame %s",
                     digital_val,
                     raw,
                     voltage,
                     digital_val == 1 ? "DETECTED!" : "not detected");
            if (digital_val == 1)
            {
                store_ky026_reading(digital_val, raw, voltage);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(KY026_INTERVAL * 1000));
    }
}

#define BUTTON_PIN GPIO_NUM_6
#define DEBOUNCE_TIME_MS 50
#define STACK_SIZE 2048
#define TASK_PRIORITY 1

void button_task(void *pvParameters)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};

    gpio_config(&io_conf);

    int last_state = 1;

    ESP_LOGI(TAG, "Button task started. Monitoring button on GPIO %d", BUTTON_PIN);

    while (1)
    {
        int current_state = gpio_get_level(BUTTON_PIN);

        if (current_state != last_state)
        {
            vTaskDelay(pdMS_TO_TICKS(50));

            current_state = gpio_get_level(BUTTON_PIN);

            if (current_state != last_state)
            {
                if (current_state == 0)
                {
                    ESP_LOGI(TAG, "Button pressed!");
                    verified = false;
                    esp_err_t ret = nvs_flash_erase();
                    if (ret != ESP_OK)
                    {
                        ESP_LOGE(TAG, "Failed to erase NVS");
                    }

                    free_sensor_storage();
                    vTaskDelay(pdMS_TO_TICKS(200));
                    init_sensor_storage();

                    ret = nvs_flash_init();
                    if (ret != ESP_OK)
                    {
                        ESP_LOGE(TAG, "Failed to init NVS after erase");
                    }
                    else
                    {
                        esp_err_t err = save_device_state(DEVICE_STATE_NOT_CONFIGURED);
                        if (err != ESP_OK)
                        {
                            ESP_LOGE(TAG, "Failed to save device state");
                        }
                        else
                        {
                            ESP_LOGI(TAG, "Changed device state to %d", DEVICE_STATE_NOT_CONFIGURED);
                            if (wifi_connected)
                            {
                                wifi_stop();
                            }
                        }
                    }
                }
                else
                {
                    ESP_LOGI(TAG, "Button released!");
                }
                last_state = current_state;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

#define PRESS_LED_PIN 2
static void boot_button_task(void *pvParameters)
{
    bool last_state = true;
    uint32_t press_start = 0;
    bool long_press_handled = false;

    while (1)
    {
        bool current_state = gpio_get_level(BOOT_BUTTON);
        uint32_t current_time = xTaskGetTickCount();

        if (last_state == true && current_state == false)
        {
            press_start = current_time;
            long_press_handled = false;
        }
        else if (current_state == false && !long_press_handled)
        {
            if (dev_state != 1 && dev_state != 2)
            {
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            }

            if ((current_time - press_start) > pdMS_TO_TICKS(3000))
            {
                is_recording = !is_recording;
                if (dev_state == 1 && is_recording)
                {
                    if (wifi_connected && !got_time)
                    {
                        esp_err_t ret = init_storage_time();
                        if (ret == ESP_OK)
                        {
                            got_time = true;
                        }
                        else
                        {
                            ESP_LOGE(TAG, "Failed to initialize time");
                        }
                    }
                    esp_err_t err = save_device_state(DEVICE_STATE_TRAVELLING);
                    if (err != ESP_OK)
                    {
                        ESP_LOGE(TAG, "Failed to save device state");
                    }
                    else
                    {
                        ESP_LOGI(TAG, "Changed device state to %d", DEVICE_STATE_TRAVELLING);
                    }
                }
                if (dev_state == 2 && !is_recording)
                {
                    esp_err_t err = save_device_state(DEVICE_STATE_AFTER_TRAVEL);
                    if (err != ESP_OK)
                    {
                        ESP_LOGE(TAG, "Failed to save device state");
                    }
                    else
                    {
                        ESP_LOGI(TAG, "Changed device state to %d", DEVICE_STATE_AFTER_TRAVEL);
                    }
                }
                ESP_LOGI(TAG, "Long press detected - Recording %s", is_recording ? "STARTED" : "STOPPED");
                long_press_handled = true;

                bool led_state = false;
                for (int i = 0; i < 10; i++)
                {
                    led_state = !led_state;
                    gpio_set_level(LED_GREEN, led_state);
                    vTaskDelay(pdMS_TO_TICKS(50));
                }
            }
        }
        else if (last_state == false && current_state == true)
        {
            if (!long_press_handled && (current_time - press_start) <= pdMS_TO_TICKS(3000))
            {
                ESP_LOGI(TAG, "Short press detected!");
                ESP_LOGI(TAG, "BLE ENABLED: %d", !ble_enabled());

                if (!ble_enabled())
                {
                    ble_toggle(true);
                }
                else
                {
                    ble_toggle(false);
                }
            }
        }

        last_state = current_state;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static const char *print_timestamp(uint32_t timestamp)
{
    static char time_str[32];
    time_t time = (time_t)timestamp;
    struct tm timeinfo;
    localtime_r(&time, &timeinfo);

    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &timeinfo);

    return time_str;
}

static char *create_mpu6050_json(const mpu6050_reading_t *reading)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "timestamp", reading->timestamp);
    cJSON_AddNumberToObject(root, "rotation_degrees_x", reading->accel_x);
    cJSON_AddNumberToObject(root, "rotation_degrees_y", reading->accel_y);
    cJSON_AddNumberToObject(root, "rotation_degrees_z", reading->accel_z);
    cJSON_AddNumberToObject(root, "gx", reading->gyro_x);
    cJSON_AddNumberToObject(root, "gy", reading->gyro_y);
    cJSON_AddNumberToObject(root, "gz", reading->gyro_z);

    char *json_str = cJSON_Print(root);
    cJSON_Delete(root);
    return json_str;
}

static char *create_bmp280_json(const bmp280_reading_t *reading)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "timestamp", reading->timestamp);
    cJSON_AddNumberToObject(root, "temperature", reading->temperature);
    cJSON_AddNumberToObject(root, "pressure", reading->pressure);

    char *json_str = cJSON_Print(root);
    cJSON_Delete(root);
    return json_str;
}

static char *create_ky026_json(const ky026_reading_t *reading)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "timestamp", reading->timestamp);
    cJSON_AddNumberToObject(root, "fire_detected", reading->digital_value);
    cJSON_AddNumberToObject(root, "sensor_value", reading->analog_value);

    char *json_str = cJSON_Print(root);
    cJSON_Delete(root);
    return json_str;
}
extern bool config_received;
static bool was_sntp_initialized = false;
extern bool wifi_connecting;

static void wifi_task(void *pvParameters)
{

    while (1)
    {
        ESP_LOGI(TAG, "Current device state: %d", dev_state);
        ESP_LOGI(TAG, "WIFI CONNECTED: %d", wifi_connected);
        ESP_LOGI(TAG, "MQTT CONNECTED: %d", mqtt_connected);
        ESP_LOGI(TAG, "GOT TIME: %d", got_time);
        if (dev_state == 2)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        if (dev_state == -1 || dev_state == 0 || dev_state == 3 || dev_state == 1)
        {
            if (!wifi_connecting)
            {
                ESP_LOGE(TAG, "WiFi CONNECTING MAIN\n\n");
                wifi_connecting = true;
                wifi_connection();
            }

            for (int i = 0; i < 50; i++)
            {
                vTaskDelay(pdMS_TO_TICKS(100));
                if (wifi_connected && !was_sntp_initialized)
                {
                    init_time_once();
                    was_sntp_initialized = true;
                }
                if (wifi_connected && mqtt_connected == 1)
                {
                    if (got_pin && !verified && dev_state == -1)
                    {
                        ESP_LOGW(TAG, "GOING TO VERIFY PIN");
                        vTaskDelay(pdMS_TO_TICKS(10));
                        printf("PIN= ===== %s\n", pin_value);
                        verified = mqtt_verify_pin(pin_value);
                        vTaskDelay(pdMS_TO_TICKS(10));

                        printf("PIN VERIFY ===== %d\n", verified);
                        got_pin = false;
                        if (verified)
                        {

                            esp_err_t err = save_device_state(DEVICE_STATE_CONFIGURING);
                            if (err != ESP_OK)
                            {
                                ESP_LOGE(TAG, "Failed to save device state");
                            }
                            else
                            {
                                ESP_LOGI(TAG, "Changed device state to %d", DEVICE_STATE_CONFIGURING);
                                gpio_set_level(LED_PIN, 0);
                            }
                        }
                    }
                    if (dev_state == 3)
                    {
                        size_t mpu_count = 0, bmp_count = 0, ky026_count = 0;

                        const mpu6050_reading_t *mpu_data = get_mpu6050_readings(&mpu_count);
                        const bmp280_reading_t *bmp_data = get_bmp280_readings(&bmp_count);
                        const ky026_reading_t *ky026_data = get_ky026_readings(&ky026_count);

                        ESP_LOGI(TAG, "-------- ZAPISANE DANE --------");
                        ESP_LOGI(TAG, "MPU6050 - liczba pomiarow: %zu", mpu_count);
                        for (size_t i = 0; i < mpu_count; i++)
                        {
                            ESP_LOGI(TAG, "[%zu] %s acc(x:%.2f y:%.2f z:%.2f) gyro(x:%.2f y:%.2f z:%.2f)",
                                     i,
                                     print_timestamp(mpu_data[i].timestamp),
                                     mpu_data[i].accel_x, mpu_data[i].accel_y, mpu_data[i].accel_z,
                                     mpu_data[i].gyro_x, mpu_data[i].gyro_y, mpu_data[i].gyro_z);
                        }

                        ESP_LOGI(TAG, "BMP280 - liczba pomiarow: %zu", bmp_count);
                        for (size_t i = 0; i < bmp_count; i++)
                        {
                            ESP_LOGI(TAG, "[%zu] %s temp:%.2f press:%.2f norm_press:%.2f",
                                     i,
                                     print_timestamp(bmp_data[i].timestamp),
                                     bmp_data[i].temperature,
                                     bmp_data[i].pressure,
                                     bmp_data[i].normalized_pressure);
                        }

                        ESP_LOGI(TAG, "KY026 - liczba pomiarow: %zu", ky026_count);
                        for (size_t i = 0; i < ky026_count; i++)
                        {
                            ESP_LOGI(TAG, "[%zu] %s digital:%u analog:%u voltage:%u",
                                     i,
                                     print_timestamp(ky026_data[i].timestamp),
                                     ky026_data[i].digital_value,
                                     ky026_data[i].analog_value,
                                     ky026_data[i].voltage);
                        }
                        ESP_LOGI(TAG, "-------- KONIEC DANYCH --------");

                        yellow_led = true;
                        send_control_signal(true);
                        bool led_state = 0;
                        for (size_t i = 0; i < mpu_count; i++)
                        {
                            char *json = create_mpu6050_json(&mpu_data[i]);
                            if (json)
                            {
                                mqtt_publish_data(json, "mpu6050");

                                free(json);
                                vTaskDelay(pdMS_TO_TICKS(10));
                            }
                        }

                        for (size_t i = 0; i < bmp_count; i++)
                        {
                            char *json = create_bmp280_json(&bmp_data[i]);
                            if (json)
                            {

                                mqtt_publish_data(json, "bmp280");
                                free(json);
                                vTaskDelay(pdMS_TO_TICKS(10));
                            }
                        }

                        for (size_t i = 0; i < ky026_count; i++)
                        {
                            char *json = create_ky026_json(&ky026_data[i]);
                            if (json)
                            {

                                mqtt_publish_data(json, "ky026");

                                free(json);
                                vTaskDelay(pdMS_TO_TICKS(10));
                            }
                        }

                        send_control_signal(false);
                        yellow_led = false;

                        free_sensor_storage();
                        vTaskDelay(pdMS_TO_TICKS(500));
                        gpio_set_level(LED_YELLOW, 0);
                        init_sensor_storage();

                        esp_err_t err = save_device_state(DEVICE_STATE_CONFIGURING);
                        if (err != ESP_OK)
                        {
                            ESP_LOGE(TAG, "Failed to save device state");
                        }
                        else
                        {
                            ESP_LOGI(TAG, "Changed device state to %d", DEVICE_STATE_CONFIGURING);
                            config_received = false;
                            got_time = false;
                        }
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void wait_for_config_task(void *pvParameters)
{
    while (1)
    {
        if (dev_state == DEVICE_STATE_CONFIGURING && wifi_connected && mqtt_connected == 1)
        {
            if (config_received)
            {
                esp_err_t err = save_device_state(DEVICE_STATE_READY_TO_TRAVEL);
                if (err != ESP_OK)
                {
                    ESP_LOGE(TAG, "Failed to save device state");
                }
                else
                {
                    ESP_LOGI(TAG, "Device state is %d", DEVICE_STATE_READY_TO_TRAVEL);
                    if (!got_time)
                    {
                        esp_err_t ret = init_storage_time();
                        if (ret == ESP_OK)
                        {
                            got_time = true;
                        }
                        else
                        {
                            ESP_LOGE(TAG, "Failed to initialize time");
                        }
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

#define LED_BUILTIN 48

static void led_ble_task(void *pvParameters)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_BUILTIN,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags = {
            .invert_out = false,
        }};

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 64,
        .flags = {
            .with_dma = false,
        }};

    led_strip_handle_t led_strip;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    led_strip_clear(led_strip);

    while (1)
    {
        if (ble_enabled())
        {
            led_strip_set_pixel(led_strip, 0, 0, 0, 150);
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(1000));

            led_strip_set_pixel(led_strip, 0, 0, 0, 0);
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        else
        {
            led_strip_set_pixel(led_strip, 0, 0, 0, 0);
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

void app_main(void)
{

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(check_device_state());
    ESP_LOGI(TAG, "Initializing I2C...");
    ESP_ERROR_CHECK(init_i2c());
    init_sensor_storage();
    if (dev_state == 3)
    {
        esp_err_t err = save_device_state(DEVICE_STATE_CONFIGURING);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to save device state");
        }
        else
        {
            ESP_LOGI(TAG, "Changed device state to %d", DEVICE_STATE_CONFIGURING);
        }
    }
    else if (dev_state == 2)
    {
        is_recording = true;
        got_time = true;
    }
    configure_ledGreen();
    configure_ledYellow();
    configure_ledRed();
    ESP_LOGI(TAG, "Configuring boot button...");

    ESP_LOGI(TAG, "Creating sensor tasks...");

    xTaskCreate(mpu6050_task, "mpu6050_task", 4096, NULL, 5, NULL);
    xTaskCreate(bmp280_task, "bmp280_task", 4096, NULL, 5, NULL);
    xTaskCreate(ky026_task, "ky026_task", 4096, NULL, 5, NULL);
    xTaskCreate(
        button_task,
        "button_task",
        4096,
        NULL,
        TASK_PRIORITY,
        NULL);
    xTaskCreate(boot_button_task, "boot_button_task", 4096, NULL, 5, NULL);
    xTaskCreate(wifi_task, "wifi_task", 4096, NULL, 5, NULL);
    mqtt_init();
    xTaskCreate(led_ble_task,
                "led_ble_task",
                4096,
                NULL,
                2,
                NULL);
    xTaskCreate(wait_for_config_task, "wait_for_config_task", 8192, NULL, 5, NULL);
    ESP_LOGI(TAG, "System initialized. Hold boot button for 3s to start/stop recording.");

    ESP_LOGI(LEDTAG, "Tworzenie zadania migania LED");

    xTaskCreate(
        led_blink_task,
        "led_blink_task",
        2048,
        NULL,
        5,
        NULL);
    xTaskCreate(
        led_yellow_task,
        "led_yellow_task",
        2048,
        NULL,
        5,
        NULL);

    ESP_LOGI(LEDTAG, "Aplikacja wystartowana");
}