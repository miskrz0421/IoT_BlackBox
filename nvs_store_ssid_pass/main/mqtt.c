#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "cJSON.h"
#include "esp_mac.h"
#include "mqtt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MQTT_BROKER_URL "mqtt://192.168.137.1:1883/"
#define TAG "mqtt_sensor_client"
#define MQTT_TASK_STACK_SIZE 4096
#define MQTT_TASK_PRIORITY 5

char DEVICE_MAC[18];
int mqtt_connected = 0;
esp_mqtt_client_handle_t mqtt_client = NULL;

extern bool wifi_connected;

void get_mac_address(void)
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    sprintf(DEVICE_MAC, "%02X:%02X:%02X:%02X:%02X:%02X",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    ESP_LOGI(TAG, "WiFi Station MAC Address: %s", DEVICE_MAC);
}

static float generate_mock_temperature(void)
{
    float scale = rand() / (float)RAND_MAX;
    return -40.0 + scale * (125.0);
}

static float generate_mock_pressure(void)
{
    float scale = rand() / (float)RAND_MAX;
    return 300.0 + scale * (800.0);
}

static void stop_mqtt_client(void)
{
    if (mqtt_client != NULL)
    {
        esp_mqtt_client_stop(mqtt_client);
        esp_mqtt_client_destroy(mqtt_client);
        mqtt_client = NULL;
        mqtt_connected = 0;
        ESP_LOGI(TAG, "MQTT Client stopped and destroyed");
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT Connected");
        mqtt_connected = 1;
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT Disconnected");
        mqtt_connected = 0;
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT Error");
        mqtt_connected = 0;
        break;
    default:
        break;
    }
}
static void start_mqtt_client(void)
{
    if (mqtt_client == NULL)
    {
        esp_mqtt_client_config_t mqtt_cfg = {
            .broker.address.uri = MQTT_BROKER_URL,
            .credentials.username = "mqtt",
            .credentials.authentication.password = "MosquitoBroker",
        };

        mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
        esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
        esp_mqtt_client_start(mqtt_client);
        ESP_LOGI(TAG, "MQTT Client started");
    }
}

static void publish_sensor_data(void)
{
    if (!mqtt_connected || mqtt_client == NULL)
        return;

    nvs_handle_t nvs_handle;
    char user_id[32] = "Undefined";
    esp_err_t err = nvs_open("config", NVS_READONLY, &nvs_handle);
    if (err == ESP_OK)
    {
        size_t user_len = sizeof(user_id);
        err = nvs_get_str(nvs_handle, "user", user_id, &user_len);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Error reading user ID from NVS, using default: %s", user_id);
        }
        nvs_close(nvs_handle);
    }
    else
    {
        ESP_LOGE(TAG, "Error opening NVS handle for user ID, using default: %s", user_id);
    }

    char temp_topic[128];
    char pressure_topic[128];
    char temp_payload[32];
    char pressure_payload[32];

    float temperature = generate_mock_temperature();
    float pressure = generate_mock_pressure();

    snprintf(temp_topic, sizeof(temp_topic), "BlackBox/%s/%s/temperature", user_id, DEVICE_MAC);
    snprintf(pressure_topic, sizeof(pressure_topic), "BlackBox/%s/%s/pressure", user_id, DEVICE_MAC);

    snprintf(temp_payload, sizeof(temp_payload), "%.2f", temperature);
    snprintf(pressure_payload, sizeof(pressure_payload), "%.2f", pressure);

    esp_mqtt_client_publish(mqtt_client, temp_topic, temp_payload, 0, 1, 0);
    ESP_LOGI(TAG, "Published temperature: %s to topic: %s", temp_payload, temp_topic);

    esp_mqtt_client_publish(mqtt_client, pressure_topic, pressure_payload, 0, 1, 0);
    ESP_LOGI(TAG, "Published pressure: %s to topic: %s", pressure_payload, pressure_topic);
}

static void mqtt_task(void *pvParameters)
{
    bool prev_wifi_state = false;
    srand(time(NULL));

    while (1)
    {

        if (wifi_connected != prev_wifi_state)
        {
            if (wifi_connected)
            {
                ESP_LOGI(TAG, "WiFi connected, starting MQTT client");
                vTaskDelay(pdMS_TO_TICKS(2000));
                start_mqtt_client();
            }
            else
            {
                ESP_LOGI(TAG, "WiFi disconnected, stopping MQTT client");
                stop_mqtt_client();
            }
            prev_wifi_state = wifi_connected;
        }

        if (wifi_connected && mqtt_connected)
        {
            publish_sensor_data();
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

static void mqtt_app_start(void)
{
    xTaskCreate(mqtt_task,
                "mqtt_task",
                MQTT_TASK_STACK_SIZE,
                NULL,
                MQTT_TASK_PRIORITY,
                NULL);
}

void mqtt_init(void)
{

    get_mac_address();

    mqtt_app_start();
}