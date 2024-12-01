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
#include "protocol_examples_common.h"
#include "cJSON.h"
#include "esp_mac.h"

#define USER_ID "BlackBox_User01"
#define MQTT_BROKER_URL "mqtt://192.168.137.1:1883/"

char DEVICE_ID[18];
static const char *TAG = "mqtt_sensor_client";
int mqtt_connected = 0;

typedef struct
{
    float temperature;
    float humidity;
    float light;
} sensor_data_t;

void get_mac_address(void)
{
    // Zmienna przechowująca adres MAC
    uint8_t mac[6];

    // Pobranie adresu MAC dla interfejsu WiFi (base MAC address)
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    sprintf(DEVICE_ID, "%02X:%02X:%02X:%02X:%02X:%02X",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    ESP_LOGI(TAG, "WiFi Station MAC Address: %s", DEVICE_ID);
}

static float generate_random_float(float min, float max)
{
    float scale = rand() / (float)RAND_MAX;
    return min + scale * (max - min);
}

static void read_sensor_data(sensor_data_t *data)
{
    data->temperature = generate_random_float(18.0, 30.0);
    data->humidity = generate_random_float(30.0, 80.0);
    data->light = generate_random_float(0.0, 1000.0);
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

static void publish_sensor_data(esp_mqtt_client_handle_t client, char *sensor, sensor_data_t *data)
{
    char topic[128];

    // Create JSON object
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "device_id", DEVICE_ID);
    cJSON_AddStringToObject(root, "user_id", USER_ID);
    cJSON_AddNumberToObject(root, "temperature", data->temperature);
    cJSON_AddNumberToObject(root, "humidity", data->humidity);
    cJSON_AddNumberToObject(root, "light", data->light);

    // Get timestamp
    time_t now;
    time(&now);
    cJSON_AddNumberToObject(root, "timestamp", now);

    // Convert JSON to string
    char *json_string = cJSON_Print(root);

    // Create single topic for all sensor data
    snprintf(topic, sizeof(topic), "/%s/%s/%s/sensor_data", USER_ID, DEVICE_ID, sensor);

    // Publish JSON data
    esp_mqtt_client_publish(client, topic, json_string, 0, 1, 0);

    ESP_LOGI(TAG, "Published: %s", json_string);

    // Cleanup
    cJSON_Delete(root);
    free(json_string);
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URL,
        .credentials.username = "mqtt",
        .credentials.authentication.password = "MosquitoBroker",
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);

    sensor_data_t sensor_data;
    srand(time(NULL));

    while (1)
    {
        int connect_flag = 1;
        for (int i = 0; i < 50; i++)
        {
            if (mqtt_connected && connect_flag)
            {
                vTaskDelay(pdMS_TO_TICKS(10));
                connect_flag = 0;
                for (int j = 0; j < 3; j++)
                {
                    read_sensor_data(&sensor_data);

                    char sensor_name[64] = "sensor";
                    char sensor_index = '1' + j;
                    strncat(sensor_name, &sensor_index, 1);

                    publish_sensor_data(client, sensor_name, &sensor_data);
                }
            }
            else if (!mqtt_connected)
            {
                vTaskDelay(pdMS_TO_TICKS(10));
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    get_mac_address();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());
    mqtt_app_start();
}