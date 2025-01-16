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

#define TAG "mqtt_sensor_client"
#define MQTT_TASK_STACK_SIZE 4096
#define MQTT_TASK_PRIORITY 5

char DEVICE_MAC[18];
int mqtt_connected = 0;
static esp_mqtt_client_handle_t mqtt_client = NULL;

extern bool wifi_connected;

void get_mac_address(void)
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    sprintf(DEVICE_MAC, "%02X:%02X:%02X:%02X:%02X:%02X",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    ESP_LOGI(TAG, "WiFi Station MAC Address: %s", DEVICE_MAC);
}

static void stop_mqtt_client(void)
{
    if (mqtt_client != NULL)

    {
        esp_mqtt_client_disconnect(mqtt_client);
        esp_mqtt_client_stop(mqtt_client);
        esp_mqtt_client_destroy(mqtt_client);
        mqtt_client = NULL;
        mqtt_connected = 0;
        ESP_LOGI(TAG, "MQTT Client stopped and destroyed");
    }
}
bool pin_verified = false;
bool response_received = false;
bool mqtt_verify_pin(char *pin_value)
{
    if (!mqtt_connected || mqtt_client == NULL)
    {
        ESP_LOGE(TAG, "MQTT not connected");
        return false;
    }

    nvs_handle_t nvs_handle;
    char username[32] = "Undefined";
    esp_err_t err = nvs_open("config", NVS_READONLY, &nvs_handle);
    if (err == ESP_OK)
    {
        size_t username_len = sizeof(username);
        err = nvs_get_str(nvs_handle, "username", username, &username_len);
        nvs_close(nvs_handle);

        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Error reading username from NVS");
            return false;
        }
    }
    else
    {
        ESP_LOGE(TAG, "Error opening NVS handle for username");
        return false;
    }

    char pin_topic[128];
    char response_topic[128];

    snprintf(pin_topic, sizeof(pin_topic), "BlackBox/%s/%s/Pin", username, DEVICE_MAC);
    snprintf(response_topic, sizeof(response_topic), "BlackBox/%s/%s/Response", username, DEVICE_MAC);

    ESP_LOGI(TAG, "Pin topic: %s", pin_topic);
    ESP_LOGI(TAG, "Response topic: %s", response_topic);
    response_received = false;
    pin_verified = false;

    esp_mqtt_client_subscribe(mqtt_client, response_topic, 0);

    int msg_id = esp_mqtt_client_publish(mqtt_client, pin_topic, pin_value, strlen(pin_value), 0, 0);
    if (msg_id == -1)
    {
        ESP_LOGE(TAG, "Failed to publish PIN");
        return false;
    }

    int timeout_counter = 0;
    while (!response_received && timeout_counter < 50)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
        timeout_counter++;
    }

    if (!response_received)
    {
        ESP_LOGE(TAG, "PIN verification timeout");
        return false;
    }

    return pin_verified;
}
void mqtt_publish_data(char *json, char *sensor_name)
{
    if (!mqtt_connected || mqtt_client == NULL)
    {
        ESP_LOGE(TAG, "MQTT not connected");
        return;
    }

    nvs_handle_t nvs_handle;
    char username[32] = "Undefined";
    esp_err_t err = nvs_open("config", NVS_READONLY, &nvs_handle);
    if (err == ESP_OK)
    {
        size_t username_len = sizeof(username);
        err = nvs_get_str(nvs_handle, "username", username, &username_len);
        nvs_close(nvs_handle);

        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Error reading username from NVS");
            return;
        }
    }
    else
    {
        ESP_LOGE(TAG, "Error opening NVS handle for username");
        return;
    }

    char response_topic[128];
    snprintf(response_topic, sizeof(response_topic), "BlackBox/%s/%s/%s", username, DEVICE_MAC, sensor_name);

    int msg_id = esp_mqtt_client_publish(mqtt_client, response_topic, json, strlen(json), 1, 0);
    if (msg_id == -1)
    {
        ESP_LOGE(TAG, "Failed to publish config response");
        return;
    }

    ESP_LOGI(TAG, "Config response sent successfully on topic: %s", response_topic);
}
bool send_config_response(void)
{
    if (!mqtt_connected || mqtt_client == NULL)
    {
        ESP_LOGE(TAG, "MQTT not connected");
        return false;
    }

    nvs_handle_t nvs_handle;
    char username[32] = "Undefined";
    esp_err_t err = nvs_open("config", NVS_READONLY, &nvs_handle);
    if (err == ESP_OK)
    {
        size_t username_len = sizeof(username);
        err = nvs_get_str(nvs_handle, "username", username, &username_len);
        nvs_close(nvs_handle);

        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Error reading username from NVS");
            return false;
        }
    }
    else
    {
        ESP_LOGE(TAG, "Error opening NVS handle for username");
        return false;
    }

    char response_topic[128];
    snprintf(response_topic, sizeof(response_topic), "BlackBox/%s/%s/ConfigResponse", username, DEVICE_MAC);

    int msg_id = esp_mqtt_client_publish(mqtt_client, response_topic, "1", 1, 0, 0);
    if (msg_id == -1)
    {
        ESP_LOGE(TAG, "Failed to publish config response");
        return false;
    }

    ESP_LOGI(TAG, "Config response sent successfully on topic: %s", response_topic);
    return true;
}
bool config_received = false;

void send_control_signal(bool start)
{
    char topic[128];
    nvs_handle_t nvs_handle;
    char username[32] = "Undefined";
    esp_err_t err = nvs_open("config", NVS_READONLY, &nvs_handle);
    if (err == ESP_OK)
    {
        size_t username_len = sizeof(username);
        err = nvs_get_str(nvs_handle, "username", username, &username_len);
        nvs_close(nvs_handle);

        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Error reading username from NVS");
            return;
        }
    }
    else
    {
        ESP_LOGE(TAG, "Error opening NVS handle for username");
        return;
    }
    snprintf(topic, sizeof(topic), "BlackBox/%s/%s/Control", username, DEVICE_MAC);
    esp_mqtt_client_publish(mqtt_client, topic, start ? "1" : "0", 1, 1, 0);
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT Connected");
        nvs_handle_t nvs_handle;
        char username[32] = "Undefined";
        esp_err_t err = nvs_open("config", NVS_READONLY, &nvs_handle);
        if (err == ESP_OK)
        {
            size_t username_len = sizeof(username);
            err = nvs_get_str(nvs_handle, "username", username, &username_len);
            nvs_close(nvs_handle);

            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Error reading username from NVS");
                return;
            }
        }
        else
        {
            ESP_LOGE(TAG, "Error opening NVS handle for username");
            return;
        }

        char config_topic[128];
        snprintf(config_topic, sizeof(config_topic), "BlackBox/%s/%s/Config", username, DEVICE_MAC);
        esp_mqtt_client_subscribe(mqtt_client, config_topic, 0);
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

    case MQTT_EVENT_DATA:
    {
        char topic[128];
        memcpy(topic, event->topic, event->topic_len);
        topic[event->topic_len] = '\0';

        if (strstr(topic, "/Response") != NULL)
        {
            char payload[event->data_len + 1];
            memcpy(payload, event->data, event->data_len);
            payload[event->data_len] = '\0';

            if (strcmp(payload, "1") == 0)
            {
                pin_verified = true;
            }
            else
            {
                pin_verified = false;
            }

            response_received = true;
        }
        if (strstr(topic, "/Config") != NULL)
        {
            ESP_LOGI(TAG, "GOT CONFIG TOPIC");
            char payload[event->data_len + 1];
            memcpy(payload, event->data, event->data_len);
            payload[event->data_len] = '\0';

            cJSON *config = cJSON_Parse(payload);
            if (config == NULL)
            {
                ESP_LOGE(TAG, "Failed to parse config JSON");
                return;
            }

            ESP_LOGI(TAG, "Received config: %s", payload);

            nvs_handle_t nvs_handle;
            esp_err_t err = nvs_open("config", NVS_READWRITE, &nvs_handle);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Error opening NVS handle");
                cJSON_Delete(config);
                return;
            }

            struct
            {
                const char *key;
                cJSON *value;
            } config_items[] = {
                {"gyro", cJSON_GetObjectItem(config, "gyro")},
                {"acce", cJSON_GetObjectItem(config, "acce")},
                {"dlpf", cJSON_GetObjectItem(config, "dlpf")},
                {"osrs_p", cJSON_GetObjectItem(config, "osrs_p")},
                {"osrs_t", cJSON_GetObjectItem(config, "osrs_t")},
                {"filter", cJSON_GetObjectItem(config, "filter")},
                {"interval", cJSON_GetObjectItem(config, "interval")},
                {NULL, NULL}};

            bool config_valid = true;
            for (int i = 0; config_items[i].key != NULL; i++)
            {
                cJSON *item = config_items[i].value;
                if (item && item->valuestring)
                {
                    char *endptr;
                    int32_t value = (int32_t)strtol(item->valuestring, &endptr, 10);

                    if (*endptr == '\0')
                    {
                        err = nvs_set_i32(nvs_handle, config_items[i].key, value);
                        if (err != ESP_OK)
                        {
                            ESP_LOGE(TAG, "Failed to save %s to NVS", config_items[i].key);
                            config_valid = false;
                            break;
                        }
                        ESP_LOGI(TAG, "Saved %s: %ld", config_items[i].key, (long)value);
                    }
                    else
                    {
                        ESP_LOGE(TAG, "Invalid number format for %s", config_items[i].key);
                        config_valid = false;
                        break;
                    }
                }
                else
                {
                    ESP_LOGE(TAG, "Missing or invalid value for %s", config_items[i].key);
                    config_valid = false;
                    break;
                }
            }

            if (config_valid)
            {
                err = nvs_commit(nvs_handle);
                if (err == ESP_OK)
                {
                    ESP_LOGI(TAG, "Configuration saved successfully");
                    config_received = true;
                }
                else
                {
                    ESP_LOGE(TAG, "Failed to commit NVS changes");
                }
            }

            nvs_close(nvs_handle);
            cJSON_Delete(config);
            if (send_config_response())
            {
                ESP_LOGI(TAG, "Config response sent");
            }
            else
            {
                ESP_LOGE(TAG, "Failed to send config response");
            }
        }
    }
    break;

    default:
        break;
    }
}
static void start_mqtt_client(void)
{
    ESP_LOGI(TAG, "Starting mqtt client");

    if (mqtt_client != NULL)
    {
        ESP_LOGI(TAG, "Mqtt client is null ,destroying and stopping....");
        esp_mqtt_client_disconnect(mqtt_client);
        esp_mqtt_client_stop(mqtt_client);
        esp_mqtt_client_destroy(mqtt_client);
        mqtt_client = NULL;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    nvs_handle_t nvs_handle;
    char broker_url[32] = "mqtt://127.0.0.1:1883/";
    esp_err_t err = nvs_open("config", NVS_READONLY, &nvs_handle);
    if (err == ESP_OK)
    {
        size_t broker_len = sizeof(broker_url);
        err = nvs_get_str(nvs_handle, "mqtt_broker", broker_url, &broker_len);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Error reading broker url from NVS");
            nvs_close(nvs_handle);
            return;
        }
        nvs_close(nvs_handle);
    }
    else
    {
        ESP_LOGE(TAG, "Error opening NVS handle for user ID, using default: %s", broker_url);
    }

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = broker_url,
        .credentials.username = "mqtt",
        .credentials.authentication.password = "MosquitoBroker",
        .network.reconnect_timeout_ms = 10000,
        .network.disable_auto_reconnect = false};

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (mqtt_client == NULL)
    {
        ESP_LOGE(TAG, "Client init failed");
        return;
    }

    err = esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Client register event failed");
        esp_mqtt_client_destroy(mqtt_client);
        mqtt_client = NULL;
        return;
    }

    err = esp_mqtt_client_start(mqtt_client);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Client start failed");
        esp_mqtt_client_destroy(mqtt_client);
        mqtt_client = NULL;
        return;
    }

    ESP_LOGI(TAG, "MQTT Client started");
}

static void mqtt_task(void *pvParameters)
{

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(2000));
        if (wifi_connected && mqtt_connected == 0)
        {
            start_mqtt_client();
        }
        else if (!wifi_connected && mqtt_connected == 1)
        {
            stop_mqtt_client();
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