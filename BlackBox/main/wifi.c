#include "wifi.h"
#include "ble.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#define WIFI_TAG "WIFI"

bool wifi_connected = false;
bool wifi_connecting = false;

static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_STA_START)
    {
        ESP_LOGI(WIFI_TAG, "WIFI CONNECTING....\n");
        wifi_connecting = true;
        // vTaskDelay(pdMS_TO_TICKS(2500));
    }
    else if (event_id == WIFI_EVENT_STA_CONNECTED)
    {
        ESP_LOGI(WIFI_TAG, "WiFi CONNECTED\n");
        wifi_connected = true;
        wifi_connecting = false;
    }
    else if (event_id == WIFI_EVENT_STA_DISCONNECTED)
    {

        // if (ble_enabled())
        // {
        //     return;
        // }

        if (wifi_connected)
            wifi_stop();
        wifi_connected = false;
        // esp_wifi_connect();
        wifi_connecting = false;
        // ESP_LOGI(WIFI_TAG, "Retrying to Connect...(%d)\n", retry_num);
    }
    else if (event_id == IP_EVENT_STA_GOT_IP)
    {
        ESP_LOGI(WIFI_TAG, "WiFi got IP...\n\n");
        vTaskDelay(pdMS_TO_TICKS(400));
        wifi_connected = true;
        wifi_connecting = false;
    }
}

void wifi_connection()
{
    ESP_LOGI(WIFI_TAG, "STARTED WIFI CONNECTION FUN");

    char ssid[32] = "";
    char password[32] = "";
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("config", NVS_READONLY, &nvs_handle);
    if (err == ESP_OK)
    {
        size_t ssid_len = sizeof(ssid);
        size_t pass_len = sizeof(password);
        err = nvs_get_str(nvs_handle, "ssid", ssid, &ssid_len);
        if (err != ESP_OK)
        {
            ESP_LOGE(WIFI_TAG, "Error reading SSID from NVS");
            nvs_close(nvs_handle);
            return;
        }
        err = nvs_get_str(nvs_handle, "password", password, &pass_len);
        if (err != ESP_OK)
        {
            ESP_LOGE(WIFI_TAG, "Error reading password from NVS");
            nvs_close(nvs_handle);
            return;
        }
        nvs_close(nvs_handle);
    }

    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_initiation);
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);

    wifi_config_t wifi_configuration = {
        .sta = {
            .ssid = "",
            .password = "",
        }};

    strcpy((char *)wifi_configuration.sta.ssid, ssid);
    strcpy((char *)wifi_configuration.sta.password, password);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_configuration);
    esp_wifi_start();
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_connect();

    ESP_LOGI(WIFI_TAG, "wifi_init_softap finished. SSID:|%s| password:|%s|\n", ssid, password);
}

void wifi_stop(void)
{
    ESP_LOGI(WIFI_TAG, "Stopping WiFi...");

    if (wifi_connected)
    {
        ESP_ERROR_CHECK(esp_wifi_disconnect());
    }

    ESP_ERROR_CHECK(esp_wifi_stop());

    ESP_ERROR_CHECK(esp_wifi_deinit());

    esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler);
    esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler);

    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif)
    {
        esp_netif_destroy(netif);
    }

    wifi_connected = false;

    ESP_LOGI(WIFI_TAG, "WiFi stopped successfully");
}

bool is_wifi_connected()
{
    return wifi_connected;
}
