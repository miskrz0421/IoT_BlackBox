#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gatt_common_api.h"
#include "driver/gpio.h"
#include "led_strip.h"

#define GATTS_TAG "GATTS"
#define TEST_DEVICE_NAME "BLACKBOX_GATT_SERVER"

#define TEMPERATURE_SERVICE_UUID 0x181A
#define TEMPERATURE_CHAR_UUID 0x2A1C
#define TEMPERATURE_DESCR_UUID 0x2902
#define TEMPERATURE_NUM_HANDLE 4

#define LED_SERVICE_UUID 0x180A
#define LED_CHAR_UUID 0xEEEE
#define LED_NUM_HANDLE 4
// SERVIS
// 3 CHARAKETRUSYUKI
// SSID  - READ/WRITE
// HASLO - READ/WRITE
// USER  - READ/WRITE

#define BLINK_GPIO 48
static led_strip_handle_t led_strip;

static void configure_led(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 1,
    };

    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    led_strip_clear(led_strip);
}

static uint8_t adv_service_uuid128[16] = {
    0xB1,
    0x5E,
    0x8D,
    0x32,
    0x96,
    0x4F,
    0x11,
    0xEE,
    0xBE,
    0x56,
    0x02,
    0x42,
    0xAC,
    0x11,
    0x00,
    0x02};

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

uint16_t gatts_if = ESP_GATT_IF_NONE;
uint16_t app_id = 0;
int conn_id = -1;
static bool connected = false;

typedef struct
{
    uint16_t service_handle;
    uint16_t char_handle;
    uint16_t descr_handle;
} temperature_service_handles_t;

static temperature_service_handles_t temperature_handles = {0};

typedef struct
{
    uint16_t service_handle;
    uint16_t char_handle;
} led_service_handles_t;

static led_service_handles_t led_handles = {0};

static float temperature = 0.0f;

static uint8_t led_state = 0;

static bool temperature_notify_enabled = false;

static void read_temperature_sensor_mock()
{
    temperature = -50.0f + ((float)rand() / (float)RAND_MAX) * (100.0f - (-50.0f));
}
static char temperature_string[8];
static void temperature_notify_task(void *arg)
{
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));

        read_temperature_sensor_mock();

        snprintf(temperature_string, sizeof(temperature_string), "%.2f", temperature);

        uint8_t bytes[8];
        for (int i = 0; i < strlen(temperature_string); i++)
        {
            bytes[i] = (uint8_t)temperature_string[i];
        }

        if (connected && temperature_notify_enabled && temperature_handles.char_handle != 0)
        {
            esp_err_t ret = esp_ble_gatts_send_indicate(
                gatts_if,
                conn_id,
                temperature_handles.char_handle,
                strlen(temperature_string),
                bytes,
                false);
            if (ret != ESP_OK)
            {
                ESP_LOGE(GATTS_TAG, "Failed to send temperature notification, error code = %d", ret);
            }
            else
            {
                ESP_LOGI(GATTS_TAG, "Sent temperature notification: %s", temperature_string);
            }
        }
    }
}

static void set_led_state(uint8_t state)
{
    led_state = state ? 1 : 0;
    if (led_state)
    {
        led_strip_set_pixel(led_strip, 0, 75, 0, 130);
    }
    else
    {
        led_strip_clear(led_strip);
    }
    led_strip_refresh(led_strip);
    ESP_LOGI(GATTS_TAG, "LED has been turned %s", led_state ? "ON" : "OFF");
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        ESP_LOGI(GATTS_TAG, "Starting advertising...");
        break;
    default:
        break;
    }
}

static void esp_gatts_cb(esp_gatts_cb_event_t event, esp_gatt_if_t interface,
                         esp_ble_gatts_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GATTS_REG_EVT:
        if (param->reg.status == ESP_GATT_OK)
        {
            ESP_LOGI(GATTS_TAG, "Successfully registered app");
            gatts_if = interface;
        }
        else
        {
            ESP_LOGE(GATTS_TAG, "Failed to register app, status %d", param->reg.status);
            break;
        }

        ESP_ERROR_CHECK(esp_ble_gap_set_device_name(TEST_DEVICE_NAME));
        ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&adv_data));
        ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&scan_rsp_data));
        ESP_LOGI(GATTS_TAG, "Set device name and configured advertising data");

        esp_gatt_srvc_id_t temperature_service_id = {
            .is_primary = true,
            .id = {
                .inst_id = 0x00,
                .uuid = {
                    .len = ESP_UUID_LEN_16,
                    .uuid = {.uuid16 = TEMPERATURE_SERVICE_UUID},
                },
            },
        };

        ESP_ERROR_CHECK(esp_ble_gatts_create_service(gatts_if, &temperature_service_id, TEMPERATURE_NUM_HANDLE));
        ESP_LOGI(GATTS_TAG, "Created temperature service");

        esp_gatt_srvc_id_t led_service_id = {
            .is_primary = true,
            .id = {
                .inst_id = 0x01,
                .uuid = {
                    .len = ESP_UUID_LEN_16,
                    .uuid = {.uuid16 = LED_SERVICE_UUID},
                },
            },
        };

        ESP_ERROR_CHECK(esp_ble_gatts_create_service(gatts_if, &led_service_id, LED_NUM_HANDLE));
        ESP_LOGI(GATTS_TAG, "Created LED service");
        break;

    case ESP_GATTS_CREATE_EVT:
        if (param->create.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTS_TAG, "Could not create service");
            break;
        }
        uint16_t service_uuid = param->create.service_id.id.uuid.uuid.uuid16;
        ESP_LOGI(GATTS_TAG, "Service created with UUID 0x%04X", service_uuid);

        if (service_uuid == TEMPERATURE_SERVICE_UUID)
        {
            temperature_handles.service_handle = param->create.service_handle;
            ESP_ERROR_CHECK(esp_ble_gatts_start_service(temperature_handles.service_handle));

            esp_bt_uuid_t temperature_char_uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid = {.uuid16 = TEMPERATURE_CHAR_UUID},
            };

            esp_attr_value_t temperature_char_val = {
                .attr_max_len = sizeof(temperature),
                .attr_len = sizeof(temperature),
                .attr_value = (uint8_t *)&temperature,
            };

            ESP_ERROR_CHECK(esp_ble_gatts_add_char(
                temperature_handles.service_handle,
                &temperature_char_uuid,
                ESP_GATT_PERM_READ,
                ESP_GATT_CHAR_PROP_BIT_NOTIFY | ESP_GATT_CHAR_PROP_BIT_READ,
                &temperature_char_val,
                NULL));
            ESP_LOGI(GATTS_TAG, "Added temperature characteristic");
        }
        else if (service_uuid == LED_SERVICE_UUID)
        {
            led_handles.service_handle = param->create.service_handle;
            ESP_ERROR_CHECK(esp_ble_gatts_start_service(led_handles.service_handle));

            esp_bt_uuid_t led_char_uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid = {.uuid16 = LED_CHAR_UUID},
            };

            esp_attr_value_t led_char_val = {
                .attr_max_len = sizeof(led_state),
                .attr_len = sizeof(led_state),
                .attr_value = &led_state,
            };

            ESP_ERROR_CHECK(esp_ble_gatts_add_char(
                led_handles.service_handle,
                &led_char_uuid,
                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE,
                &led_char_val,
                NULL));
            ESP_LOGI(GATTS_TAG, "Added LED characteristic");
        }
        break;

    case ESP_GATTS_ADD_CHAR_EVT:
        if (param->add_char.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTS_TAG, "Could not add characteristic");
            break;
        }
        uint16_t char_uuid = param->add_char.char_uuid.uuid.uuid16;
        uint16_t service_handle = param->add_char.service_handle;
        ESP_LOGI(GATTS_TAG, "Characteristic added with UUID 0x%04X", char_uuid);

        if (char_uuid == TEMPERATURE_CHAR_UUID)
        {
            temperature_handles.char_handle = param->add_char.attr_handle;

            esp_bt_uuid_t temperature_descr_uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid = {.uuid16 = TEMPERATURE_DESCR_UUID},
            };

            ESP_ERROR_CHECK(esp_ble_gatts_add_char_descr(
                service_handle,
                &temperature_descr_uuid,
                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                NULL,
                NULL));
            ESP_LOGI(GATTS_TAG, "Added temperature characteristic descriptor");
        }
        else if (char_uuid == LED_CHAR_UUID)
        {
            led_handles.char_handle = param->add_char.attr_handle;
        }
        break;

    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        if (param->add_char_descr.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTS_TAG, "Could not add descriptor");
            break;
        }
        uint16_t descr_uuid = param->add_char_descr.descr_uuid.uuid.uuid16;
        ESP_LOGI(GATTS_TAG, "Descriptor added with UUID 0x%04X", descr_uuid);

        if (descr_uuid == TEMPERATURE_DESCR_UUID)
        {
            temperature_handles.descr_handle = param->add_char_descr.attr_handle;
        }
        break;

    case ESP_GATTS_START_EVT:
        ESP_LOGI(GATTS_TAG, "Service started, status %d", param->start.status);
        break;

    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(GATTS_TAG, "Connected to client, conn_id = %d", param->connect.conn_id);
        conn_id = param->connect.conn_id;
        connected = true;
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(GATTS_TAG, "Disconnected from client, conn_id = %d", param->disconnect.conn_id);
        conn_id = -1;
        connected = false;
        temperature_notify_enabled = false;
        esp_ble_gap_start_advertising(&adv_params);
        break;

    case ESP_GATTS_READ_EVT:
        ESP_LOGI(GATTS_TAG, "Read request on handle %d", param->read.handle);
        if (param->read.handle == led_handles.char_handle)
        {
            esp_gatt_rsp_t rsp;
            memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
            rsp.attr_value.handle = param->read.handle;
            rsp.attr_value.len = sizeof(led_state);
            rsp.attr_value.value[0] = led_state;
            ESP_ERROR_CHECK(esp_ble_gatts_send_response(
                interface,
                param->read.conn_id,
                param->read.trans_id,
                ESP_GATT_OK,
                &rsp));
            ESP_LOGI(GATTS_TAG, "LED state sent back: %d", led_state);
        }
        else if (param->read.handle == temperature_handles.char_handle)
        {
            snprintf(temperature_string, sizeof(temperature_string), "%.2f", temperature);

            uint8_t bytes[8];
            for (int i = 0; i < strlen(temperature_string); i++)
            {
                bytes[i] = (uint8_t)temperature_string[i];
            }
            esp_gatt_rsp_t rsp;
            memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
            rsp.attr_value.handle = param->read.handle;
            rsp.attr_value.len = strlen(temperature_string);
            memcpy(rsp.attr_value.value, bytes, strlen(temperature_string));
            ESP_ERROR_CHECK(esp_ble_gatts_send_response(
                interface,
                param->read.conn_id,
                param->read.trans_id,
                ESP_GATT_OK,
                &rsp));
            ESP_LOGI(GATTS_TAG, "Sent temperature: %s", temperature_string);
        }
        else if (param->read.handle == temperature_handles.descr_handle)
        {

            esp_gatt_rsp_t rsp;
            memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
            rsp.attr_value.handle = param->read.handle;
            rsp.attr_value.len = 2;
            uint16_t cccd_value = temperature_notify_enabled ? 0x0001 : 0x0000;
            rsp.attr_value.value[0] = cccd_value & 0xFF;
            rsp.attr_value.value[1] = (cccd_value >> 8) & 0xFF;
            ESP_ERROR_CHECK(esp_ble_gatts_send_response(
                interface,
                param->read.conn_id,
                param->read.trans_id,
                ESP_GATT_OK,
                &rsp));
            ESP_LOGI(GATTS_TAG, "CCCD value sent: 0x%04X", cccd_value);
        }
        else
        {
            ESP_LOGW(GATTS_TAG, "Read request on unknown handle %d", param->read.handle);
            esp_ble_gatts_send_response(
                interface,
                param->read.conn_id,
                param->read.trans_id,
                ESP_GATT_READ_NOT_PERMIT,
                NULL);
        }
        break;

    case ESP_GATTS_WRITE_EVT:
        ESP_LOGI(GATTS_TAG, "Write request on handle %d", param->write.handle);

        if (param->write.handle == led_handles.char_handle)
        {
            if (param->write.len == 1)
            {
                set_led_state(param->write.value[0]);
            }
            esp_ble_gatts_send_response(
                interface,
                param->write.conn_id,
                param->write.trans_id,
                ESP_GATT_OK,
                NULL);
        }
        else if (param->write.handle == temperature_handles.descr_handle)
        {
            uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
            if (descr_value == 0x0001)
            {
                ESP_LOGI(GATTS_TAG, "Client subscribed to temperature notifications");
                temperature_notify_enabled = true;
            }
            else if (descr_value == 0x0000)
            {
                ESP_LOGI(GATTS_TAG, "Client unsubscribed from temperature notifications");
                temperature_notify_enabled = false;
            }
            esp_ble_gatts_send_response(
                interface,
                param->write.conn_id,
                param->write.trans_id,
                ESP_GATT_OK,
                NULL);
        }
        else
        {
            ESP_LOGW(GATTS_TAG, "Write to unknown handle %d", param->write.handle);
            esp_ble_gatts_send_response(
                interface,
                param->write.conn_id,
                param->write.trans_id,
                ESP_GATT_OK,
                NULL);
        }
        break;

    default:
        break;
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    configure_led();

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(esp_gatts_cb));
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(esp_gap_cb));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(0));
    ESP_ERROR_CHECK(esp_ble_gatt_set_local_mtu(200));

    xTaskCreate(temperature_notify_task, "temperature_notify_task", 4096, NULL, 5, NULL);
}