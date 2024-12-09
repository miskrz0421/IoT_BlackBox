#include "ble.h"
#include "wifi.h"

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
#include "esp_mac.h"
#include "esp_timer.h"

#define GATTS_TAG "GATTS"
#define TEST_DEVICE_NAME "BLACKBOX_GATT_SERVER"

#define CONFIG_SERVICE_UUID 0xFF00
#define SSID_CHAR_UUID 0xFF01
#define PASSWORD_CHAR_UUID 0xFF02
#define USER_CHAR_UUID 0xFF03
#define MAX_STRING_LENGTH 32
#define CONFIG_NUM_HANDLE 7
void ble_toggle(bool enable);
static bool enabled = false;

static uint8_t adv_service_uuid128[16] = {
    0xB1, 0x5E, 0x8D, 0x32, 0x96, 0x4F, 0x11, 0xEE,
    0xBE, 0x56, 0x02, 0x42, 0xAC, 0x11, 0x00, 0x02};

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
    uint16_t ssid_handle;
    uint16_t password_handle;
    uint16_t user_handle;
} config_service_handles_t;

static config_service_handles_t config_handles = {0};

static char ssid_value[MAX_STRING_LENGTH] = "";
static char password_value[MAX_STRING_LENGTH] = "";
static char user_value[MAX_STRING_LENGTH] = "";

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
static void handle_read_request(esp_gatt_if_t interface, esp_ble_gatts_cb_param_t *param)
{
    esp_gatt_rsp_t rsp;
    memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
    rsp.attr_value.handle = param->read.handle;

    if (param->read.handle == config_handles.ssid_handle)
    {
        rsp.attr_value.len = strlen(ssid_value);
        memcpy(rsp.attr_value.value, ssid_value, rsp.attr_value.len);
        ESP_LOGI(GATTS_TAG, "Read SSID: %s", ssid_value);
    }
    else if (param->read.handle == config_handles.password_handle)
    {
        rsp.attr_value.len = strlen(password_value);
        memcpy(rsp.attr_value.value, password_value, rsp.attr_value.len);
        ESP_LOGI(GATTS_TAG, "Read Password: %s", password_value);
    }
    else if (param->read.handle == config_handles.user_handle)
    {

        nvs_handle_t nvs_handle;
        esp_err_t err = nvs_open("config", NVS_READWRITE, &nvs_handle);
        if (err == ESP_OK)
        {
            size_t required_size = MAX_STRING_LENGTH;
            char temp_user[MAX_STRING_LENGTH] = {0};
            err = nvs_get_str(nvs_handle, "user", temp_user, &required_size);
            if (err == ESP_OK)
            {
                rsp.attr_value.len = strlen(temp_user);
                memcpy(rsp.attr_value.value, temp_user, rsp.attr_value.len);
                ESP_LOGI(GATTS_TAG, "Read User from NVS: %s", temp_user);
            }
            else
            {
                rsp.attr_value.len = strlen(user_value);
                memcpy(rsp.attr_value.value, user_value, rsp.attr_value.len);
                ESP_LOGI(GATTS_TAG, "Failed to read from NVS, using cached value: %s", user_value);
            }
            nvs_close(nvs_handle);
        }
        else
        {
            rsp.attr_value.len = strlen(user_value);
            memcpy(rsp.attr_value.value, user_value, rsp.attr_value.len);
            ESP_LOGE(GATTS_TAG, "Error opening NVS handle, using cached value: %s", user_value);
        }
    }
    else
    {
        ESP_LOGW(GATTS_TAG, "Read from unknown handle %d", param->read.handle);
        esp_ble_gatts_send_response(interface, param->read.conn_id, param->read.trans_id,
                                    ESP_GATT_READ_NOT_PERMIT, NULL);
        return;
    }

    ESP_ERROR_CHECK(esp_ble_gatts_send_response(interface, param->read.conn_id,
                                                param->read.trans_id, ESP_GATT_OK, &rsp));
}

static void handle_write_request(esp_gatt_if_t interface, esp_ble_gatts_cb_param_t *param)
{
    if (param->write.len > MAX_STRING_LENGTH - 1)
    {
        ESP_LOGW(GATTS_TAG, "Write value too long");
        esp_ble_gatts_send_response(interface, param->write.conn_id, param->write.trans_id,
                                    ESP_GATT_INVALID_ATTR_LEN, NULL);
        return;
    }

    char temp_value[MAX_STRING_LENGTH] = {0};
    memcpy(temp_value, param->write.value, param->write.len);

    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("config", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(GATTS_TAG, "Error opening NVS handle");
        return;
    }

    if (param->write.handle == config_handles.ssid_handle)
    {
        strncpy(ssid_value, temp_value, MAX_STRING_LENGTH - 1);
        ESP_LOGI(GATTS_TAG, "Write SSID: %s", ssid_value);

        err = nvs_set_str(nvs_handle, "ssid", ssid_value);
        if (err != ESP_OK)
        {
            ESP_LOGE(GATTS_TAG, "Error saving SSID to NVS");
        }
    }
    else if (param->write.handle == config_handles.password_handle)
    {
        strncpy(password_value, temp_value, MAX_STRING_LENGTH - 1);
        ESP_LOGI(GATTS_TAG, "Write Password: %s", password_value);

        err = nvs_set_str(nvs_handle, "password", password_value);
        if (err != ESP_OK)
        {
            ESP_LOGE(GATTS_TAG, "Error saving password to NVS");
        }
    }
    else if (param->write.handle == config_handles.user_handle)
    {
        strncpy(user_value, temp_value, MAX_STRING_LENGTH - 1);
        ESP_LOGI(GATTS_TAG, "Write User: %s", user_value);

        err = nvs_set_str(nvs_handle, "user", user_value);
        if (err != ESP_OK)
        {
            ESP_LOGE(GATTS_TAG, "Error saving user to NVS");
        }
    }
    else
    {
        ESP_LOGW(GATTS_TAG, "Write to unknown handle %d", param->write.handle);
    }

    err = nvs_commit(nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(GATTS_TAG, "Error committing NVS changes");
    }
    nvs_close(nvs_handle);

    esp_ble_gatts_send_response(interface, param->write.conn_id,
                                param->write.trans_id, ESP_GATT_OK, NULL);
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

        esp_gatt_srvc_id_t config_service_id = {
            .is_primary = true,
            .id = {
                .inst_id = 0x00,
                .uuid = {
                    .len = ESP_UUID_LEN_16,
                    .uuid = {.uuid16 = CONFIG_SERVICE_UUID},
                },
            },
        };

        ESP_ERROR_CHECK(esp_ble_gatts_create_service(gatts_if, &config_service_id, CONFIG_NUM_HANDLE));
        break;

    case ESP_GATTS_CREATE_EVT:
        if (param->create.status == ESP_GATT_OK)
        {
            config_handles.service_handle = param->create.service_handle;
            ESP_ERROR_CHECK(esp_ble_gatts_start_service(config_handles.service_handle));

            esp_bt_uuid_t ssid_uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid = {.uuid16 = SSID_CHAR_UUID},
            };
            ESP_ERROR_CHECK(esp_ble_gatts_add_char(
                config_handles.service_handle,
                &ssid_uuid,
                ESP_GATT_PERM_WRITE,
                ESP_GATT_CHAR_PROP_BIT_WRITE,
                NULL,
                NULL));
        }
        break;

    case ESP_GATTS_ADD_CHAR_EVT:
        if (param->add_char.status == ESP_GATT_OK)
        {
            uint16_t char_uuid = param->add_char.char_uuid.uuid.uuid16;

            if (char_uuid == SSID_CHAR_UUID)
            {
                config_handles.ssid_handle = param->add_char.attr_handle;

                esp_bt_uuid_t password_uuid = {
                    .len = ESP_UUID_LEN_16,
                    .uuid = {.uuid16 = PASSWORD_CHAR_UUID},
                };
                ESP_ERROR_CHECK(esp_ble_gatts_add_char(
                    config_handles.service_handle,
                    &password_uuid,
                    ESP_GATT_PERM_WRITE,
                    ESP_GATT_CHAR_PROP_BIT_WRITE,
                    NULL,
                    NULL));
            }
            else if (char_uuid == PASSWORD_CHAR_UUID)
            {
                config_handles.password_handle = param->add_char.attr_handle;

                esp_bt_uuid_t user_uuid = {
                    .len = ESP_UUID_LEN_16,
                    .uuid = {.uuid16 = USER_CHAR_UUID},
                };
                ESP_ERROR_CHECK(esp_ble_gatts_add_char(
                    config_handles.service_handle,
                    &user_uuid,
                    ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                    ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE,
                    NULL,
                    NULL));
            }
            else if (char_uuid == USER_CHAR_UUID)
            {
                config_handles.user_handle = param->add_char.attr_handle;
            }
        }
        break;

    case ESP_GATTS_START_EVT:
        ESP_LOGI(GATTS_TAG, "Service started");
        break;

    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(GATTS_TAG, "Client connected");
        conn_id = param->connect.conn_id;
        connected = true;
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(GATTS_TAG, "Client disconnected");
        conn_id = -1;
        connected = false;
        vTaskDelay(pdMS_TO_TICKS(50));
        wifi_connection();
        break;

    case ESP_GATTS_READ_EVT:
        handle_read_request(interface, param);
        break;

    case ESP_GATTS_WRITE_EVT:
        handle_write_request(interface, param);
        break;

    default:
        break;
    }
}
bool ble_enabled()
{
    return enabled;
}

static bool ble_initialized = false;
static int first = 0;
void ble_init_once(void)
{
    if (!ble_initialized)
    {
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
        first++;
        ble_initialized = true;
        ESP_LOGI(GATTS_TAG, "BLE initialized");
        enabled = true;
    }
}

void ble_toggle(bool enable)
{
    if (!ble_initialized)
    {
        ble_init_once();
        return;
    }

    if (enable)
    {
        esp_ble_gap_start_advertising(&adv_params);
        enabled = true;
    }
    else
    {
        esp_ble_gap_stop_advertising();
        enabled = false;
    }
}