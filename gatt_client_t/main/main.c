#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_mac.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"

#define GATTC_TAG "GATTC"
#define DEVICE_NAME "iTAG"
#define BATTERY_SERVICE_UUID 0x180F
#define BATTERY_CHAR_UUID 0x2A19
#define ALERT_SERVICE_UUID 0x1802
#define ALERT_CHAR_UUID 0x2A06
#define CLICK_SERVICE_UUID 0xFFE0
#define CLICK_CHAR_UUID 0xFFE1
#define PROFILE_NUM 1
#define PROFILE_A_APP_ID 0
#define INVALID_HANDLE 0

static bool connect = false;
static bool get_server = false;
static esp_gattc_char_elem_t *char_elem_result = NULL;
void alert_test_task(void *pvParameters);
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

struct gattc_profile_inst
{
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t battery_service_start_handle;
    uint16_t battery_service_end_handle;
    uint16_t alert_service_start_handle;
    uint16_t alert_service_end_handle;
    uint16_t click_service_start_handle;
    uint16_t click_service_end_handle;

    uint16_t alert_char_handle;
    uint16_t battery_char_handle;
    uint16_t click_char_handle;
    esp_bd_addr_t remote_bda;
};

static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gattc_cb = gattc_profile_event_handler,
        .gattc_if = ESP_GATT_IF_NONE,
    },
};

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval = 0x100,
    .scan_window = 0x80,
    .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE};

static void set_alert(bool enable)
{
    if (connect)
    {
        uint8_t alert_level = enable ? 1 : 0;
        esp_ble_gattc_write_char(gl_profile_tab[PROFILE_A_APP_ID].gattc_if,
                                 gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                 gl_profile_tab[PROFILE_A_APP_ID].alert_char_handle,
                                 sizeof(alert_level),
                                 &alert_level,
                                 ESP_GATT_WRITE_TYPE_RSP,
                                 ESP_GATT_AUTH_REQ_NONE);
    }
}

static void read_battery_level()
{
    if (connect)
    {
        esp_ble_gattc_read_char(gl_profile_tab[PROFILE_A_APP_ID].gattc_if,
                                gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                gl_profile_tab[PROFILE_A_APP_ID].battery_char_handle,
                                ESP_GATT_AUTH_REQ_NONE);
    }
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        ESP_LOGI(GATTC_TAG, "Scan params set complete, starting scan...");
        esp_ble_gap_start_scanning(30);
        break;

    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTC_TAG, "Scan start failed, error status = %x", param->scan_start_cmpl.status);
        }
        else
        {
            ESP_LOGI(GATTC_TAG, "Scan successfully started, searching for iTAG...");
        }
        break;

    case ESP_GAP_BLE_SCAN_RESULT_EVT:
    {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt)
        {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
        {

            if (scan_result->scan_rst.adv_data_len > 0)
            {
                uint8_t *adv_name = NULL;
                uint8_t adv_name_len = 0;
                adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
                                                    ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);

                if (adv_name != NULL && adv_name_len > 0)
                {
                    char device_name[32] = {0};
                    memcpy(device_name, adv_name, adv_name_len);

                    if (strncmp((char *)adv_name, DEVICE_NAME, strlen(DEVICE_NAME)) == 0)
                    {
                        ESP_LOGI(GATTC_TAG, "Found iTAG! Attempting connection...");
                        ESP_LOGI(GATTC_TAG, "Address type: %d, Address: " ESP_BD_ADDR_STR,
                                 scan_result->scan_rst.ble_addr_type,
                                 ESP_BD_ADDR_HEX(scan_result->scan_rst.bda));

                        if (!connect)
                        {

                            memcpy(gl_profile_tab[PROFILE_A_APP_ID].remote_bda,
                                   scan_result->scan_rst.bda, sizeof(esp_bd_addr_t));

                            esp_err_t scan_stop_err = esp_ble_gap_stop_scanning();
                            if (scan_stop_err != ESP_OK)
                            {
                                ESP_LOGE(GATTC_TAG, "Failed to stop scanning: %d", scan_stop_err);
                            }
                            else
                            {
                                ESP_LOGI(GATTC_TAG, "Scanning stopped successfully");
                            }

                            esp_err_t open_err = esp_ble_gattc_open(
                                gl_profile_tab[PROFILE_A_APP_ID].gattc_if,
                                scan_result->scan_rst.bda,
                                scan_result->scan_rst.ble_addr_type,
                                true);

                            if (open_err != ESP_OK)
                            {
                                ESP_LOGE(GATTC_TAG, "Failed to open connection: %d", open_err);
                                connect = false;
                            }
                            else
                            {
                                ESP_LOGI(GATTC_TAG, "Connection request sent successfully");
                            }
                        }
                        else
                        {
                            ESP_LOGI(GATTC_TAG, "Already trying to connect");
                        }
                    }
                }
            }
            break;
        }
        default:
            break;
        }
        break;
    }
    default:
        break;
    }
}

static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event)
    {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(GATTC_TAG, "REG_EVT, setting scan params...");
        esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
        if (scan_ret)
        {
            ESP_LOGE(GATTC_TAG, "Failed to set scan params, error code = %x", scan_ret);
        }
        break;

    case ESP_GATTC_CONNECT_EVT:
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_CONNECT_EVT");
        gl_profile_tab[PROFILE_A_APP_ID].conn_id = p_data->connect.conn_id;
        memcpy(gl_profile_tab[PROFILE_A_APP_ID].remote_bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
        esp_ble_gattc_search_service(gattc_if, p_data->connect.conn_id, NULL);
        break;

    case ESP_GATTC_SEARCH_RES_EVT:
    {
        if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16)
        {
            if (p_data->search_res.srvc_id.uuid.uuid.uuid16 == BATTERY_SERVICE_UUID)
            {
                ESP_LOGI(GATTC_TAG, "Found battery service");
                gl_profile_tab[PROFILE_A_APP_ID].battery_service_start_handle = p_data->search_res.start_handle;
                gl_profile_tab[PROFILE_A_APP_ID].battery_service_end_handle = p_data->search_res.end_handle;
            }
            else if (p_data->search_res.srvc_id.uuid.uuid.uuid16 == ALERT_SERVICE_UUID)
            {
                ESP_LOGI(GATTC_TAG, "Found alert service");

                gl_profile_tab[PROFILE_A_APP_ID].alert_service_start_handle = p_data->search_res.start_handle;
                gl_profile_tab[PROFILE_A_APP_ID].alert_service_end_handle = p_data->search_res.end_handle;
            }
            else if (p_data->search_res.srvc_id.uuid.uuid.uuid16 == CLICK_SERVICE_UUID)
            {
                ESP_LOGI(GATTC_TAG, "Found click service");
                gl_profile_tab[PROFILE_A_APP_ID].click_service_start_handle = p_data->search_res.start_handle;
                gl_profile_tab[PROFILE_A_APP_ID].click_service_end_handle = p_data->search_res.end_handle;
            }
        }
        break;
    }

    case ESP_GATTC_SEARCH_CMPL_EVT:
    {
        if (p_data->search_cmpl.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTC_TAG, "Search service failed, error status = %x", p_data->search_cmpl.status);
            break;
        }

        uint16_t count = 0;
        esp_gatt_status_t status = esp_ble_gattc_get_attr_count(gattc_if,
                                                                p_data->search_cmpl.conn_id,
                                                                ESP_GATT_DB_CHARACTERISTIC,
                                                                gl_profile_tab[PROFILE_A_APP_ID].battery_service_start_handle,
                                                                gl_profile_tab[PROFILE_A_APP_ID].battery_service_end_handle,
                                                                INVALID_HANDLE,
                                                                &count);
        if (status == ESP_GATT_OK && count > 0)
        {
            char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
            if (char_elem_result)
            {
                status = esp_ble_gattc_get_all_char(gattc_if,
                                                    p_data->search_cmpl.conn_id,
                                                    gl_profile_tab[PROFILE_A_APP_ID].battery_service_start_handle,
                                                    gl_profile_tab[PROFILE_A_APP_ID].battery_service_end_handle,
                                                    char_elem_result,
                                                    &count,
                                                    0);

                if (char_elem_result[0].uuid.uuid.uuid16 == BATTERY_CHAR_UUID)
                {
                    gl_profile_tab[PROFILE_A_APP_ID].battery_char_handle = char_elem_result[0].char_handle;
                    ESP_LOGI(GATTC_TAG, "Found battery characteristic");
                }

                free(char_elem_result);
                char_elem_result = NULL;
            }
        }

        count = 0;
        status = esp_ble_gattc_get_attr_count(gattc_if,
                                              p_data->search_cmpl.conn_id,
                                              ESP_GATT_DB_CHARACTERISTIC,
                                              gl_profile_tab[PROFILE_A_APP_ID].alert_service_start_handle,
                                              gl_profile_tab[PROFILE_A_APP_ID].alert_service_end_handle,
                                              INVALID_HANDLE,
                                              &count);
        if (status == ESP_GATT_OK && count > 0)
        {
            char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
            if (char_elem_result)
            {
                status = esp_ble_gattc_get_all_char(gattc_if,
                                                    p_data->search_cmpl.conn_id,
                                                    gl_profile_tab[PROFILE_A_APP_ID].alert_service_start_handle,
                                                    gl_profile_tab[PROFILE_A_APP_ID].alert_service_end_handle,
                                                    char_elem_result,
                                                    &count,
                                                    0);

                if (char_elem_result[0].uuid.uuid.uuid16 == ALERT_CHAR_UUID)
                {
                    gl_profile_tab[PROFILE_A_APP_ID].alert_char_handle = char_elem_result[0].char_handle;
                    ESP_LOGI(GATTC_TAG, "Found alert characteristic");
                }

                free(char_elem_result);
                char_elem_result = NULL;
            }
        }

        count = 0;
        status = esp_ble_gattc_get_attr_count(gattc_if,
                                              p_data->search_cmpl.conn_id,
                                              ESP_GATT_DB_CHARACTERISTIC,
                                              gl_profile_tab[PROFILE_A_APP_ID].click_service_start_handle,
                                              gl_profile_tab[PROFILE_A_APP_ID].click_service_end_handle,
                                              INVALID_HANDLE,
                                              &count);
        if (status == ESP_GATT_OK && count > 0)
        {
            char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
            if (char_elem_result)
            {
                status = esp_ble_gattc_get_all_char(gattc_if,
                                                    p_data->search_cmpl.conn_id,
                                                    gl_profile_tab[PROFILE_A_APP_ID].click_service_start_handle,
                                                    gl_profile_tab[PROFILE_A_APP_ID].click_service_end_handle,
                                                    char_elem_result,
                                                    &count,
                                                    0);

                if (char_elem_result[0].uuid.uuid.uuid16 == CLICK_CHAR_UUID)
                {
                    gl_profile_tab[PROFILE_A_APP_ID].click_char_handle = char_elem_result[0].char_handle;
                    ESP_LOGI(GATTC_TAG, "Found click characteristic");
                    if (count > 0 && (char_elem_result[0].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY))
                    {
                        gl_profile_tab[PROFILE_A_APP_ID].click_char_handle =
                            char_elem_result[0].char_handle;
                        esp_ble_gattc_register_for_notify(gattc_if,
                                                          gl_profile_tab[PROFILE_A_APP_ID].remote_bda,
                                                          char_elem_result[0].char_handle);
                    }
                }

                free(char_elem_result);
                char_elem_result = NULL;
            }
        }
        connect = true;
        break;
    }

    case ESP_GATTC_READ_CHAR_EVT:
        if (param->read.status == ESP_GATT_OK)
        {
            if (param->read.handle == gl_profile_tab[PROFILE_A_APP_ID].battery_char_handle)
            {
                ESP_LOGI(GATTC_TAG, "Battery level: %d%%", param->read.value[0]);
            }
        }
        break;

    case ESP_GATTC_DISCONNECT_EVT:
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_DISCONNECT_EVT, reason = %d", p_data->disconnect.reason);
        connect = false;
        get_server = false;

        vTaskDelay(pdMS_TO_TICKS(3000));

        ESP_LOGI(GATTC_TAG, "Starting reconnection procedure...");
        esp_ble_gap_start_scanning(30);
        break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT:
    {
        ESP_LOGI(GATTC_TAG, "Registered for notification");
        if (p_data->reg_for_notify.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTC_TAG, "REG FOR NOTIFY failed: error status = %d", p_data->reg_for_notify.status);
        }
        else
        {
            uint16_t count = 0;
            esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count(gattc_if, gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                        ESP_GATT_DB_DESCRIPTOR,
                                                                        gl_profile_tab[PROFILE_A_APP_ID].click_service_start_handle,
                                                                        gl_profile_tab[PROFILE_A_APP_ID].click_service_end_handle,
                                                                        gl_profile_tab[PROFILE_A_APP_ID].click_char_handle, &count);
            if (ret_status != ESP_GATT_OK)
            {
                ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
            }
        }
        break;
    }
    case ESP_GATTC_NOTIFY_EVT:
        if (p_data->notify.is_notify)
        {
            ESP_LOGI(GATTC_TAG, "Clicker has been clicked (notify)");
        }
        break;

    default:
        break;
    }
}

static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    if (event == ESP_GATTC_REG_EVT)
    {
        if (param->reg.status == ESP_GATT_OK)
        {
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        }
        else
        {
            ESP_LOGI(GATTC_TAG, "Reg app failed, app_id %04x, status %d",
                     param->reg.app_id,
                     param->reg.status);
            return;
        }
    }

    do
    {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++)
        {
            if (gattc_if == ESP_GATT_IF_NONE || gattc_if == gl_profile_tab[idx].gattc_if)
            {
                if (gl_profile_tab[idx].gattc_cb)
                {
                    gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
                }
            }
        }
    } while (0);
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

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(GATTC_TAG, "%s initialize controller failed", __func__);
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(GATTC_TAG, "%s enable controller failed", __func__);
        return;
    }

    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(GATTC_TAG, "%s init bluetooth failed", __func__);
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(GATTC_TAG, "%s enable bluetooth failed", __func__);
        return;
    }

    ret = esp_ble_gap_register_callback(esp_gap_cb);
    if (ret)
    {
        ESP_LOGE(GATTC_TAG, "gap register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gattc_register_callback(esp_gattc_cb);
    if (ret)
    {
        ESP_LOGE(GATTC_TAG, "gattc register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gattc_app_register(PROFILE_A_APP_ID);
    if (ret)
    {
        ESP_LOGE(GATTC_TAG, "gattc app register error, error code = %x", ret);
        return;
    }

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret)
    {
        ESP_LOGE(GATTC_TAG, "set local MTU failed, error code = %x", local_mtu_ret);
    }

    xTaskCreate(alert_test_task, "alert_test_task", 2048, NULL, 5, NULL);
    ESP_LOGI(GATTC_TAG, "BLE initialization complete, starting operation...");
}

void alert_test_task(void *pvParameters)
{
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (connect)
        {
            ESP_LOGI(GATTC_TAG, "Writing to Clicker - ON");
            set_alert(true);
            vTaskDelay(pdMS_TO_TICKS(800));

            ESP_LOGI(GATTC_TAG, "Writing to Clicker - OFF");
            set_alert(false);
            vTaskDelay(pdMS_TO_TICKS(3900));

            read_battery_level();
        }
    }
}