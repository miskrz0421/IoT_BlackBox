#include <stdio.h>
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "freertos/FreeRTOS.h"
#include "nvs_flash.h"
#include "esp_gatt_common_api.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "nvs.h"
#include "esp_gatt_defs.h"
#include "freertos/semphr.h"

#define GATTC_TAG "BLE_CLIENT"
#define PROFILE_NUM      1
#define PROFILE_APP_ID   0

// Te wartości musisz podmienić na te z Twojego klikera
#define TARGET_SERVER_NAME "ESP32_BLE_SERVER"  
#define TARGET_DEVICE_NAME "iTAG"  
#define REMOTE_SERVICE_UUID 0x180F        
#define REMOTE_CHAR_UUID    0x2A19        
int get_data_from_td = 0;

// Semafor
static SemaphoreHandle_t ble_semaphore = NULL;

// Deklaracje funkcji
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void esp_gap_cb_td(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void esp_gap_cb_s(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

static bool connect = false;
static bool get_server = false;
static esp_gattc_char_elem_t *char_elem_result = NULL;
static esp_gatt_if_t gattc_if_for_register = 0;

// Struktura profilu
struct gattc_profile_inst {
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handle;
    esp_bd_addr_t remote_bda;
};

// Profil
static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_ID] = {
        .gattc_cb = gattc_profile_event_handler,
        .gattc_if = ESP_GATT_IF_NONE,
    },
};

// Parametry skanowania
static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type         = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy    = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval        = 0x50,
    .scan_window          = 0x30,
    .scan_duplicate       = BLE_SCAN_DUPLICATE_DISABLE
};

// Handler zdarzeń GAP (skanowanie iTAG)
static void esp_gap_cb_td(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
            esp_ble_gap_start_scanning(30);
            break;
        }
        
        case ESP_GAP_BLE_SCAN_RESULT_EVT: {
            esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
            switch (scan_result->scan_rst.search_evt) {
                case ESP_GAP_SEARCH_INQ_RES_EVT: {
                    uint8_t *adv_name = NULL;
                    uint8_t adv_name_len = 0;
                    
                    adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
                                                       ESP_BLE_AD_TYPE_NAME_CMPL,
                                                       &adv_name_len);
                    
                    if (adv_name != NULL) {
                        ESP_LOGI(GATTC_TAG, "Znalezione urządzenie:%s", (char*)adv_name);
                        ESP_LOGI(GATTC_TAG, "TARGET_DEVICE_NAME:%s", TARGET_DEVICE_NAME);
                        ESP_LOGI(GATTC_TAG, "bool  %d", strcmp((char *)adv_name, TARGET_DEVICE_NAME));
                        
                        if (strncmp((char*)adv_name, TARGET_DEVICE_NAME,strlen(TARGET_DEVICE_NAME))==0) {
                            get_data_from_td = 1;
                            ESP_LOGI(GATTC_TAG, "Znaleziono iTAG!");
                            if (!connect) {
                                connect = true;
                                ESP_LOGI(GATTC_TAG, "Łączenie z urządzeniem...");
                                esp_ble_gap_stop_scanning();
                                esp_ble_gattc_open(gl_profile_tab[PROFILE_APP_ID].gattc_if,
                                                 scan_result->scan_rst.bda,
                                                 scan_result->scan_rst.ble_addr_type,
                                                 true);
                                // Daj sygnał przez semafor
                                xSemaphoreGive(ble_semaphore);
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

// Handler zdarzeń GAP (skanowanie serwera)
static void esp_gap_cb_s(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
            esp_ble_gap_start_scanning(30);
            break;
        }
        
        case ESP_GAP_BLE_SCAN_RESULT_EVT: {
            esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
            switch (scan_result->scan_rst.search_evt) {
                case ESP_GAP_SEARCH_INQ_RES_EVT: {
                    uint8_t *adv_name = NULL;
                    uint8_t adv_name_len = 0;
                    
                    adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
                                                       ESP_BLE_AD_TYPE_NAME_CMPL,
                                                       &adv_name_len);
                    
                    if (adv_name != NULL && get_data_from_td == 1) {
                        ESP_LOGI(GATTC_TAG, "Znaleziony serwer:%s", (char*)adv_name);
                        ESP_LOGI(GATTC_TAG, "TARGET_SERVER_NAME:%s", TARGET_SERVER_NAME);
                        ESP_LOGI(GATTC_TAG, "bool  %d", strcmp((char *)adv_name, TARGET_SERVER_NAME));

                        if (strncmp((char*)adv_name, TARGET_SERVER_NAME,strlen(TARGET_SERVER_NAME))==0) {
                            ESP_LOGI(GATTC_TAG, "Znaleziono serwer!");
                            if (!connect) {
                                connect = true;
                                ESP_LOGI(GATTC_TAG, "Łączenie z serwerem...");
                                esp_ble_gap_stop_scanning();
                                esp_ble_gattc_open(gl_profile_tab[PROFILE_APP_ID].gattc_if,
                                                 scan_result->scan_rst.bda,
                                                 scan_result->scan_rst.ble_addr_type,
                                                 true);
                            }
                        }
                        get_data_from_td = 0;
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

// Handler zdarzeń GATTC
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    switch (event) {
        case ESP_GATTC_REG_EVT:
            ESP_LOGI(GATTC_TAG, "REG_EVT");
            esp_ble_gap_set_scan_params(&ble_scan_params);
            break;

        case ESP_GATTC_CONNECT_EVT:
            ESP_LOGI(GATTC_TAG, "ESP_GATTC_CONNECT_EVT");
            esp_ble_gattc_search_service(gattc_if, param->connect.conn_id, NULL);
            break;

        case ESP_GATTC_SEARCH_RES_EVT: {
            ESP_LOGI(GATTC_TAG, "SEARCH_RES_EVT");
            if (param->search_res.srvc_id.uuid.uuid.uuid16 == REMOTE_SERVICE_UUID) {
                ESP_LOGI(GATTC_TAG, "Znaleziono serwis");
                get_server = true;
                gl_profile_tab[PROFILE_APP_ID].service_start_handle = param->search_res.start_handle;
                gl_profile_tab[PROFILE_APP_ID].service_end_handle = param->search_res.end_handle;
            }
            break;
        }

        case ESP_GATTC_SEARCH_CMPL_EVT:
            if (get_server) {
                uint16_t count = 0;
                esp_gatt_status_t status = esp_ble_gattc_get_attr_count(gattc_if,
                                                                       param->search_cmpl.conn_id,
                                                                       ESP_GATT_DB_CHARACTERISTIC,
                                                                       gl_profile_tab[PROFILE_APP_ID].service_start_handle,
                                                                       gl_profile_tab[PROFILE_APP_ID].service_end_handle,
                                                                       0,
                                                                       &count);
                
                if (status == ESP_GATT_OK && count > 0) {
                    char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                    if (char_elem_result) {
                        status = esp_ble_gattc_get_all_char(gattc_if,
                                                          param->search_cmpl.conn_id,
                                                          gl_profile_tab[PROFILE_APP_ID].service_start_handle,
                                                          gl_profile_tab[PROFILE_APP_ID].service_end_handle,
                                                          char_elem_result,
                                                          &count,
                                                          0);
                        
                        if (status == ESP_GATT_OK) {
                            for (int i = 0; i < count; i++) {
                                if (char_elem_result[i].uuid.uuid.uuid16 == REMOTE_CHAR_UUID) {
                                    ESP_LOGI(GATTC_TAG, "Znaleziono charakterystykę, odczytuję dane");
                                    esp_ble_gattc_read_char(gattc_if,
                                                          param->search_cmpl.conn_id,
                                                          char_elem_result[i].char_handle,
                                                          ESP_GATT_AUTH_REQ_NONE);
                                    break;
                                }
                            }
                        }
                        free(char_elem_result);
                    }
                }
            }
            break;

        case ESP_GATTC_READ_CHAR_EVT:
            if (param->read.status == ESP_GATT_OK) {
                ESP_LOGI(GATTC_TAG, "Odczytano dane (hex):");
                ESP_LOG_BUFFER_HEX(GATTC_TAG, param->read.value, param->read.value_len);
                
                printf("Dane jako liczby: ");
                for(int i = 0; i < param->read.value_len; i++) {
                    printf("%d ", param->read.value[i]);
                }
                printf("\n");
                
                printf("Dane jako tekst: %.*s\n", param->read.value_len, (char*)param->read.value);
            }
            break;

        default:
            break;
    }
}

// Główny callback GATTC
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    if (event == ESP_GATTC_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        } else {
            ESP_LOGI(GATTC_TAG, "Reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gattc_if == ESP_GATT_IF_NONE || 
                    gattc_if == gl_profile_tab[idx].gattc_if) {
                if (gl_profile_tab[idx].gattc_cb) {
                    gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
                }
            }
        }
    } while (0);
}

void app_main(void)
{
    // Inicjalizacja NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Utworzenie semafora binarnego
    ble_semaphore = xSemaphoreCreateBinary();
    if (ble_semaphore == NULL) {
        ESP_LOGE(GATTC_TAG, "Nie można utworzyć semafora!");
        return;
    }

    // Inicjalizacja BLE
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    // Pierwszy callback (dla iTAG)
    ESP_ERROR_CHECK(esp_ble_gattc_register_callback(esp_gattc_cb));
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(esp_gap_cb_td));
    ESP_ERROR_CHECK(esp_ble_gattc_app_register(PROFILE_APP_ID));

    // Czekaj na znalezienie iTAG
    if (xSemaphoreTake(ble_semaphore, pdMS_TO_TICKS(30000)) == pdTRUE) {
        ESP_LOGI(GATTC_TAG, "Znaleziono iTAG, rozpoczynam skanowanie serwera");
        
        // Zatrzymaj skanowanie i poczekaj chwilę
        // esp_ble_gap_stop_scanning();
        vTaskDelay(pdMS_TO_TICKS(100));

        // Reset zmiennej connect przed drugim skanowaniem
        connect = false;

        // Drugi callback (dla serwera)
        ESP_ERROR_CHECK(esp_ble_gattc_register_callback(esp_gattc_cb));
        ESP_ERROR_CHECK(esp_ble_gap_register_callback(esp_gap_cb_s));
        ESP_ERROR_CHECK(esp_ble_gattc_app_register(PROFILE_APP_ID));
    } else {
        ESP_LOGE(GATTC_TAG, "Timeout - nie znaleziono iTAG");
    }

    // Ustaw MTU
    ESP_ERROR_CHECK(esp_ble_gatt_set_local_mtu(200));
}