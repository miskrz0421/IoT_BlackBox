#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/gpio.h" // Nagłówek dla obsługi GPIO
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#define WIFI_SSID "testtest"
#define WIFI_PASS "testtest"
#define LED_PIN GPIO_NUM_2  // Dioda LED na pinie GPIO 2 (standardowa dla ESP32)

int retry_num = 0;
bool wifi_connected = false;  // Flaga oznaczająca stan połączenia Wi-Fi

// Tworzymy zmienną semafora binarnego
SemaphoreHandle_t wifi_semaphore;

// Task do mrugania diodą LED
void led_blink_task(void *pvParameter) {
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    while (1) {
        if (!wifi_connected) {  // Mrugamy, gdy Wi-Fi jest rozłączone
            gpio_set_level(LED_PIN, 1);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            gpio_set_level(LED_PIN, 0);
            vTaskDelay(500 / portTICK_PERIOD_MS);
        } else {  // Gdy Wi-Fi jest połączone, wyłączamy diodę
            gpio_set_level(LED_PIN, 0);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
}

static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_id == WIFI_EVENT_STA_START) {
        printf("WIFI CONNECTING....\n");
    } else if (event_id == WIFI_EVENT_STA_CONNECTED) {
        printf("WiFi CONNECTED\n");
        retry_num = 0;
        wifi_connected = true;  // Ustawienie flagi na true, gdy Wi-Fi jest połączone
    } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (retry_num == 0) {
            printf("WiFi lost connection\n");
        }
        
        wifi_connected = false;  // Ustawienie flagi na false, gdy Wi-Fi jest rozłączone
        esp_wifi_connect();  // Ponawiamy próbę połączenia
        retry_num++;
        printf("Retrying to Connect...(%d)\n", retry_num);
    } else if (event_id == IP_EVENT_STA_GOT_IP) {
        printf("WiFi got IP...\n\n");
        retry_num = 0;
        wifi_connected = true;  // Aktualizacja flagi po uzyskaniu IP
        
        // Zwolnienie semafora po uzyskaniu IP
        xSemaphoreGive(wifi_semaphore);
    }
}

void wifi_connection() {
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
        }
    };
    strcpy((char*)wifi_configuration.sta.ssid, WIFI_SSID);
    strcpy((char*)wifi_configuration.sta.password, WIFI_PASS);
    
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_configuration);
    esp_wifi_start();
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_connect();
    
    printf("wifi_init_softap finished. SSID:%s  password:%s\n", WIFI_SSID, WIFI_PASS);
}

void app_main(void) {
    // Inicjalizacja NVS
    nvs_flash_init();

    // Inicjalizacja semafora binarnego
    wifi_semaphore = xSemaphoreCreateBinary();

    // Uruchamiamy połączenie Wi-Fi
    wifi_connection();

    // Tworzymy task do mrugania diodą LED
    xTaskCreate(&led_blink_task, "led_blink_task", 1024, NULL, 1, NULL);

    // Oczekiwanie na semafor przed kontynuowaniem
    if (xSemaphoreTake(wifi_semaphore, portMAX_DELAY) == pdTRUE) {
        printf("Skończono konfigurowanie wifi. Kontynuowanie programu głównego...\n");
    }

    printf("Hello World\n");
    for (int i = 10; i >= 0; i--) {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}