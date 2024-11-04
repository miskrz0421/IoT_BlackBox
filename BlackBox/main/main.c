#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

#define WIFI_SSID "testtest"
#define WIFI_PASS "testtest"
#define LED_PIN GPIO_NUM_2
#define MAX_HTTP_RESPONSE_SIZE 2048
#define HTTP_SERVER "example.com"

int retry_num = 0;
bool wifi_connected = false;
bool isFirstLoop = true;
SemaphoreHandle_t http_semaphore = NULL;  // Semafor do kontroli wykonania HTTP request

void led_blink_task(void *pvParameter) {
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    while (1) {
        if (!wifi_connected) {
            gpio_set_level(LED_PIN, 1);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            gpio_set_level(LED_PIN, 0);
            vTaskDelay(500 / portTICK_PERIOD_MS);
        } else {
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
        isFirstLoop = false;
        retry_num = 0;
        wifi_connected = true;
    } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (retry_num == 0 && !isFirstLoop) {
            printf("WiFi lost connection\n");
        }
        wifi_connected = false;
        esp_wifi_connect();
        retry_num++;
        printf("Retrying to Connect...(%d)\n", retry_num);
    } else if (event_id == IP_EVENT_STA_GOT_IP) {
        printf("WiFi got IP...\n\n");
        retry_num = 0;
        wifi_connected = true;
        // Po uzyskaniu IP dajemy semafor, aby wykonać request HTTP
        xSemaphoreGive(http_semaphore);
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

void http_get_task(void *pvParameters) {
    while (1) {
        // Czekamy na semafor - zostanie on dany po uzyskaniu IP
        if (xSemaphoreTake(http_semaphore, portMAX_DELAY) == pdTRUE) {
            struct sockaddr_in server_addr;
            struct hostent *hp;
            
            int sock = socket(AF_INET, SOCK_STREAM, 0);
            if (sock < 0) {
                printf("Nie można utworzyć gniazda\n");
                continue;
            }

            server_addr.sin_family = AF_INET;
            server_addr.sin_port = htons(80);
            hp = gethostbyname(HTTP_SERVER);
            if (hp == NULL) {
                printf("Nie można znaleźć hosta\n");
                close(sock);
                continue;
            }
            memcpy(&server_addr.sin_addr.s_addr, hp->h_addr, hp->h_length);

            if (connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) != 0) {
                printf("Błąd połączenia z serwerem\n");
                close(sock);
                continue;
            }

            char request[128];
            snprintf(request, sizeof(request),
                    "GET / HTTP/1.1\r\n"
                    "Host: %s\r\n"
                    "Connection: close\r\n\r\n",
                    HTTP_SERVER);

            if (send(sock, request, strlen(request), 0) < 0) {
                printf("Błąd wysyłania zapytania\n");
                close(sock);
                continue;
            }

            char response[MAX_HTTP_RESPONSE_SIZE];
            int total_len = 0;
            int len;

            while ((len = recv(sock, response + total_len, sizeof(response) - total_len - 1, 0)) > 0) {
                total_len += len;
                if (total_len >= sizeof(response) - 1) break;
            }
            response[total_len] = '\0';

            printf("Otrzymana odpowiedź HTTP:\n%s\n", response);
            close(sock);
        }
    }
}

void app_main(void) {
    nvs_flash_init();
    
    // Utworzenie semafora binarnego
    http_semaphore = xSemaphoreCreateBinary();
    if (http_semaphore == NULL) {
        printf("Nie można utworzyć semafora\n");
        return;
    }

    wifi_connection();
    xTaskCreate(&led_blink_task, "led_blink_task", 1024, NULL, 1, NULL);
    xTaskCreate(&http_get_task, "http_get_task", 4096, NULL, 4, NULL);

    fflush(stdout);
}