#include "nvs_flash.h"
#include "wifi.h"
#include "ble.h"
#include "mqtt.h"

#include "nvs_flash.h"
#include "wifi.h"
#include "ble.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BOOT_BUTTON GPIO_NUM_0

void button_task(void *pvParameters)
{
    bool last_state = true;
    while (1)
    {
        bool current_state = gpio_get_level(BOOT_BUTTON);

        if (last_state == true && current_state == false)
        {
            printf("BLE ENABLED: %d", ble_enabled());
            if (!ble_enabled())
            {
                wifi_stop();
                ble_toggle(true);
            }
            else
            {
                ble_toggle(false);
                wifi_connection();
            }
        }

        last_state = current_state;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
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
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BOOT_BUTTON),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);

    wifi_connection();
    vTaskDelay(pdMS_TO_TICKS(2000));
    mqtt_init();

    xTaskCreate(button_task, "button_task", 4096, NULL, 5, NULL);
}