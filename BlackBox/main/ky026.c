#include <stdio.h>
#include "ky026.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#define FLAME_SENSOR_DIGITAL_PIN GPIO_NUM_4
#define FLAME_SENSOR_ANALOG_CHANNEL ADC1_CHANNEL_0

static void init_adc(void)
{
    printf("Inicjalizacja ADC...\n");
    esp_err_t err = adc1_config_width(ADC_WIDTH_BIT_12);
    if (err != ESP_OK)
    {
        printf("Błąd konfiguracji szerokości ADC: %d\n", err);
        return;
    }
    err = adc1_config_channel_atten(FLAME_SENSOR_ANALOG_CHANNEL, ADC_ATTEN_DB_12);
    if (err != ESP_OK)
    {
        printf("Błąd konfiguracji atenuacji ADC: %d\n", err);
        return;
    }
    printf("Konfiguracja ADC zakończona pomyślnie\n");
}

static void init_gpio(void)
{
    printf("Inicjalizacja GPIO...\n");
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << FLAME_SENSOR_DIGITAL_PIN),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_DISABLE};
    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK)
    {
        printf("Błąd konfiguracji GPIO: %d\n", err);
        return;
    }
    printf("Konfiguracja GPIO zakończona pomyślnie\n");
}

static int read_raw_adc(void)
{
    return adc1_get_raw(FLAME_SENSOR_ANALOG_CHANNEL);
}

esp_err_t ky026_init(void)
{
    printf("\n\n=== Start diagnostyki czujnika KY-026 ===\n");
    init_gpio();
    init_adc();
    printf("\nRozpoczynam odczyty:\n");
    printf("- GPIO1 (ADC1_0) jest używany jako wejście analogowe\n");
    printf("- GPIO4 jest używany jako wejście cyfrowe\n\n");
    return ESP_OK;
}

int ky026_read_digital(void)
{
    return gpio_get_level(FLAME_SENSOR_DIGITAL_PIN);
}

int ky026_read_raw_analog(void)
{
    return read_raw_adc();
}

int ky026_read_voltage(void)
{
    int raw_value = read_raw_adc();
    return (raw_value * 3300) / 4095; // dla 12-bit ADC
}

void ky026_print_diagnostics(void)
{
    int digital_val = ky026_read_digital();
    int raw_value = ky026_read_raw_analog();
    int voltage = ky026_read_voltage();

    printf("Odczyty:\n");
    printf("- Digital (GPIO4): %d\n", digital_val);
    printf("- Analog surowy (GPIO1): %d\n", raw_value);
    printf("- Analog napięcie: ~%dmV\n", voltage);
    printf("------------------------\n");
}