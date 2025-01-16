#ifndef KY026_H
#define KY026_H

#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define FLAME_SENSOR_DIGITAL_PIN GPIO_NUM_4
#define FLAME_SENSOR_ADC_UNIT ADC_UNIT_1
#define FLAME_SENSOR_ADC_CHANNEL ADC_CHANNEL_0

esp_err_t ky026_init(void);
int ky026_read_digital(void);
int ky026_read_raw_analog(void);
int ky026_read_voltage(void);
void ky026_print_diagnostics(void);

#endif