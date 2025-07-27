#pragma once

#include <stdint.h>

// Every bit = 2.441 grams
// Full YETI Rambler = ~2100 grams
// So the trigger range will be 2500 grams so 0x0400

#define EXAMPLE_ADC_UNIT         ADC_UNIT_1
#define EXAMPLE_CONV_MODE        ADC_CONV_SINGLE_UNIT_1
#define EXAMPLE_OUTPUT_TYPE      ADC_DIGI_OUTPUT_FORMAT_TYPE1
#define EXAMPLE_ADC_ATTEN        ADC_ATTEN_DB_12
#define EXAMPLE_ADC_BIT_WIDTH    SOC_ADC_DIGI_MAX_BITWIDTH

// Force sensors
#define TL_FORCE_SENSOR_CH ADC_CHANNEL_0 // pin 36
#define TR_FORCE_SENSOR_CH ADC_CHANNEL_3 // pin 39
#define BL_FORCE_SENSOR_CH ADC_CHANNEL_6 // pin 34
#define BR_FORCE_SENSOR_CH ADC_CHANNEL_7 // pin 35
#define FORCE_SENSOR_ACTIVATED_THRESHOLD    0x0400
#define DEBOUNCE_DELAY_MS                   1000
#define READ_LEN                            1024

void adc_continuous_init(void);
void get_activated_force_sensors(void);
void detect_raising_or_lowering(void);
void change_desk_height(void);
void open_usb_hub(void);
void adc_reader_task(void *arg);
void get_activated_force_sensors(void);