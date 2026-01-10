#pragma once

#include <stdint.h>
#include "esp_adc/adc_continuous.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "tasks/ipc.h"

#define ADC_UNIT         ADC_UNIT_1
#define CONV_MODE        ADC_CONV_SINGLE_UNIT_1
#define OUTPUT_TYPE      ADC_DIGI_OUTPUT_FORMAT_TYPE1
#define ADC_ATTEN        ADC_ATTEN_DB_12
#define ADC_BIT_WIDTH    SOC_ADC_DIGI_MAX_BITWIDTH

// Force sensors - connected to 4.7k Ohm resistors
#define TL_FORCE_SENSOR_CH ADC_CHANNEL_3        // pin 39 Yellow White
#define TR_FORCE_SENSOR_CH ADC_CHANNEL_0        // pin 36 Green Green
#define BL_FORCE_SENSOR_CH ADC_CHANNEL_6        // pin 34 Blue White
#define BR_FORCE_SENSOR_CH ADC_CHANNEL_7        // pin 35 Yellow Yellow
// Tuned all force sensor thresholds based on variations in housing (ideally at 0 pressure output is 0x0FFF or max voltage)
#define TL_FORCE_SENSOR_ACTIVATED_THRESHOLD    0x0000
#define TR_FORCE_SENSOR_ACTIVATED_THRESHOLD    0x0000
#define BL_FORCE_SENSOR_ACTIVATED_THRESHOLD    0x0000
#define BR_FORCE_SENSOR_ACTIVATED_THRESHOLD    0x0000
#define DEBOUNCE_DELAY_MS                      1000 // Delay (ms) that force sensors must be held under threshold
#define READ_LEN                               8

// Linear Actuator
#define LINEAR_ACTUATOR_IN1 GPIO_NUM_27
#define LINEAR_ACTUATOR_IN2 GPIO_NUM_26
#define LINEAR_ACTUATOR_ENA GPIO_NUM_25

typedef struct {
    TickType_t start_time;   // when the motion started
    TickType_t move_duration; // how long the actuator should move
} actuator_ctx_t;

// Relay
#define RELAY_LINEAR_ACTUATOR_PIN GPIO_NUM_33

// Height control
#define RELAY_DESK_RAISE_PIN GPIO_NUM_14
#define RELAY_DESK_LOWER_PIN GPIO_NUM_12

// Power
#define RELAY_MOBO_POWER GPIO_NUM_13
#define ONBOARD_LED GPIO_NUM_2

// Feedback Vibrators
#define VIBRATOR_PIN GPIO_NUM_5

void adc_continuous_init(void);
void adc_reader(void);

void get_activated_force_sensors(void);
void detect_raising_or_lowering(void);
void get_activated_force_sensors(void);

void linear_actuator_init(void);
void control_usb_hub(void);

void relay_init(void);
// void extend_actuator(void);
// void retract_actuator(void);
void start_raising_desk(void);
void stop_raising_desk(void);
void start_lowering_desk(void);
void stop_lowering_desk(void);
void short_mobo_power_pins(void);

void vibrator_init(void);
void vibrator_set_strength(uint8_t duty);
