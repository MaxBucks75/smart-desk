/******************************************************************************
 * @file    desk_controls.h
 * @brief   Declarations for desk control and sensor helpers for Smart Desk
 *****************************************************************************/

#pragma once

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_continuous.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tasks/ipc.h"
#include <stdint.h>

// ADC 
#define ADC_UNIT ADC_UNIT_1
#define CONV_MODE ADC_CONV_SINGLE_UNIT_1
#define OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE1
#define ADC_ATTEN ADC_ATTEN_DB_12
#define ADC_BIT_WIDTH SOC_ADC_DIGI_MAX_BITWIDTH

// Force sensors - connected to 4.7k Ohm resistors
#define TL_FORCE_SENSOR_CH ADC_CHANNEL_3 // pin 39 Yellow White
#define TR_FORCE_SENSOR_CH ADC_CHANNEL_0 // pin 36 Green Green
#define BL_FORCE_SENSOR_CH ADC_CHANNEL_6 // pin 34 Blue White
#define BR_FORCE_SENSOR_CH ADC_CHANNEL_7 // pin 35 Yellow Yellow

// Tuned force sensor detection thresholds based on physical differences in housing
#define TL_FORCE_SENSOR_ACTIVATED_THRESHOLD 0x01F4 // 500
#define TR_FORCE_SENSOR_ACTIVATED_THRESHOLD 0x01F4 // 500
#define BL_FORCE_SENSOR_ACTIVATED_THRESHOLD 0x01F4 // 500
#define BR_FORCE_SENSOR_ACTIVATED_THRESHOLD 0x01F4 // 500

#define DEBOUNCE_DELAY_MS 1000 // Delay (ms) that force sensors must be held under threshold
#define FORCE_ALPHA       0.1f 
#define READ_LEN          8     

// Global sensor calibration variables
extern uint16_t tl_raw, tr_raw, bl_raw, br_raw;
extern uint16_t tl_filtered, tr_filtered, bl_filtered, br_filtered;
extern uint16_t tl_baseline, tr_baseline, bl_baseline, br_baseline;

// Linear Actuator
#define LINEAR_ACTUATOR_IN1 GPIO_NUM_27
#define LINEAR_ACTUATOR_IN2 GPIO_NUM_26
#define LINEAR_ACTUATOR_ENA GPIO_NUM_25

typedef struct {
    TickType_t start_time;    // when the motion started
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

/**
 * @brief Initialize ADC continuous sampling and baseline/filter values.
 */
void adc_continuous_init(void);

/**
 * @brief Read ADC continuous buffer and update raw/filtered sensor values.
 */
void adc_reader(void);

/**
 * @brief Update 'smart_desk_events' activation flags from filtered sensor values.
 */
void get_activated_force_sensors(void);

/**
 * @brief Detect raise/lower inputs from force sensors and debounce them.
 *
 * This function updates 'smart_desk_events.desk_moving' and will call
 * start/stop functions for desk motion when debounced inputs are observed.
 */
void detect_raising_or_lowering(void);

/**
 * @brief Initialize relay GPIOs and onboard LED used by desk controls.
 */
void relay_init(void);

/**
 * @brief Assert relay to start raising the desk.
 */
void start_raising_desk(void);

/**
 * @brief Deassert relay to stop raising the desk.
 */
void stop_raising_desk(void);

/**
 * @brief Assert relay to start lowering the desk.
 */
void start_lowering_desk(void);

/**
 * @brief Deassert relay to stop lowering the desk.
 */
void stop_lowering_desk(void);

/**
 * @brief Pulse motherboard power relay pins to simulate a power button press.
 */
void short_mobo_power_pins(void);

// Completed code that I didn't physically implement into the desk:
// void vibrator_init(void);
// void vibrator_set_strength(uint8_t duty);
// void linear_actuator_init(void);
// void control_usb_hub(void);
// void extend_actuator(void);
// void retract_actuator(void);