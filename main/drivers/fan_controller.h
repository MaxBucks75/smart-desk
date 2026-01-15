/******************************************************************************
 * @file    fan_controller.h
 * @brief   Fan PWM control API for Smart Desk
 *****************************************************************************/

#pragma once

#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "temperature_sensor.h"

#define FAN_PWM_GPIO        GPIO_NUM_32
#define FAN_PWM_FREQ_HZ     25000
#define FAN_PWM_TIMER       LEDC_TIMER_0
#define FAN_PWM_MODE        LEDC_HIGH_SPEED_MODE
#define FAN_PWM_CHANNEL     LEDC_CHANNEL_0
#define FAN_PWM_RES         LEDC_TIMER_8_BIT

// Temperature thresholds used to determine fan duty cycle
#define MCP9808_EXHAUST_START_FANS_THRESHOLD_C  22
#define MCP9808_EXHAUST_MAX_FANS_THRESHOLD_C    38
#define MCP9808_CENTRAL_START_FANS_THRESHOLD_C  20
#define MCP9808_CENTRAL_MAX_FANS_THRESHOLD_C    36

/**
 * @brief Initialize the LEDC PWM timer/channel for fan control.
 *
 * Configures the PWM timer and channel used to drive the fan PWM pin.
 */
void fans_init(void);

/**
 * @brief Set fan speed based on temperature readings.
 *
 * This function computes a PWM duty cycle from the provided central and
 * exhaust temperature readings and applies it to the fan PWM channel.
 *
 * @param temp_central Pointer to the central temperature (°C).
 * @param temp_exhaust Pointer to the exhaust temperature (°C).
 */
void fan_set_speed(float* temp_central, float* temp_exhaust);