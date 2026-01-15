/******************************************************************************
 * @file    fan_controller.c
 * @brief   Implements fan PWM control for Smart Desk
 * @author  Max Bucks
 * @date    2026-01-14
 *****************************************************************************/

#include "fan_controller.h"

#define TAG "FANCONTROLLER"

/**
 * @brief Initialize PWM timer/channel for the fan.
 *
 * Configures LEDC timer and channel according to definitions in the
 * corresponding header file and starts with duty=0 (fan off).
 */
void fans_init(void) {

    ledc_timer_config_t timer_conf = {.speed_mode      = LEDC_HIGH_SPEED_MODE,
                                      .duty_resolution = FAN_PWM_RES,
                                      .timer_num       = FAN_PWM_TIMER,
                                      .freq_hz         = FAN_PWM_FREQ_HZ,
                                      .clk_cfg         = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    ledc_channel_config_t channel_conf = {.gpio_num   = FAN_PWM_GPIO,
                                          .speed_mode = FAN_PWM_MODE,
                                          .channel    = FAN_PWM_CHANNEL,
                                          .timer_sel  = FAN_PWM_TIMER,
                                          .duty       = 0,
                                          .hpoint     = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&channel_conf));
}

/**
 * @brief Compute and apply fan PWM duty based on temperatures.
 *
 * The routine applies threshold logic:
 * - If both temps are below start thresholds -> fan off
 * - If either temp exceeds max thresholds -> fan at max duty
 * - Otherwise scale duty between start and max thresholds proportionally
 *
 * @param temp_central Pointer to central temperature (°C).
 * @param temp_exhaust Pointer to exhaust temperature (°C).
 */
void fan_set_speed(float* temp_central, float* temp_exhaust) {

    uint16_t duty_cycle = 0;

    // If both sensors are below min threshold, minimize duty cycle
    if (*temp_central < MCP9808_CENTRAL_START_FANS_THRESHOLD_C &&
        *temp_exhaust < MCP9808_EXHAUST_START_FANS_THRESHOLD_C) {
        duty_cycle = 0;

    // If either sensor is above the max threshold, maximize duty cycle
    } else if (*temp_central >= MCP9808_CENTRAL_MAX_FANS_THRESHOLD_C ||
               *temp_exhaust >= MCP9808_EXHAUST_MAX_FANS_THRESHOLD_C) {
        duty_cycle = 255;
    } else {

        // Scale duty cycle to (temp - threshold) / (max to min threshold temp range)
        if ((*temp_central - MCP9808_CENTRAL_START_FANS_THRESHOLD_C) >
            (*temp_exhaust - MCP9808_EXHAUST_START_FANS_THRESHOLD_C)) {
            duty_cycle = (uint8_t)(((*temp_central - MCP9808_CENTRAL_START_FANS_THRESHOLD_C) /
                                    (MCP9808_CENTRAL_MAX_FANS_THRESHOLD_C -
                                     MCP9808_CENTRAL_START_FANS_THRESHOLD_C)) *
                                   255.0);
        } else {
            duty_cycle = (uint8_t)(((*temp_exhaust - MCP9808_EXHAUST_START_FANS_THRESHOLD_C) /
                                    (MCP9808_EXHAUST_MAX_FANS_THRESHOLD_C -
                                     MCP9808_EXHAUST_START_FANS_THRESHOLD_C)) *
                                   255.0);
        }
    }

    ESP_LOGI(TAG, "Fan duty cycle set: %d", duty_cycle);

    ESP_ERROR_CHECK(ledc_set_duty(FAN_PWM_MODE, FAN_PWM_CHANNEL, duty_cycle));
    ESP_ERROR_CHECK(ledc_update_duty(FAN_PWM_MODE, FAN_PWM_CHANNEL));

}