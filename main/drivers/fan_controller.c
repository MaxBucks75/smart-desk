#include "fan_controller.h"
#include "driver/ledc.h"
#include "temperature_sensor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"

#define TAG "FANCONTROLLER"

void fans_init(void) {

    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,  
        .duty_resolution = FAN_PWM_RES,  
        .timer_num = FAN_PWM_TIMER,
        .freq_hz = FAN_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    ledc_channel_config_t channel_conf = {
        .gpio_num = FAN_PWM_GPIO,
        .speed_mode = FAN_PWM_MODE,
        .channel = FAN_PWM_CHANNEL,
        .timer_sel = FAN_PWM_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_conf));
}

void fan_set_speed(float *temp_central, float *temp_exhaust) {

    uint16_t duty_cycle = 0;

    if (*temp_central < MCP9808_CENTRAL_START_FANS_THRESHOLD_C && *temp_exhaust < MCP9808_EXHAUST_START_FANS_THRESHOLD_C) {
        duty_cycle = 0; // Shut the fans off if both of the sensors are reading temperatures below the start fans threshold.
    } else if (*temp_central >= MCP9808_CENTRAL_MAX_FANS_THRESHOLD_C || *temp_exhaust >= MCP9808_EXHAUST_MAX_FANS_THRESHOLD_C) {
        duty_cycle = 255; // Max out the fan duty cycle if either sensor is reading above their threshold.
    } else {
        // Get the highest temperature reading from the sensor relative to their starting thresholds and base the duty cycle on that reading.
        if ((*temp_central - MCP9808_CENTRAL_START_FANS_THRESHOLD_C) > (*temp_exhaust - MCP9808_EXHAUST_START_FANS_THRESHOLD_C)) {
            // Converting the temperature ratio of the temp reading over the temp range into the one byte PWM signal.
            duty_cycle = (uint8_t)(((*temp_central - MCP9808_CENTRAL_START_FANS_THRESHOLD_C) / (MCP9808_CENTRAL_MAX_FANS_THRESHOLD_C - MCP9808_CENTRAL_START_FANS_THRESHOLD_C)) * 255.0 );
        } else {
            duty_cycle = (uint8_t)(((*temp_exhaust - MCP9808_EXHAUST_START_FANS_THRESHOLD_C) / (MCP9808_EXHAUST_MAX_FANS_THRESHOLD_C - MCP9808_EXHAUST_START_FANS_THRESHOLD_C)) * 255.0 );
        }
    }

    ESP_LOGI(TAG, "Duty cycle: %d", duty_cycle);

    ESP_ERROR_CHECK(ledc_set_duty(FAN_PWM_MODE, FAN_PWM_CHANNEL, duty_cycle));
    ESP_ERROR_CHECK(ledc_update_duty(FAN_PWM_MODE, FAN_PWM_CHANNEL));

}