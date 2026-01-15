/******************************************************************************
 * @file    fan_control_task.c
 * @brief   Task that updates fan PWM based on temperature readings
 * @author  Max Bucks
 * @date    2026-01-14
 *****************************************************************************/

#include "fan_control_task.h"

#define TAG "FanControlTask"

/**
 * @brief FreeRTOS task that listens for temperature control messages.
 *
 * Extracts packed float temperature values from 'control_msg_t' messages
 * and forwards them to the fan controller when the computer is powered.
 */
void fan_control_task(void* pvParam) {

    control_msg_t msg;

    for (;;) {

        if (xQueueReceive(control_queue, &msg, portMAX_DELAY) == pdTRUE &&
            msg.cmd == SET_FAN_SPEED) {

            // Only update PWM if the computer is on, PSU doesn't provide 12V idle power rail
            if (smart_desk_events.computer_power) {

                // Get the current temp readings by unpacking float data
                uint32_t central_bits = (uint32_t)(msg.data >> 32);
                uint32_t exhaust_bits = (uint32_t)(msg.data & 0xFFFFFFFF);

                float temp_central, temp_exhaust;
                memcpy(&temp_central, &central_bits, sizeof(float));
                memcpy(&temp_exhaust, &exhaust_bits, sizeof(float));

                // Update the fan speed
                fan_set_speed(&temp_central, &temp_exhaust);

                ESP_LOGI(TAG, "Fan speed updated");
            }
        }
    }
}

/**
 * @brief Create the fan control FreeRTOS task.
 */
void fan_control_task_init(void) {
    xTaskCreate(fan_control_task, "FanControlTask", 4096, NULL, PRIO_CONTROL, NULL);
}