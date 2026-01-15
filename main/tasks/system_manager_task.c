/******************************************************************************
 * @file    system_manager_task.c
 * @brief   Aggregates sensor inputs and dispatches control messages
 * @author  Max Bucks
 * @date    2026-01-14
 *****************************************************************************/

#include "system_manager_task.h"

#define TAG "SystemManager"

static float last_temp_central = NAN;
static float last_temp_exhaust = NAN;

/**
 * @brief Task that aggregates sensor readings and sends control commands.
 *
 * Listens on 'sensor_queue' for sensor messages and translates significant
 * changes into control messages (e.g., fan speed, power updates) sent to
 * 'control_queue'.
 */
void system_manager_task(void* pvParam) {

    sensor_msg_t  sensor_msg;
    control_msg_t control_msg;

    for (;;) {

        if (xQueueReceive(sensor_queue, &sensor_msg, pdMS_TO_TICKS(200)) == pdTRUE) {

            if (sensor_msg.source == SENSOR_SOURCE_TEMP) {

                // Get the current temp readings by unpacking float data
                uint32_t central_bits = (uint32_t)(sensor_msg.data >> 32);
                uint32_t exhaust_bits = (uint32_t)(sensor_msg.data & 0xFFFFFFFF);

                float curr_temp_central, curr_temp_exhaust;
                memcpy(&curr_temp_central, &central_bits, sizeof(float));
                memcpy(&curr_temp_exhaust, &exhaust_bits, sizeof(float));

                // Check if this is the first reading
                if (isnan(last_temp_central)) {
                    last_temp_central = curr_temp_central;
                    last_temp_exhaust = curr_temp_exhaust;
                }

                // Check if there has been a change of +/- 0.07 degrees in either of the readings
                if (fabsf(curr_temp_central - last_temp_central) >= 0.07 ||
                    fabsf(curr_temp_exhaust - last_temp_exhaust) >= 0.07) {
                    // Build control message and add it to queue
                    control_msg.cmd  = SET_FAN_SPEED;
                    control_msg.data = sensor_msg.data;
                    if (xQueueSend(control_queue, &control_msg, pdMS_TO_TICKS(10)) == pdTRUE) {
                        // ONLY update last temp if we are updating fan PWM based on that temp delta
                        last_temp_central = curr_temp_central;
                        last_temp_exhaust = curr_temp_exhaust;
                    } else {
                        ESP_LOGW(TAG, "Control queue error when setting fan speed");
                    }
                }

            } else if (sensor_msg.source == SENSOR_SOURCE_FP) {
                control_msg.cmd  = UPDATE_POWER;
                control_msg.data = sensor_msg.data;
                if (xQueueSend(control_queue, &control_msg, pdMS_TO_TICKS(10)) == pdTRUE) {
                    ESP_LOGW(TAG, "Control queue error when updating power");
                }
            }
        }
    }
}

/**
 * @brief Create the system manager task.
 */
void system_manager_task_init(void) {
    xTaskCreate(system_manager_task, "SystemManagerTask", 4096, NULL, PRIO_SYSTEM_MANAGER, NULL);
}