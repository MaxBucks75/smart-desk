/******************************************************************************
 * @file    temperature_task.c
 * @brief   Periodically reads temperature sensors and publishes readings.
 ******************************************************************************/

#include "temperature_task.h"

#define TAG "TempTask"

static const TickType_t TEMP_SAMPLE_PERIOD = pdMS_TO_TICKS(200);

/**
 * @brief FreeRTOS task that samples temperature sensors and posts messages.
 *
 * Reads the central and exhaust temperature sensors via the temperature
 * driver, packs the two float values into a single 'sensor_msg_t' and sends
 * it on 'sensor_queue' for aggregation by the system manager.
 */
void temperature_task(void* pvParam) {

    sensor_msg_t msg;
    TickType_t   last_wake = xTaskGetTickCount();

    for (;;) {

        float temp_central = get_temperature_reading(temp_handle_central);
        float temp_exhaust = get_temperature_reading(temp_handle_exhaust);

        // Pack float data into message
        uint32_t central_bits, exhaust_bits;
        memcpy(&central_bits, &temp_central, sizeof(float));
        memcpy(&exhaust_bits, &temp_exhaust, sizeof(float));
        msg.data = ((uint64_t)central_bits << 32) | exhaust_bits;

        msg.source = SENSOR_SOURCE_TEMP;

        xQueueSend(sensor_queue, &msg, pdMS_TO_TICKS(10));

        vTaskDelayUntil(&last_wake, TEMP_SAMPLE_PERIOD);
    }
}

/**
 * @brief Create and start the temperature sampling task.
 */
void temperature_task_init(void) {
    xTaskCreate(temperature_task, "TempTask", 4096, NULL, PRIO_SENSOR, NULL);
}