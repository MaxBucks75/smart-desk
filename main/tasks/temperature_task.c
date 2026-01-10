#include "temperature_task.h"

#define TAG "TempTask"

static const TickType_t TEMP_SAMPLE_PERIOD = pdMS_TO_TICKS(200);

void temperature_task(void *pvParam)
{

    sensor_msg_t msg;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {

        ESP_LOGI(TAG, "Temp task running...");

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

void temperature_task_init(void) {
    xTaskCreate(temperature_task, "TempTask", 4096, NULL, PRIO_SENSOR, NULL);
}