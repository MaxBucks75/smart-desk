#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "ipc.h"
#include "drivers/fingerprint.h"
#include "freertos/FreeRTOS.h"

static const TickType_t ID_SAMPLE_PERIOD = pdMS_TO_TICKS(200);

void identify_task(void *pvParam) {

    sensor_msg_t msg;
    TickType_t last_wake = xTaskGetTickCount();
    esp_err_t err;

    for (;;) {

        err = auto_identify();
        msg.source = SENSOR_SOURCE_TEMP;

        switch (err)
        {
        case ESP_OK: {
            msg.data = 1;
            xQueueSend(sensor_queue, &msg, pdMS_TO_TICKS(10));
            break;
        }
        case ESP_ERR_NOT_FOUND: {
            msg.data = 2;
            xQueueSend(sensor_queue, &msg, pdMS_TO_TICKS(10));
            break;
        }
        case ESP_FAIL: {
            break;
        }
        default:
            break;
        }

        vTaskDelayUntil(&last_wake, ID_SAMPLE_PERIOD);

    }

}

void identify_task_init(void) {
    xTaskCreate(identify_task, "IdentifyTask", 4096, NULL, 2, NULL);
}