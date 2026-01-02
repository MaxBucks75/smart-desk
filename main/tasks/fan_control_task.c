#include <stdio.h>
#include <string.h>
#include "ipc.h"
#include "drivers/fan_controller.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#define TAG "FanControl"

void fan_control_task(void *pvParam)
{
 
    control_msg_t msg;

    for (;;) {

        if (xQueueReceive(control_queue, &msg, portMAX_DELAY) == pdTRUE) {

            if (msg.cmd == SET_FAN_SPEED) {

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

void fan_control_task_init(void) {
    xTaskCreate(fan_control_task, "FanControlTask", 4096, NULL, 2, NULL);
}