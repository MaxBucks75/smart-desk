#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "ipc.h"
#include "drivers/desk_controls.h"

static const TickType_t HEIGHT_SAMPLE_PERIOD = pdTICKS_TO_MS(50);

void height_control_task(void *pvParam) {

    TickType_t last_wake = xTaskGetTickCount();
    

    for (;;) {

        // Check if the conditions are met to lower or raise the desk every ~50 ms
        detect_raising_or_lowering();
        vTaskDelayUntil(&last_wake, HEIGHT_SAMPLE_PERIOD);

    }

}

void height_control_task_init(void) {
    xTaskCreate(height_control_task, "HeightControlTask", 4096, NULL, 2, NULL);
}