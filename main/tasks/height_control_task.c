#include "height_control_task.h"

#define TAG "HeightControlTask"

static const TickType_t HEIGHT_SAMPLE_PERIOD = pdMS_TO_TICKS(50);

void height_control_task(void *pvParam) {

    TickType_t last_wake = xTaskGetTickCount();
    

    for (;;) {

        ESP_LOGI(TAG, "Height Control task running...");

        // Sample ADC, update raw values
        adc_reader();

        // Check if the conditions are met to lower or raise the desk every ~50 ms
        detect_raising_or_lowering();
        vTaskDelayUntil(&last_wake, HEIGHT_SAMPLE_PERIOD);

    }

}

void height_control_task_init(void) {
    xTaskCreate(height_control_task, "HeightControlTask", 4096, NULL, PRIO_HEIGHT_CONTROL, NULL);
}