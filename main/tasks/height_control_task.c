#pragma once

#include <stdio.h>

void height_control_task(void *pvParam) {

}

void height_control_task_init(void) {
    xTaskCreate(height_control_task, "HeightControlTask", 4096, NULL, 2, NULL);
}