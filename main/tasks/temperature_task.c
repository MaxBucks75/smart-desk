#pragma once

#include <stdio.h>

void temperature_task(void *pvParam) {

}

void temperature_task_init(void) {
    xTaskCreate(temperature_task, "TempTask", 4096, NULL, 1, NULL);
}