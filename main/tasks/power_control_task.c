#pragma once

#include <stdio.h>

void power_control_task(void *pvParam) {

}

void power_control_task_init(void) {
    xTaskCreate(power_control_task, "PowerControlTask", 4096, NULL, 2, NULL);
}