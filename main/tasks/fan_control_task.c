#pragma once

#include <stdio.h>

void fan_control_task(void *pvParam) {

}

void fan_control_task_init(void) {
    xTaskCreate(fan_control_task, "FanControlTask", 4096, NULL, 1, NULL);
}