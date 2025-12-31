#pragma once

#include <stdio.h>

void system_manager_task(void *pvParam) {

    //sensor_msg_t msg;
    //TickType_t now = xTaskGetTickCount();

}

void system_manager_task_init(void) {
    xTaskCreate(system_manager_task, "SystemManagerTask", 4096, NULL, 3, NULL);
}