#pragma once

#include <stdio.h>

void force_task(void *pvParam) {

}

void force_task_init(void) {
    xTaskCreate(force_task, "ForceTask", 4096, NULL, 1, NULL);
}