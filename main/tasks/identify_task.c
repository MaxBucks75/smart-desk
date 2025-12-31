#pragma once

#include <stdio.h>

void identify_task(void *pvParam) {

}

void identify_task_init(void) {
    xTaskCreate(identify_task, "IdentifyTask", 4096, NULL, 2, NULL);
}