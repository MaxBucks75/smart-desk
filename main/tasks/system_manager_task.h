#pragma once

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "ipc.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

void system_manager_task(void *pvParam);
void system_manager_task_init(void);
