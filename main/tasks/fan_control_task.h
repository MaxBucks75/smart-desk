#pragma once

#include <stdio.h>
#include <string.h>
#include "ipc.h"
#include "drivers/fan_controller.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

void fan_control_task(void *pvParam);
void fan_control_task_init(void);
