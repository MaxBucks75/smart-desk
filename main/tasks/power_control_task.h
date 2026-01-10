#pragma once

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "ipc.h"
#include "drivers/fingerprint.h"
#include "drivers/desk_controls.h"
#include "esp_log.h"

void power_control_task(void *pvParam);
void power_control_task_init(void);
