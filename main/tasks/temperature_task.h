#pragma once

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "ipc.h"
#include "drivers/temperature_sensor.h"
#include "freertos/FreeRTOS.h"

void temperature_task(void *pvParam);
void temperature_task_init(void);
