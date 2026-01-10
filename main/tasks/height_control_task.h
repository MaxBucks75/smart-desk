#pragma once

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "ipc.h"
#include "drivers/desk_controls.h"

void height_control_task(void *pvParam);
void height_control_task_init(void);
