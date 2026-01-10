#pragma once

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "ipc.h"
#include "drivers/fingerprint.h"
#include "freertos/FreeRTOS.h"

void identify_task(void *pvParam);
void identify_task_init(void);
