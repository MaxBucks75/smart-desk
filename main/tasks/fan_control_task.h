/******************************************************************************
 * @file    fan_control_task.h
 * @brief   Declarations for fan control task
 *****************************************************************************/

#pragma once

#include "drivers/fan_controller.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "ipc.h"
#include <stdio.h>
#include <string.h>

/**
 * @brief Task entry that receives temperature messages and updates fans.
 */
void fan_control_task(void* pvParam);

/**
 * @brief Create the fan control task.
 */
void fan_control_task_init(void);
