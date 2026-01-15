/******************************************************************************
 * @file    power_control_task.h
 * @brief   Declarations for power control task
 *****************************************************************************/

#pragma once

#include "drivers/desk_controls.h"
#include "drivers/fingerprint.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "ipc.h"
#include <stdio.h>

/**
 * @brief Task that processes fingerprint results and controls power.
 */
void power_control_task(void* pvParam);

/**
 * @brief Create the power control task.
 */
void power_control_task_init(void);
