/*******************************************************************************
 * @file    system_manager_task.h
 * @brief   Public API for the system manager task which aggregates sensor
 *          inputs and issues control decisions to other subsystems.
 ******************************************************************************/

#pragma once

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "ipc.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/**
 * @brief The FreeRTOS task function for the system manager.
 *
 * @param pvParam Unused task parameter (NULL expected).
 */
void system_manager_task(void* pvParam);

/**
 * @brief Create and start the system manager FreeRTOS task.
 */
void system_manager_task_init(void);
