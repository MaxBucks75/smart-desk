/******************************************************************************
 * @file    height_control_task.h
 * @brief   Declarations for desk height sampling and baseline maintenance task
 *****************************************************************************/

#pragma once

#include "drivers/desk_controls.h"
#include "freertos/FreeRTOS.h"
#include "ipc.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define STABLE_TIME_TICKS pdMS_TO_TICKS(120000) // 2 minutes
#define STABLE_THRESHOLD 100                    // Stability threshold for samples
#define BASELINE_ALPHA 0.001f

/**
 * @brief Periodic task that maintains sensor baselines and detects gestures.
 */
void height_control_task(void* pvParam);

/**
 * @brief Create the height control task.
 */
void height_control_task_init(void);
