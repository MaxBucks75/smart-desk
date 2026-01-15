/******************************************************************************
 * @file    temperature_task.h
 * @brief   Public API for the temperature sampling task.
 ******************************************************************************/

#pragma once

#include "drivers/temperature_sensor.h"
#include "freertos/FreeRTOS.h"
#include "ipc.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/**
 * @brief FreeRTOS task that samples temperature sensors and posts messages.
 *
 * @param pvParam Unused task parameter (NULL expected).
 */
void temperature_task(void* pvParam);

/**
 * @brief Create and start the temperature sampling task.
 */
void temperature_task_init(void);
