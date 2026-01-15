/******************************************************************************
 * @file    identify_task.h
 * @brief   Declarations for fingerprint identification task
 *****************************************************************************/

#pragma once

#include "drivers/fingerprint.h"
#include "freertos/FreeRTOS.h"
#include "ipc.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/**
 * @brief Task that attempts to identify presented fingerprints.
 */
void identify_task(void* pvParam);

/**
 * @brief Create the identify task.
 */
void identify_task_init(void);
