/******************************************************************************
 * @file    debug_tasks.h
 * @brief   Declarations for debug/testing tasks for the Smart Desk project
 *****************************************************************************/

#pragma once

#include "desk_controls.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "fan_controller.h"
#include "fingerprint.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "semaphore.h"
#include "string.h"
#include "tasks/ipc.h"
#include "temperature_sensor.h"

#include <stdio.h>

#define DEBUG_UART UART_NUM_0

void init_debug_tasks(void);