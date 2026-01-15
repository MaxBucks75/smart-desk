/******************************************************************************
 * @file    main.c
 * @brief   Entry point: initialize drivers, peripherals, and start tasks.
 *
 * Initializes platform drivers (UART, I2C, ADC, fans, relays), sets up IPC
 * and starts FreeRTOS tasks for temperature sampling, identification,
 * fan/height/power control and the system manager.
 *
 * @date    2026-01-14
 *****************************************************************************/

#include "driver/i2c.h"
#include "driver/uart.h"
#include "drivers/debug_tasks.h"
#include "drivers/desk_controls.h"
#include "drivers/fan_controller.h"
#include "drivers/fingerprint.h"
#include "drivers/temperature_sensor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "semaphore.h"
#include "string.h"
#include "tasks/fan_control_task.h"
#include "tasks/height_control_task.h"
#include "tasks/identify_task.h"
#include "tasks/ipc.h"
#include "tasks/power_control_task.h"
#include "tasks/system_manager_task.h"
#include "tasks/temperature_task.h"
#include <stdio.h>

// #define CONFIG_DEBUG

/**
 * @brief ESP-IDF application entry point.
 *
 * Initializes core drivers and starts the application's FreeRTOS tasks.
 * When 'CONFIG_DEBU' is defined the debug task set is started instead of
 * the normal runtime tasks.
 */
void app_main(void) {
    printf("Smart Desk Booting Up...\n");
    vTaskDelay(pdMS_TO_TICKS(250));

    R503_init();
    i2c_init();
    adc_continuous_init();
    fans_init();
    relay_init();

    vTaskDelay(pdMS_TO_TICKS(50));

#ifdef CONFIG_DEBUG
    init_debug_tasks();
#else

    ipc_init();
    smart_desk_events.computer_power = 0; // Computer is off when power is restored

    temperature_task_init();
    identify_task_init();

    fan_control_task_init();
    height_control_task_init();
    power_control_task_init();

    system_manager_task_init();

    smart_desk_events.set_fp_led_pur = 1; // Set R503 LED to purple

#endif
}