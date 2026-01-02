#include <stdio.h>
#include "drivers/debug_tasks.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "drivers/fingerprint.h"
#include "esp_log.h"
#include "drivers/desk_controls.h"
#include "driver/uart.h"
#include "drivers/temperature_sensor.h"
#include "drivers/fan_controller.h"
#include "string.h"
#include "driver/i2c.h"
#include "semaphore.h"
#include "tasks/temperature_task.c"
#include "tasks/identify_task.c"
#include "tasks/fan_control_task.c"
#include "tasks/height_control_task.c"
#include "tasks/power_control_task.c"
#include "tasks/system_manager_task.c"
#include "tasks/ipc.h"


#define CONFIG_DEBUG

void app_main(void)
{
    printf("Smart Desk Booting Up...\n");
    vTaskDelay(pdMS_TO_TICKS(500));

    R503_init();
    i2c_init();
    adc_continuous_init();
    fans_init();
    linear_actuator_init();
    relay_init();
    vibrator_init();

    ipc_init();

    temperature_task_init();
    identify_task_init();

    fan_control_task_init();
    height_control_task_init();
    power_control_task_init();

    system_manager_task_init();

    #ifdef CONFIG_DEBUG
    init_debug_tasks();
    #endif

}