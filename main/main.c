#include <stdio.h>
#include "debug_tasks.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "fingerprint.h"
#include "esp_log.h"
#include "events.h"
#include "desk_controls.h"
#include "driver/uart.h"
#include "temperature_sensor.h"
#include "fan_controller.h"
#include "string.h"
#include "driver/i2c.h"
#include "semaphore.h"

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

    #ifdef CONFIG_DEBUG
    init_debug_tasks();
    #endif

}