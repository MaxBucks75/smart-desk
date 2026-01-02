#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "ipc.h"
#include "drivers/fingerprint.h"
#include "drivers/desk_controls.h"
#include "esp_log.h"

#define TAG "PowerControl"

void power_control_task(void *pvParam) {

    control_msg_t msg;

    for (;;) {

        if (xQueueReceive(control_queue, &msg, portMAX_DELAY) == pdTRUE && msg.cmd == UPDATE_POWER) {

            // Check if the fingerpint was found in the R503 library
            if (msg.data == 1) {

                // Turn computer on or off and update flag
                short_mobo_power_pins();
                smart_desk_events.computer_power = ~smart_desk_events.computer_power;

                // Check if the desk is now on or off
                if (smart_desk_events.computer_power == 0) {
                    set_led(R503_LED_GRADUALLY_OFF, 0xC0, R503_LED_COLOR_PURPLE, 0x00); // flash purple if powering down
                    ESP_LOGI(TAG, "Computer powering down...");
                } else if (smart_desk_events.computer_power == 1) {
                    set_led(R503_LED_GRADUALLY_ON, 0xC0, R503_LED_COLOR_BLUE, 0x00); // flash blue if powering up
                    ESP_LOGI(TAG, "Computer booting up...");
                }

                // Fingerprint not found in R503 library
            } else {
                set_led(R503_LED_GRADUALLY_OFF, 0xC0, R503_LED_COLOR_RED, 0x00); // flash red if fingerprint not found
            }

        }

    }

}

void power_control_task_init(void) {
    xTaskCreate(power_control_task, "PowerControlTask", 4096, NULL, 2, NULL);
}