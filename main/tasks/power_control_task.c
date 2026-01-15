/******************************************************************************
 * @file    power_control_task.c
 * @brief   Task that handles power on/off actions driven by fingerprint results
 * @author  Max Bucks
 * @date    2026-01-14
 *****************************************************************************/

#include "power_control_task.h"

#define TAG "PowerControlTask"

/**
 * @brief Task that receives power update commands and toggles motherboard power.
 */
void power_control_task(void* pvParam) {

    control_msg_t msg;

    for (;;) {

        if (xQueueReceive(control_queue, &msg, portMAX_DELAY) == pdTRUE &&
            msg.cmd == UPDATE_POWER) {

            ESP_LOGI(TAG, "Power control task running...");

            // Check if the fingerpint was found in the R503 library
            if (msg.data == 1) {

                // Turn computer on or off and update flag
                short_mobo_power_pins();
                smart_desk_events.computer_power = ~smart_desk_events.computer_power;

                // Check if the desk is now on or off
                if (smart_desk_events.computer_power == 0) {
                    smart_desk_events.set_fp_led_pur = 1;
                    ESP_LOGI(TAG, "Computer powering down...");
                } else if (smart_desk_events.computer_power == 1) {
                    smart_desk_events.set_fp_led_blu = 1;
                    ESP_LOGI(TAG, "Computer booting up...");
                }

                smart_desk_events.finger_detected = 0; // Ready to resume auto identify

                // Fingerprint not found in R503 library, update instantly
            } else {

                // Update flag so LED will reflect current power state
                if (smart_desk_events.computer_power == 0) {
                    smart_desk_events.set_fp_led_pur = 1;
                } else if (smart_desk_events.computer_power == 1) {
                    smart_desk_events.set_fp_led_blu = 1;
                }

                set_led(R503_LED_GRADUALLY_OFF, 0xCC, R503_LED_COLOR_RED,
                        0x00); // Flash red if fingerprint not found
            }
        }
    }
}

/**
 * @brief Create the power control task.
 */
void power_control_task_init(void) {
    xTaskCreate(power_control_task, "PowerControlTask", 4096, NULL, PRIO_CONTROL, NULL);
}