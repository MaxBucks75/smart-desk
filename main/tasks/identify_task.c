/******************************************************************************
 * @file    identify_task.c
 * @brief   Task that performs fingerprint identification and LED updates
 * @author  Max Bucks
 * @date    2026-01-14
 *****************************************************************************/

#include "identify_task.h"

#define TAG "IdentifyTask"

static const TickType_t ID_SAMPLE_PERIOD = pdMS_TO_TICKS(500);

/**
 * @brief Periodically attempts fingerprint identification and signals results.
 *
 * Calls 'auto_identify()' and forwards identification results via 'sensor_queue'.
 */
void identify_task(void* pvParam) {

    sensor_msg_t msg;
    TickType_t   last_wake                  = xTaskGetTickCount();
    TickType_t   time_finger_print_detected = xTaskGetTickCount();
    TickType_t   now;
    esp_err_t    err;

    for (;;) {

        // Ensure we are doing nothing while power control task is working
        if (smart_desk_events.finger_detected == 1) {

            now = xTaskGetTickCount();
            if ((now - time_finger_print_detected) >= pdMS_TO_TICKS(500)) {
                smart_desk_events.finger_detected = 0;
            }

            // Update the fingerprint LED to reflect computer's power
        } else if (smart_desk_events.set_fp_led_blu || smart_desk_events.set_fp_led_pur) {

            if (smart_desk_events.set_fp_led_pur) {
                // breathe purple while computer is off
                set_led(R503_LED_BREATHING, 0xFF, R503_LED_COLOR_PURPLE,0x00); 
                smart_desk_events.set_fp_led_pur = 0;
                ESP_LOGI(TAG, "Computer powering down...");
            } else if (smart_desk_events.set_fp_led_blu) {
                // solid blue while computer is on
                set_led(R503_LED_GRADUALLY_ON, 0xC0, R503_LED_COLOR_BLUE,0x00);
                smart_desk_events.set_fp_led_blu = 0;
                ESP_LOGI(TAG, "Computer booting up...");
            }

            smart_desk_events.finger_detected = 0; // Fingerprint detected has been handled

        } else if (smart_desk_events.finger_detected == 0) {

            err        = auto_identify();
            msg.source = SENSOR_SOURCE_FP;

            switch (err) {
            case ESP_OK: {
                msg.data                          = 1;
                smart_desk_events.finger_detected = 1;
                time_finger_print_detected        = xTaskGetTickCount();
                xQueueSend(sensor_queue, &msg, pdMS_TO_TICKS(10));
                break;
            }
            case ESP_ERR_NOT_FOUND: {
                msg.data                          = 2;
                smart_desk_events.finger_detected = 1;
                time_finger_print_detected        = xTaskGetTickCount();
                xQueueSend(sensor_queue, &msg, pdMS_TO_TICKS(10));
                break;
            }
            case ESP_FAIL: {
                break;
            }
            default:
                break;
            }
        }

        vTaskDelayUntil(&last_wake, ID_SAMPLE_PERIOD);
    }
}

/**
 * @brief Create the identify task.
 */
void identify_task_init(void) {
    xTaskCreate(identify_task, "IdentifyTask", 4096, NULL, PRIO_IDLE, NULL);
}