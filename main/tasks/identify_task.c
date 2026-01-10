#include "identify_task.h"

#define TAG "IdentifyTask"

static const TickType_t ID_SAMPLE_PERIOD = pdMS_TO_TICKS(500);

void identify_task(void *pvParam) {

    sensor_msg_t msg;
    TickType_t last_wake = xTaskGetTickCount();
    TickType_t time_finger_print_detected = xTaskGetTickCount();
    TickType_t now;
    esp_err_t err;

    for (;;) {

        // Ensure we are doing nothing while power control task is working
        if (smart_desk_events.finger_detected == 1) {

            ESP_LOGI(TAG, "ID task running NOTHING!");

            now = xTaskGetTickCount();
            if ((now - time_finger_print_detected) >= pdMS_TO_TICKS(500)) {
                smart_desk_events.finger_detected = 0;
            }

        // Update the fingerprint LED to reflect computer's power
        } else if (smart_desk_events.set_fp_led_blu || smart_desk_events.set_fp_led_pur) {

            ESP_LOGI(TAG, "ID task running, updating R503 LED");

            if (smart_desk_events.set_fp_led_pur) {
                set_led(R503_LED_BREATHING, 0xFF, R503_LED_COLOR_PURPLE, 0x00); // breathe purple while computer is off
                smart_desk_events.set_fp_led_pur = 0;
                ESP_LOGI(TAG, "Computer powering down...");
            } else if (smart_desk_events.set_fp_led_blu) {
                set_led(R503_LED_GRADUALLY_ON, 0xC0, R503_LED_COLOR_BLUE, 0x00); // solid blue while computer is on
                smart_desk_events.set_fp_led_blu = 0;
                ESP_LOGI(TAG, "Computer booting up...");
            }

            smart_desk_events.finger_detected = 0; // fingerprint 

        } else if (smart_desk_events.finger_detected == 0) {

            ESP_LOGI(TAG, "ID task running auto identify");

            err = auto_identify();
            msg.source = SENSOR_SOURCE_FP;

            switch (err)
            {
            case ESP_OK: {
                msg.data = 1;
                smart_desk_events.finger_detected = 1;
                time_finger_print_detected = xTaskGetTickCount();
                xQueueSend(sensor_queue, &msg, pdMS_TO_TICKS(10));
                break;
            }
            case ESP_ERR_NOT_FOUND: {
                msg.data = 2;
                smart_desk_events.finger_detected = 1;
                time_finger_print_detected = xTaskGetTickCount();
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

void identify_task_init(void) {
    xTaskCreate(identify_task, "IdentifyTask", 4096, NULL, PRIO_IDLE, NULL);
}