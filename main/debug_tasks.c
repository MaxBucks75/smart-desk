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
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "semaphore.h"

#include <stdio.h>

#define DEBUG_UART UART_NUM_0

#define TAG "DEBUG"

extern volatile uint16_t tl_raw;
extern volatile uint16_t tr_raw;
extern volatile uint16_t bl_raw;
extern volatile uint16_t br_raw;

/**
 * @brief Tester for the fingerpint sensor
 * 
 * @param pvParameter 
 */
void fingerprint_test_task(void *pvParam) {
    
    esp_err_t err;
    err = set_led(true, R503_LED_ALWAYS_ON, 0xFF, R503_LED_COLOR_RED, 0x00);
    if (err != ESP_OK) {
        ESP_LOGE("FINGERPRINT", "set_led is broken");
    }

    ESP_LOGI("FINGERPRINT", "Waiting for finger...");

    while (1) {

        err = generate_char_file_from_image(0x01);
        while (generate_char_file_from_image(0x01) != ESP_OK) {

        }



        printf("Inside of loop\n");
        if (generate_image() == ESP_OK) {
            generate_char_file_from_image(R503_CHAR_BUFFER_1);
            printf("Stuck at gen char\n");
            if (search_fingerprint(0, 100) == ESP_OK) {
                // Blue for success
                printf("Stuck at search\n");
                set_led(true, R503_LED_BREATHING, 0x20, 0x05, 1);
                printf("cant set led blue\n");
            } else {
                // Red for failure
                set_led(true, R503_LED_BREATHING, 0x20, R503_LED_COLOR_RED, 1);
                printf("cant set led red\n");
            }
            vTaskDelay(pdMS_TO_TICKS(1500));
            set_led(true, R503_LED_ALWAYS_ON, 0x80, R503_LED_COLOR_PURPLE, 0x00);
            printf("cant set led purple\n");
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }

}

void force_sensor_test_task(void *pvParam) {

    while (1) {

        detect_raising_or_lowering();

        printf("Force Sensor Activated: TL:%d TR:%d BL:%d BR:%d | Raw: TL:%d TR:%d BL:%d BR:%d\n",
            smart_desk_events.tl_sensor_activated,
            smart_desk_events.tr_sensor_activated,
            smart_desk_events.bl_sensor_activated,
            smart_desk_events.br_sensor_activated,
            tl_raw,
            tr_raw,
            bl_raw,
            br_raw);

        vTaskDelay(pdMS_TO_TICKS(50));
        
    }

}

void temp_sensor_test_task(void *pvParam) {

    float temp_reading = -1000.0f;
    i2c_master_dev_handle_t temp_handle;
    uint8_t mcp9808_address = 0x19;
    mcp9808_init(get_i2c_bus_handle(), mcp9808_address, &temp_handle);

    while (1) {

        temp_reading = get_temperature_reading(temp_handle);
        if (temp_reading != -1000.0f) {
            printf("Ambient Temp: %.4f °C\n", temp_reading);
        } else {
            printf("Failed to read temperature\n");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));

    }

}



void fan_test_task(void *pvParam) {

    i2c_master_dev_handle_t temp_handle;
    mcp9808_init(get_i2c_bus_handle(), 0x19, &temp_handle);
    float start_temp = get_temperature_reading(temp_handle);
    ESP_LOGI("FAN_TEST", "Starting temp: %.4f °C", start_temp);

    while (1) {

        float curr_temp = get_temperature_reading(temp_handle);
        ESP_LOGI("FAN_TEST", "Current temp: %.4f °C", curr_temp);

        // Artificially max out or zero the current temp if I breathe on the sensor
        if (curr_temp > start_temp) {
            curr_temp = 100;
        } else {
            curr_temp = 0;
        }

        fan_set_speed(&curr_temp, &curr_temp);
        vTaskDelay(pdMS_TO_TICKS(1000));
        
    }
}

void actuator_test_task(void *pvParam) {

    while (1) {

        printf("\n*** ENTER 'c' TO EXTEND/RETRACT LINEAR ACTUATOR ***\n");
        char c = 0;
        while (c != 'c') {
            int len = uart_read_bytes(UART_NUM_0, (uint8_t*)&c, 1, pdMS_TO_TICKS(5000));
            if (len > 0) {
                printf("%c\n", c);
            }
        }

        control_usb_hub();

        while (smart_desk_events.usb_hub_moving == 1) {
            control_usb_hub();
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        
    }

}

void vibrator_test_task(void *pvParam) {

    uint8_t vibration_strength = 0;

    while (1) {

        vibration_strength++;
        vibration_strength = vibration_strength % 0xFF;

        vibrator_set_strength(vibration_strength);
        printf("PWM signal: %d\n", vibration_strength);

        vTaskDelay(pdMS_TO_TICKS(100));

    }

}

void power_test_task(void *pvParam) {

    while (1) {

        detect_raising_or_lowering();
        if (smart_desk_events.tl_sensor_activated && smart_desk_events.tr_sensor_activated) {
            short_mobo_power_pins();
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));

    }

}

void fingerprint_enroll_task(void *pvParam) {
    uint8_t slot = 1;

    ESP_LOGI("FINGERPRINT", "Starting manual enroll for slot %d", slot);
    set_led(true, R503_LED_ALWAYS_ON, 0x80, R503_LED_COLOR_PURPLE, 0x00);

    // First capture
    wait_for_finger_and_capture(R503_CHAR_BUFFER_1);
    ESP_LOGI("FINGERPRINT", "First scan done. Remove finger...");

    vTaskDelay(pdMS_TO_TICKS(2000));

    wait_for_finger_and_capture(R503_CHAR_BUFFER_2);
    ESP_LOGI("FINGERPRINT", "Second scan done.");

    if (register_model() == ESP_OK) {
        store_template(R503_CHAR_BUFFER_1, slot);
        ESP_LOGI("FINGERPRINT", "Enroll SUCCESS at slot %d", slot);
    } else {
        ESP_LOGE("FINGERPRINT", "Enroll FAILED");
    }

    // Combine and store
    if (register_model() != ESP_OK) {
        ESP_LOGE("FINGERPRINT", "Register model failed");
        goto fail;
    }
    if (store_template(R503_CHAR_BUFFER_1, slot) != ESP_OK) {
        ESP_LOGE("FINGERPRINT", "Store template failed");
        goto fail;
    }

    ESP_LOGI("FINGERPRINT", "Enroll SUCCESS at slot %d", slot);
    set_led(true, R503_LED_BREATHING, 0x40, R503_LED_COLOR_BLUE, 2);
    goto done;

fail:
    set_led(true, R503_LED_BREATHING, 0x40, R503_LED_COLOR_RED, 2);

done:
    vTaskDelay(pdMS_TO_TICKS(2000));
    set_led(true, R503_LED_ALWAYS_ON, 0x40, R503_LED_COLOR_PURPLE, 0x00);
    vTaskDelete(NULL);
}

void init_debug_tasks(void) {

     uart_config_t uart_config = {
        .baud_rate = 115200,  // match your monitor speed
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    ESP_ERROR_CHECK(uart_param_config(DEBUG_UART, &uart_config));
    ESP_ERROR_CHECK(uart_driver_install(DEBUG_UART, 1024, 0, 0, NULL, 0));

    vTaskDelay(pdMS_TO_TICKS(500));

    // Print menu
    printf("\n*** SMART DESK DEBUG MENU ***\n");
    printf("1. Run Fingerprint Test\n");
    printf("2. Run Force Sensor Test\n");
    printf("3. Run Temp Sensor Test\n");
    printf("4. Run Fan Test\n");
    printf("5. Run Actuator Test\n");
    printf("6. Run Vibrator Test\n");
    printf("7. Run Power Test\n");
    printf("8. Run Enroll Test\n");
    printf("Enter selection: ");

    // Wait for input
    char c = '0';
    while (c < '1' || c > '8') {
        int len = uart_read_bytes(UART_NUM_0, (uint8_t*)&c, 1, pdMS_TO_TICKS(5000));
        if (len > 0) {
            printf("%c\n", c);
        }
    }

    switch (c) {
        case '1':
            xTaskCreate(fingerprint_test_task, "fp_test", 6144, NULL, 5, NULL);
            break;
        case '2':
            xTaskCreate(force_sensor_test_task, "force_test", 4096, NULL, 5, NULL);
            break;
        case '3':
            xTaskCreate(temp_sensor_test_task, "temp_test", 4096, NULL, 5, NULL);
            break;
        case '4':
            xTaskCreate(fan_test_task, "fan_test", 4096, NULL, 5, NULL);
            break;
        case '5':
            xTaskCreate(actuator_test_task, "actuator_test", 4096, NULL, 5, NULL);
            break;
        case '6':
            xTaskCreate(vibrator_test_task, "vibrator_test", 4096, NULL, 5, NULL);
            break;
        case '7':
            xTaskCreate(power_test_task, "power_test", 4096, NULL, 5, NULL);
            break;
        case '8':
            xTaskCreate(fingerprint_enroll_task, "enroll_test", 4096, NULL, 5, NULL);
            break;
        default:
            printf("Invalid input\n");
            break;
    }
}