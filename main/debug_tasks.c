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
    
    ESP_LOGI("FINGERPRINT", "Waiting for finger...");

    while (1) {

        // Turn solid purple to indicate ready
        ESP_LOGI(TAG, "Trying to set LED to purple...");
        set_led(true, R503_LED_ALWAYS_ON, 0xFF, R503_LED_COLOR_RED, 0x00);
        ESP_LOGI(TAG, "set_led() complete");

        printf("Inside of loop\n");
        if (generate_image() == ESP_OK) {
            generate_char_file_from_image(R503_CHAR_BUFFER_1);
            printf("Stuck at gen char\n");
            if (search_fingerprint(0, 100) == ESP_OK) {
                // Blue for success
                printf("Stuck at search\n");
                set_led(true, R503_LED_BREATHING, 0x20, R503_LED_COLOR_BLUE, 1);
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
        get_activated_force_sensors();

        printf("Force Sensor Activated: TL:%d TR:%d BL:%d BR:%d | Raw: TL:%d TR:%d BL:%d BR:%d\n",
            smart_desk_events.tl_sensor_activated,
            smart_desk_events.tr_sensor_activated,
            smart_desk_events.bl_sensor_activated,
            smart_desk_events.br_sensor_activated,
            tl_raw,
            tr_raw,
            bl_raw,
            br_raw);

        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

void temp_sensor_test_task(void *pvParam) {

    float temp_reading = -1000.0f;

    i2c_master_dev_handle_t temp1_handle; //, temp2_handle;

    // Assume bus_handle is already initialized
    mcp9808_init(get_i2c_bus_handle(), 0x19, &temp1_handle);
    //mcp9808_init(get_i2c_bus_handle(), 0x19, &temp2_handle);

    // float t1 = get_temperature_reading(temp1_handle);
    // float t2 = get_temperature_reading(temp2_handle);
    // ESP_LOGI(TAG, "Sensor 1 = %.2f°C | Sensor 2 = %.2f°C", t1, t2);

    vTaskDelay(pdMS_TO_TICKS(250));

    while (1) {
        temp_reading = get_temperature_reading(temp1_handle);
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

    xTaskCreate(adc_reader_task, "adc_reader", 4096, NULL, 5, NULL);

    vTaskDelay(pdMS_TO_TICKS(500));

    // Print menu
    printf("\n*** SMART DESK DEBUG MENU ***\n");
    printf("1. Run Fingerprint Test\n");
    printf("2. Run Force Sensor Test\n");
    printf("3. Run Temp Sensor Test\n");
    printf("4. Run Fan Test\n");
    printf("Enter selection: ");

    // Wait for input
    char c = 0;
    while (c < '1' || c > '4') {
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
        default:
            printf("Invalid input\n");
            break;
    }
}