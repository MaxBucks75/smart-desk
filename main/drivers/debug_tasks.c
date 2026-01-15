/******************************************************************************
 * @file    debug_tasks.c
 * @brief   Implements all debug/testing FreeRTOS tasks for the Smart Desk
 * @author  Max Bucks
 * @date    2026-01-13
 *****************************************************************************/

#include "debug_tasks.h"

#define TAG "DEBUG"

// Raw force sensor variables
uint16_t tl_raw, tr_raw, bl_raw, br_raw;

/**
 * @brief Tests the fingerprint sensor's auto identify function.
 *
 * This task reads system parameters, attempts automatic identification,
 * then updates the R503 LED accordingly. If the fingerprint is recognized,
 * the motherboard power pins are shorted.
 * 
 * @param pvParameter 
 */
void fingerprint_power_task(void *pvParam) {
    
    esp_err_t err;
    err = read_system_parameters();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "read systen params is broken");
    }

    while (1) {

        err = auto_identify();

        ESP_LOGI(TAG, "Checking if buffer 1 & 2 match 00 = match, 01 = err receiving, 08 = bufs don't match");
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Shorting motherboard pins");
            set_led(R503_LED_ALWAYS_ON, 0x80, R503_LED_COLOR_RED, 0x00);
            short_mobo_power_pins();
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
        set_led(R503_LED_ALWAYS_ON, 0x80, R503_LED_COLOR_PURPLE, 0x00);

    }

}

/**
 * @brief Periodically reads force sensors and reports activation state.
 *
 * This debug task reads ADC values for the four corner force sensors,
 * runs the detection logic that updates 'smart_desk_events', and
 * prints both the processed activation flags and raw sensor values
 * to the serial console for debugging and calibration.
 *
 * @param pvParam Unused FreeRTOS task parameter.
 */
void force_sensor_test_task(void *pvParam) {

    while (1) {

        adc_reader();

        vTaskDelay(pdMS_TO_TICKS(50));

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

/**
 * @brief Reads and prints temperature sensor readings.
 *
 * This task polls the central and exhaust temperature sensors over I2C,
 * prints the readings (or errors) to the console, and delays between
 * samples. Useful for validating I2C connectivity and sensor values.
 *
 * @param pvParam Unused FreeRTOS task parameter.
 */
void temp_sensor_test_task(void *pvParam) {

    float temp_reading_central = -1000.0f;
    float temp_reading_exhaust = -1000.0f;

    check_i2c_address();

    while (1) {

        temp_reading_central = get_temperature_reading(temp_handle_central);
        if (temp_reading_central != -1000.0f) {
            printf("Ambient Temp Central: %.4f °C\n", temp_reading_central);
        } else {
            printf("Failed to read temperature\n");
        }
        vTaskDelay(pdMS_TO_TICKS(500));
        temp_reading_exhaust = get_temperature_reading(temp_handle_exhaust);
        if (temp_reading_exhaust != -1000.0f) {
            printf("Ambient Temp Exhaust: %.4f °C\n", temp_reading_exhaust);
        } else {
            printf("Failed to read temperature\n");
        }
        vTaskDelay(pdMS_TO_TICKS(500));

    }

}



/**
 * @brief Exercise fan control logic by varying the temperature inputs.
 *
 * The task reads temperature sensors and calls 'fan_set_speed()' to
 * exercise fan ramp-up and ramp-down logic. It also simulates a high
 * temperature condition to force fans to full speed for testing.
 *
 * @param pvParam Unused FreeRTOS task parameter.
 */
void fan_test_task(void *pvParam) {

    check_i2c_address();

    while (1) {

        float curr_temp_central = get_temperature_reading(temp_handle_central);
        float curr_temp_exhaust = get_temperature_reading(temp_handle_exhaust);

        fan_set_speed(&curr_temp_central, &curr_temp_exhaust);

        vTaskDelay(pdMS_TO_TICKS(10000));

        curr_temp_exhaust = 100.0;

        fan_set_speed(&curr_temp_central, &curr_temp_exhaust);

        vTaskDelay(pdMS_TO_TICKS(5000));
        
    }
}

/**
 * @brief Interactive actuator test driven by UART input.
 *
 * Prompts the user over UART to press 'c' to toggle the linear actuator
 * (extend/retract) and drives 'control_usb_hub()' while the actuator is
 * moving. Useful for manually verifying actuator motion and limits.
 *
 * @param pvParam Unused FreeRTOS task parameter.
 */
void actuator_test_task(void *pvParam) {

    while (1) {

        // printf("\n*** ENTER 'c' TO EXTEND/RETRACT LINEAR ACTUATOR ***\n");
        // char c = 0;
        // while (c != 'c') {
        //     int len = uart_read_bytes(UART_NUM_0, (uint8_t*)&c, 1, pdMS_TO_TICKS(5000));
        //     if (len > 0) {
        //         printf("%c\n", c);
        //     }
        // }

        // control_usb_hub();

        // while (smart_desk_events.usb_hub_moving == 1) {
        //     control_usb_hub();
        //     vTaskDelay(pdMS_TO_TICKS(100));
        // }
        
    }

}

/**
 * @brief Ramp and print vibrator PWM strength values.
 *
 * Cycles the vibration strength through PWM values and calls
 * 'vibrator_set_strength()' while printing the current PWM level to
 * the console. Used to validate PWM output and the vibrator motor.
 *
 * @param pvParam Unused FreeRTOS task parameter.
 */
void vibrator_test_task(void *pvParam) {

    // uint8_t vibration_strength = 0;

    while (1) {

        // vibration_strength++;
        // vibration_strength = vibration_strength % 0xFF;

        // vibrator_set_strength(vibration_strength);
        // printf("PWM signal: %d\n", vibration_strength);

        // vTaskDelay(pdMS_TO_TICKS(100));

    }

}

/**
 * @brief Monitor sensor state and short motherboard power pins for testing.
 *
 * This task periodically checks the desk position sensors and, when the
 * top-left and top-right sensors indicate a specific condition, it
 * performs a motherboard power pin short to exercise the power control
 * path during debug sessions.
 *
 * @param pvParam Unused FreeRTOS task parameter.
 */
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

/**
 * @brief Runs fingerprint enrollment loop for manual testing.
 *
 * Reads system parameters and repeatedly calls the enrollment routine
 * 'auto_enroll'. On successful enrollment the R503 LED is set to a
 * breathing purple pattern for visual confirmation.
 *
 * @param pvParam Unused FreeRTOS task parameter.
 */
void fingerprint_enroll_task(void *pvParam) {

    esp_err_t err;
    err = read_system_parameters();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "read systen params is broken");
    }

    while (1) {

        err = auto_enroll();
        if (err == ESP_OK) {
            set_led(R503_LED_BREATHING, 0x40, R503_LED_COLOR_PURPLE, 0x00);
            vTaskDelay(pdMS_TO_TICKS(60000));
        }

    }

}

/**
 * @brief Initializes debug UART and presents an interactive test menu.
 *
 * Configures the debug UART, prints the debug menu to the console, and
 * creates the selected debug FreeRTOS task. This function is intended
 * to be called from a development/debug entrypoint to exercise hardware
 * subsystems individually.
 */
void init_debug_tasks(void) {

     uart_config_t uart_config = {
        .baud_rate = 115200,
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
    printf("1. Run Fingerprint Power Test\n");
    printf("2. Run Force Sensor Test\n");
    printf("3. Run Temp Sensor Test\n");
    printf("4. Run Fan Test\n");
    printf("5. Run Actuator Test (not active)\n");
    printf("6. Run Vibrator Test (not active)\n");
    printf("7. Run Power Test\n");
    printf("8. Run Enroll Test\n");
    printf("Enter selection: ");

    // Wait for input
    char c = '0';
    while (c < '1' || c > '9') {
        int len = uart_read_bytes(UART_NUM_0, (uint8_t*)&c, 1, pdMS_TO_TICKS(5000));
        if (len > 0) {
            printf("%c\n", c);
        }
    }

    // Create selected task
    switch (c) {
        case '1':
            xTaskCreate(fingerprint_power_task, "fp_test", 6144, NULL, 5, NULL);
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
        // case '5':
        //     xTaskCreate(actuator_test_task, "actuator_test", 4096, NULL, 5, NULL);
        //     break;
        // case '6':
        //     xTaskCreate(vibrator_test_task, "vibrator_test", 4096, NULL, 5, NULL);
        //     break;
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