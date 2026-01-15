/******************************************************************************
 * @file    desk_controls.c
 * @brief   Implements desk control logic and sensor handling for Smart Desk
 * @author  Max Bucks
 * @date    2026-01-14
 *****************************************************************************/

#include "desk_controls.h"

#define TAG "ADC_CONT"

static int desk_raise_detected_time = 0;
static int desk_lower_detected_time = 0;

// Global sensor calibration variables
uint16_t tl_raw = 0;
uint16_t tr_raw = 0;
uint16_t bl_raw = 0;
uint16_t br_raw = 0;

uint16_t tl_filtered = 0;
uint16_t tr_filtered = 0;
uint16_t bl_filtered = 0;
uint16_t br_filtered = 0;

uint16_t tl_baseline = 0;
uint16_t tr_baseline = 0;
uint16_t bl_baseline = 0;
uint16_t br_baseline = 0;

static const adc_channel_t channels[] = {
    TL_FORCE_SENSOR_CH, // GPIO39 CH3
    TR_FORCE_SENSOR_CH, // GPIO36 CH0
    BL_FORCE_SENSOR_CH, // GPIO34 CH6
    BR_FORCE_SENSOR_CH  // GPIO35 CH7
};

adc_continuous_handle_t adc_handle;

/**
 * @brief Configure and start ADC continuous sampling.
 *
 * Creates the ADC continuous handle, configures sampling pattern for the
 * 4 force sensors, starts the ADC engine, and initializes filtered/baseline
 * readings by performing an initial read.
 */
void adc_continuous_init(void) {

    adc_continuous_handle_cfg_t handle_cfg = {
        .max_store_buf_size = 64,
        .conv_frame_size    = 32,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_cfg, &adc_handle));

    adc_digi_pattern_config_t pattern[4];
    for (int i = 0; i < 4; i++) {
        pattern[i].atten     = ADC_ATTEN;
        pattern[i].channel   = channels[i];
        pattern[i].unit      = ADC_UNIT;
        pattern[i].bit_width = ADC_BIT_WIDTH;
    }

    adc_continuous_config_t adc_config = {
        .sample_freq_hz = SOC_ADC_SAMPLE_FREQ_THRES_HIGH,
        .conv_mode      = CONV_MODE,
        .format         = OUTPUT_TYPE,
        .pattern_num    = 4,
        .adc_pattern    = pattern,
    };

    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &adc_config));
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));

    // Read current adc values to initialize filtered and baseline readings
    adc_reader();

    tr_filtered = tr_raw;
    tl_filtered = tl_raw;
    br_filtered = br_raw;
    bl_filtered = bl_raw;

    tr_baseline = tr_raw;
    tl_baseline = tl_raw;
    br_baseline = br_raw;
    bl_baseline = bl_raw;

}

/**
 * @brief Read ADC continuous buffer and update raw/filtered values.
 *
 * Reads a frame from the ADC continuous driver, extracts per-channel raw
 * values into the global raw variables and applies a simple exponential
 * filter to update filtered sensor readings.
 */
void adc_reader(void) {

    uint8_t  result[64];
    uint32_t output_length = 0;

    // Read the adc values of all 4 channels into result array
    esp_err_t ret = adc_continuous_read(adc_handle, result, sizeof(result), &output_length, 10);
    if (ret == ESP_OK && output_length > 0) {
        for (int i = 0; i < output_length; i += sizeof(adc_digi_output_data_t)) {

            // Pull raw data from result and update global raw variables accordingly
            adc_digi_output_data_t* data = (adc_digi_output_data_t*)&result[i];
            switch (data->type1.channel) {
            case 0:
                tr_raw = data->type1.data;
                break;
            case 3:
                tl_raw = data->type1.data;
                break;
            case 6:
                bl_raw = data->type1.data;
                break;
            case 7:
                br_raw = data->type1.data;
                break;
            }
        }

        // Compute filtered value to remove noise from raw readings
        tr_filtered += (uint16_t)(FORCE_ALPHA * (tr_raw - tr_filtered));
        tl_filtered += (uint16_t)(FORCE_ALPHA * (tl_raw - tl_filtered));
        br_filtered += (uint16_t)(FORCE_ALPHA * (br_raw - br_filtered));
        bl_filtered += (uint16_t)(FORCE_ALPHA * (bl_raw - bl_filtered));

    } else {
        ESP_LOGE(TAG, "Timeout when running adc read");
    }

}

/**
 * @brief Update 'smart_desk_events' activation flags from filtered values.
 *
 * Compares filtered sensor readings against calibrated baselines and sets
 * the corresponding 'smart_desk_events.*_sensor_activated' flags.
 */
void get_activated_force_sensors(void) {
    smart_desk_events.tl_sensor_activated =
        ((tl_baseline - tl_filtered) >= TL_FORCE_SENSOR_ACTIVATED_THRESHOLD);
    smart_desk_events.tr_sensor_activated =
        ((tr_baseline - tr_filtered) >= TR_FORCE_SENSOR_ACTIVATED_THRESHOLD);
    smart_desk_events.bl_sensor_activated =
        ((bl_baseline - bl_filtered) >= BL_FORCE_SENSOR_ACTIVATED_THRESHOLD);
    smart_desk_events.br_sensor_activated =
        ((br_baseline - br_filtered) >= BR_FORCE_SENSOR_ACTIVATED_THRESHOLD);
}

/**
 * @brief Detect raise/lower gestures from force sensors and debounce inputs.
 *
 * Calls 'get_activated_force_sensors()' then determines whether the top or
 * bottom pair of sensors indicate a raise or lower command. Debouncing is
 * applied using 'DEBOUNCE_DELAY_MS' before triggering motion functions.
 */
void detect_raising_or_lowering(void) {

    // Read force sensor activation and update flags
    get_activated_force_sensors();

    // If top sensors are active, raise desk. If bottom sensors are active, lower desk
    bool lower_desk_input_detected =
        (smart_desk_events.tl_sensor_activated && smart_desk_events.tr_sensor_activated &&
         !smart_desk_events.bl_sensor_activated && !smart_desk_events.br_sensor_activated);
    bool raise_desk_input_detected =
        (!smart_desk_events.tl_sensor_activated && !smart_desk_events.tr_sensor_activated &&
         smart_desk_events.bl_sensor_activated && smart_desk_events.br_sensor_activated);

    // Update global flag
    smart_desk_events.desk_moving = (raise_desk_input_detected | lower_desk_input_detected);

    TickType_t now = xTaskGetTickCount(); // Read the time

    // Debouncing logic for desk height
    if (lower_desk_input_detected) {
        // Update activation tick
        if (desk_lower_detected_time == 0) {
            desk_lower_detected_time = now;
            // Check if the ticks since first detection is greater than the debounce threshold
        } else if (((now - desk_lower_detected_time) >= pdMS_TO_TICKS(DEBOUNCE_DELAY_MS))) {
            start_lowering_desk();
            smart_desk_events.desk_moving = 1;
            // vibrator_set_strength(200);
            ESP_LOGI(TAG, "Lowering Desk...");
        }

    } else if (raise_desk_input_detected) {
        if (desk_raise_detected_time == 0) {
            desk_raise_detected_time = now;
        } else if (((now - desk_raise_detected_time) >= pdMS_TO_TICKS(DEBOUNCE_DELAY_MS))) {
            start_raising_desk();
            smart_desk_events.desk_moving = 1;
            // vibrator_set_strength(200);
            ESP_LOGI(TAG, "Raising Desk...");
        }
        // Reset time and update flag as lower desk input is no longer detected
    } else {
        stop_raising_desk();
        stop_lowering_desk();
        // vibrator_set_strength(0);
        desk_raise_detected_time      = 0;
        desk_lower_detected_time      = 0;
        smart_desk_events.desk_moving = 0;
    }

}

// Add this back for linear actuator (1ULL << RELAY_LINEAR_ACTUATOR_PIN) |
/**
 * @brief Initialize relay GPIOs used for desk motion and power control.
 */
void relay_init(void) {

    gpio_config_t io_conf = {.pin_bit_mask = (1ULL << RELAY_DESK_RAISE_PIN) |
                                             (1ULL << RELAY_DESK_LOWER_PIN) |
                                             (1ULL << RELAY_MOBO_POWER),
                             .mode         = GPIO_MODE_OUTPUT,
                             .pull_up_en   = GPIO_PULLUP_DISABLE,
                             .pull_down_en = GPIO_PULLDOWN_DISABLE,
                             .intr_type    = GPIO_INTR_DISABLE};

    gpio_config(&io_conf);

    // Set default LOW states
    // gpio_set_level(RELAY_LINEAR_ACTUATOR_PIN, 0);
    gpio_set_level(RELAY_DESK_RAISE_PIN, 0);
    gpio_set_level(RELAY_DESK_LOWER_PIN, 0);
    gpio_set_level(RELAY_MOBO_POWER, 0);

    // Initialize ESP32 LED for powering on MOBO
    gpio_reset_pin(ONBOARD_LED);
    gpio_set_direction(ONBOARD_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(ONBOARD_LED, 0); // start OFF

}

void start_raising_desk(void) { gpio_set_level(RELAY_DESK_RAISE_PIN, 1); }

/**
 * @brief Deassert relay to stop raising the desk.
 */
void stop_raising_desk(void) { gpio_set_level(RELAY_DESK_RAISE_PIN, 0); }

/**
 * @brief Assert relay to start lowering the desk.
 */
void start_lowering_desk(void) { gpio_set_level(RELAY_DESK_LOWER_PIN, 1); }

/**
 * @brief Deassert relay to stop lowering the desk.
 */
void stop_lowering_desk(void) { gpio_set_level(RELAY_DESK_LOWER_PIN, 0); }

/**
 * @brief Pulse motherboard power pins to simulate a power button press.
 */
void short_mobo_power_pins(void) {
    gpio_set_level(ONBOARD_LED, 1);
    gpio_set_level(RELAY_MOBO_POWER, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(RELAY_MOBO_POWER, 0);
    gpio_set_level(ONBOARD_LED, 0);
}

// Completed code that I didn't physically implement into the desk:

// void linear_actuator_init(void) {

//     // Direction pins
//     gpio_reset_pin(LINEAR_ACTUATOR_IN1);
//     gpio_reset_pin(LINEAR_ACTUATOR_IN2);
//     gpio_set_direction(LINEAR_ACTUATOR_IN1, GPIO_MODE_OUTPUT);
//     gpio_set_direction(LINEAR_ACTUATOR_IN2, GPIO_MODE_OUTPUT);

//     // PWM for ENA
//     ledc_timer_config_t ledc_timer = {
//         .speed_mode      = LEDC_LOW_SPEED_MODE,
//         .duty_resolution = LEDC_TIMER_8_BIT,
//         .timer_num       = LEDC_TIMER_0,
//         .freq_hz         = 1000,
//         .clk_cfg         = LEDC_AUTO_CLK,
//     };
//     ledc_timer_config(&ledc_timer);

//     ledc_channel_config_t ledc_channel = {
//         .gpio_num   = LINEAR_ACTUATOR_ENA,
//         .speed_mode = LEDC_LOW_SPEED_MODE,
//         .channel    = LEDC_CHANNEL_0,
//         .timer_sel  = LEDC_TIMER_0,
//         .duty       = 0,
//         .hpoint     = 0,
//     };
//     ledc_channel_config(&ledc_channel);
//
// }

// static actuator_ctx_t actuator = {.start_time = 0, .move_duration = 0};

// void control_usb_hub(void) {

//     TickType_t now = xTaskGetTickCount();

//     if (smart_desk_events.usb_hub_moving) {

//         // Read the time and check if the move duration has passed
//         if ((now - actuator.start_time) >= actuator.move_duration) {
//             smart_desk_events.usb_hub_extended = !smart_desk_events.usb_hub_extended;

//             // Stop the actuator
//             ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
//             ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
//             gpio_set_level(LINEAR_ACTUATOR_IN1, 0);
//             gpio_set_level(LINEAR_ACTUATOR_IN2, 0);

//             smart_desk_events.usb_hub_moving = 0;
//         }

//     } else {

//         smart_desk_events.usb_hub_moving = 1;

//         // Toggle state
//         if (smart_desk_events.usb_hub_extended) {
//             // Retract
//             gpio_set_level(LINEAR_ACTUATOR_IN1, 0);
//             gpio_set_level(LINEAR_ACTUATOR_IN2, 1);
//         } else {
//             // Extend
//             gpio_set_level(LINEAR_ACTUATOR_IN1, 1);
//             gpio_set_level(LINEAR_ACTUATOR_IN2, 0);
//         }
//         // extend/retract at full speed: duty = 255
//         ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 255);
//         ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

//         actuator.start_time    = now;
//         actuator.move_duration = pdMS_TO_TICKS(1700);
//     }

// }

// void extend_actuator(void) {
//     gpio_set_level(RELAY_LINEAR_ACTUATOR_PIN, 1);   // HIGH = energize relay (depends on module)
// }

// void retract_actuator(void) {
//     gpio_set_level(RELAY_LINEAR_ACTUATOR_PIN, 0);   // LOW = release relay
// }

// void vibrator_init(void) {

//     ledc_timer_config_t ledc_timer = {
//         .speed_mode      = LEDC_LOW_SPEED_MODE,
//         .duty_resolution = LEDC_TIMER_8_BIT,
//         .timer_num       = LEDC_TIMER_0,
//         .freq_hz         = 200, // vibration frequency
//         .clk_cfg         = LEDC_AUTO_CLK,
//     };
//     ledc_timer_config(&ledc_timer);

//     ledc_channel_config_t ledc_channel = {
//         .gpio_num   = VIBRATOR_PIN,
//         .speed_mode = LEDC_LOW_SPEED_MODE,
//         .channel    = LEDC_CHANNEL_1,
//         .timer_sel  = LEDC_TIMER_0,
//         .duty       = 0, // start off
//         .hpoint     = 0,
//     };
//     ledc_channel_config(&ledc_channel);

// }

// void vibrator_set_strength(uint8_t duty) {
//     ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
//     ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
// }