#include "desk_controls.h"
#include "esp_log.h"

#define TAG "ADC_CONT"

// ADC1 GPIO 32 - 39
// ADC2 GPIO0, GPIO2, GPIO4, GPIO12 - GPIO15, GOIO25 - GPIO27
// ADC docs: https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32/api-reference/peripherals/adc.html
// TODO set the GPIO pin channels in io.h

// https://esp32io.com/tutorials/es p32-force-sensor

static int desk_raise_detected_time = 0;
static int desk_lower_detected_time = 0;

volatile uint16_t tl_raw = 0;
volatile uint16_t tr_raw = 0;
volatile uint16_t bl_raw = 0;
volatile uint16_t br_raw = 0;

static const adc_channel_t channels[] = {
    ADC_CHANNEL_0,  // GPIO36 - TL
    ADC_CHANNEL_3,  // GPIO39 - TR
    ADC_CHANNEL_6,  // GPIO34 - BL
    ADC_CHANNEL_7   // GPIO35 - BR
};

adc_continuous_handle_t adc_handle;

void adc_continuous_init(void) {
    adc_continuous_handle_cfg_t handle_cfg = {
        .max_store_buf_size = 64,  
        .conv_frame_size = 32,       
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_cfg, &adc_handle));

    adc_digi_pattern_config_t pattern[4];
    for (int i = 0; i < 4; i++) {
        pattern[i].atten = ADC_ATTEN;
        pattern[i].channel = channels[i];
        pattern[i].unit = ADC_UNIT;
        pattern[i].bit_width = ADC_BIT_WIDTH;
    }

    adc_continuous_config_t adc_config = {
        .sample_freq_hz = SOC_ADC_SAMPLE_FREQ_THRES_HIGH,
        .conv_mode = CONV_MODE,
        .format = OUTPUT_TYPE,
        .pattern_num = 4,
        .adc_pattern = pattern,
    };

    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &adc_config));
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
}

void adc_reader_task(void *arg) {

    uint8_t result[64];
    uint32_t output_length = 0;

    while (1) {
        esp_err_t ret = adc_continuous_read(adc_handle, result, sizeof(result), &output_length, 100);
        if (ret == ESP_OK && output_length > 0) {
            for (int i = 0; i < output_length; i += sizeof(adc_digi_output_data_t)) {
                adc_digi_output_data_t *data = (adc_digi_output_data_t *)&result[i];
                switch (data->type1.channel) {
                    case 0: tl_raw = data->type1.data; break;
                    case 3: tr_raw = data->type1.data; break;
                    case 6: bl_raw = data->type1.data; break;
                    case 7: br_raw = data->type1.data; break;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void get_activated_force_sensors(void) {
    smart_desk_events.tl_sensor_activated = (tl_raw > FORCE_SENSOR_ACTIVATED_THRESHOLD);
    smart_desk_events.tr_sensor_activated = (tr_raw > FORCE_SENSOR_ACTIVATED_THRESHOLD);
    smart_desk_events.bl_sensor_activated = (bl_raw > FORCE_SENSOR_ACTIVATED_THRESHOLD);
    smart_desk_events.br_sensor_activated = (br_raw > FORCE_SENSOR_ACTIVATED_THRESHOLD);
}


void detect_raising_or_lowering(void) {

    // Read force sensor activation and update flags
    get_activated_force_sensors();

    // If both the top sensors are active, lower the desk. If both the bottom sensors are active raise the desk
    bool raise_desk_input_detected = (smart_desk_events.tl_sensor_activated && smart_desk_events.tr_sensor_activated && 
                                    !smart_desk_events.bl_sensor_activated && !smart_desk_events.br_sensor_activated);
    bool lower_desk_input_detected = (!smart_desk_events.tl_sensor_activated && !smart_desk_events.tr_sensor_activated && 
                                    smart_desk_events.bl_sensor_activated && smart_desk_events.br_sensor_activated);

    TickType_t now = xTaskGetTickCount(); // Read the time

    // Debouncing logic for desk height
    if (lower_desk_input_detected) {
        // Update activation tick
        if (desk_lower_detected_time == 0) {
            desk_lower_detected_time = now;
        // Check if the ticks since first detection is greater than the debounce threshold
        } else if (((now - desk_lower_detected_time) >= pdMS_TO_TICKS(DEBOUNCE_DELAY_MS))) {
            start_lowering_desk();
            vibrator_set_strength(200);
            //ESP_LOGI(TAG, "Lowering Desk...");
        }

    } else if (raise_desk_input_detected) {
        if (desk_raise_detected_time == 0) {
            desk_raise_detected_time = now;
        } else if (((now - desk_raise_detected_time) >= pdMS_TO_TICKS(DEBOUNCE_DELAY_MS))) {
            start_raising_desk();
            vibrator_set_strength(200);
            //ESP_LOGI(TAG, "Raising Desk...");
        }
    // Reset time and update flag as lower desk input is no longer detected
    } else {
        stop_raising_desk();
        stop_lowering_desk();
        vibrator_set_strength(0);
        desk_raise_detected_time = 0;
        desk_lower_detected_time = 0;
    }

}

void linear_actuator_init(void) {
    // Direction pins
    gpio_reset_pin(LINEAR_ACTUATOR_IN1);
    gpio_reset_pin(LINEAR_ACTUATOR_IN2);
    gpio_set_direction(LINEAR_ACTUATOR_IN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(LINEAR_ACTUATOR_IN2, GPIO_MODE_OUTPUT);

    // PWM for ENA
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_8_BIT,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = 1000,
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .gpio_num   = LINEAR_ACTUATOR_ENA,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0,
    };
    ledc_channel_config(&ledc_channel);
}

static actuator_ctx_t actuator = {
    .start_time = 0,
    .move_duration = 0
};

void control_usb_hub(void) {
    TickType_t now = xTaskGetTickCount();

    if (smart_desk_events.usb_hub_moving) {

        // Read the time and check if the move duration has passed
        if ((now - actuator.start_time) >= actuator.move_duration) {
            smart_desk_events.usb_hub_extended = !smart_desk_events.usb_hub_extended;

            // Stop the actuator
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            gpio_set_level(LINEAR_ACTUATOR_IN1, 0);
            gpio_set_level(LINEAR_ACTUATOR_IN2, 0);

            smart_desk_events.usb_hub_moving = 0;

        }

    } else {

        smart_desk_events.usb_hub_moving = 1;

        // Toggle state
        if (smart_desk_events.usb_hub_extended) {
            // Retract
            gpio_set_level(LINEAR_ACTUATOR_IN1, 0);
            gpio_set_level(LINEAR_ACTUATOR_IN2, 1);
        } else {
            // Extend
            gpio_set_level(LINEAR_ACTUATOR_IN1, 1);
            gpio_set_level(LINEAR_ACTUATOR_IN2, 0);
        }
        // extend/retract at full speed: duty = 255
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 255);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

        // TODO: Add vibration feedback funtion here

        actuator.start_time = now;
        actuator.move_duration = pdMS_TO_TICKS(1700);
    
    }

}

void relay_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RELAY_LINEAR_ACTUATOR_PIN) |
                        (1ULL << RELAY_DESK_RAISE_PIN) |
                        (1ULL << RELAY_DESK_LOWER_PIN) |
                        (1ULL << RELAY_MOBO_POWER),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    gpio_config(&io_conf);

    // Set default LOW states
    gpio_set_level(RELAY_LINEAR_ACTUATOR_PIN, 0);
    gpio_set_level(RELAY_DESK_RAISE_PIN, 0);
    gpio_set_level(RELAY_DESK_LOWER_PIN, 0);
    gpio_set_level(RELAY_MOBO_POWER, 0);

    // Initialize ESP32 LED for powering on MOBO
    gpio_reset_pin(ONBOARD_LED);
    gpio_set_direction(ONBOARD_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(ONBOARD_LED, 0); // start OFF

}

void extend_actuator(void) {
    gpio_set_level(RELAY_LINEAR_ACTUATOR_PIN, 1);   // HIGH = energize relay (depends on module)
}

void retract_actuator(void) {
    gpio_set_level(RELAY_LINEAR_ACTUATOR_PIN, 0);   // LOW = release relay
}

void start_raising_desk(void) {
    gpio_set_level(RELAY_DESK_RAISE_PIN, 1);
}

void stop_raising_desk(void) {
    gpio_set_level(RELAY_DESK_RAISE_PIN, 0);
}

void start_lowering_desk(void) {
    gpio_set_level(RELAY_DESK_LOWER_PIN, 1);
}

void stop_lowering_desk(void) {
    gpio_set_level(RELAY_DESK_LOWER_PIN, 0);
}

void short_mobo_power_pins(void) {
    gpio_set_level(ONBOARD_LED, 1);
    gpio_set_level(RELAY_MOBO_POWER, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(RELAY_MOBO_POWER, 0);
    gpio_set_level(ONBOARD_LED, 0);
}

void vibrator_init(void) {
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_8_BIT,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = 200,              // vibration frequency
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .gpio_num   = VIBRATOR_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_1,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,  // start off
        .hpoint     = 0,
    };
    ledc_channel_config(&ledc_channel);
}

void vibrator_set_strength(uint8_t duty) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}