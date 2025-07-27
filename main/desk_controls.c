#include <stdint.h>
#include "esp_adc/adc_continuous.h"
#include "desk_controls.h"
#include "events.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "ADC_CONT"

// ADC1 GPIO 32 - 39
// ADC2 GPIO0, GPIO2, GPIO4, GPIO12 - GPIO15, GOIO25 - GPIO27
// ADC docs: https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32/api-reference/peripherals/adc.html
// TODO set the GPIO pin channels in io.h

// https://esp32io.com/tutorials/esp32-force-sensor

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
        .max_store_buf_size = 2048,
        .conv_frame_size = READ_LEN,
        .flags = {
            .flush_pool = 0,
        },
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_cfg, &adc_handle));

    adc_digi_pattern_config_t pattern[4];
    for (int i = 0; i < 4; i++) {
        pattern[i].atten = EXAMPLE_ADC_ATTEN;
        pattern[i].channel = channels[i];
        pattern[i].unit = EXAMPLE_ADC_UNIT;
        pattern[i].bit_width = EXAMPLE_ADC_BIT_WIDTH;
    }

    adc_continuous_config_t adc_config = {
        .sample_freq_hz = SOC_ADC_SAMPLE_FREQ_THRES_HIGH,
        .conv_mode = EXAMPLE_CONV_MODE,
        .format = EXAMPLE_OUTPUT_TYPE,
        .pattern_num = 4,
        .adc_pattern = pattern,
    };

    ESP_ERROR_CHECK(adc_continuous_config(adc_handle, &adc_config));
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle));
}

void adc_reader_task(void *arg) {
    uint8_t result[READ_LEN] = {0};
    uint32_t ret_num = 0;
    adc_digi_output_data_t *data = NULL;

    while (1) {
        esp_err_t ret = adc_continuous_read(adc_handle, result, READ_LEN, &ret_num, 1000);
        if (ret == ESP_OK) {
            for (int i = 0; i < ret_num; i += sizeof(adc_digi_output_data_t)) {
                data = (void *)&result[i];
                if (data->type2.unit == 0) {
                    //ESP_LOGI(TAG, "CH-%d: %d", data->type2.channel, data->type2.data);
                    switch (data->type2.channel) {
                        case ADC_CHANNEL_0: tl_raw = data->type2.data; break; // GPIO36
                        case ADC_CHANNEL_3: tr_raw = data->type2.data; break; // GPIO39
                        case ADC_CHANNEL_6: bl_raw = data->type2.data; break; // GPIO34
                        case ADC_CHANNEL_7: br_raw = data->type2.data; break; // GPIO35
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));
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
            smart_desk_events.move_desk_down = 1;
        }
    // Reset time and update flag as lower desk input is no longer detected
    } else {
        smart_desk_events.move_desk_down = 0;
        desk_lower_detected_time = 0;
    }

    if (raise_desk_input_detected) {
        if (desk_raise_detected_time == 0) {
            desk_raise_detected_time = now;
        } else if (((now - desk_raise_detected_time) >= pdMS_TO_TICKS(DEBOUNCE_DELAY_MS))) {
            smart_desk_events.move_desk_up = 1;
        }
    } else {
        smart_desk_events.move_desk_up = 0;
        desk_raise_detected_time = 0;
    }

}

// TODO write update height function
void change_desk_height(void) {}

// TODO write open/close functionality for the USB hub and linear actuator
void open_usb_hub(void) {}