/******************************************************************************
 * @file    height_control_task.c
 * @brief   Task that samples force sensors and maintains baseline readings
 * @author  Max Bucks
 * @date    2026-01-14
 *****************************************************************************/

#include "height_control_task.h"

#define TAG "HeightControlTask"

static const TickType_t HEIGHT_SAMPLE_PERIOD = pdMS_TO_TICKS(50);

/**
 * @brief Periodically samples ADC force sensors and updates baselines.
 *
 * This task performs ADC sampling, updates filtered/baseline values when
 * stable, and calls 'detect_raising_or_lowering()' to apply motion logic.
 */
void height_control_task(void* pvParam) {

    TickType_t      last_wake      = xTaskGetTickCount();
    TickType_t      tl_stable_time = 0;
    TickType_t      tr_stable_time = 0;
    TickType_t      bl_stable_time = 0;
    TickType_t      br_stable_time = 0;

    for (;;) {


        // Sample ADC, update raw values
        adc_reader();

        // Check if each sensor's filtered reading is within the stable threshold 
        // Then increment or zero stable time accordingly
        if (fabsf((float)tl_filtered - (float)tl_baseline) < STABLE_THRESHOLD) {
            tl_stable_time += HEIGHT_SAMPLE_PERIOD;
        } else {
            tl_stable_time = 0;
        }
        if (fabsf((float)tr_filtered - (float)tr_baseline) < STABLE_THRESHOLD) {
            tr_stable_time += HEIGHT_SAMPLE_PERIOD;
        } else {
            tr_stable_time = 0;
        }
        if (fabsf((float)bl_filtered - (float)bl_baseline) < STABLE_THRESHOLD) {
            bl_stable_time += HEIGHT_SAMPLE_PERIOD;
        } else {
            bl_stable_time = 0;
        }
        if (fabsf((float)br_filtered - (float)br_baseline) < STABLE_THRESHOLD) {
            br_stable_time += HEIGHT_SAMPLE_PERIOD;
        } else {
            br_stable_time = 0;
        }

        // If any sensor has been stable for long enough, update it's baseline reading
        if (tl_stable_time > STABLE_TIME_TICKS) {
            tl_baseline += BASELINE_ALPHA * (tl_filtered - tl_baseline);
        }
        if (tr_stable_time > STABLE_TIME_TICKS) {
            tr_baseline += BASELINE_ALPHA * (tr_filtered - tr_baseline);
        }
        if (bl_stable_time > STABLE_TIME_TICKS) {
            bl_baseline += BASELINE_ALPHA * (bl_filtered - bl_baseline);
        }
        if (br_stable_time > STABLE_TIME_TICKS) {
            br_baseline += BASELINE_ALPHA * (br_filtered - br_baseline);
        }

        // Check if the conditions are met to lower or raise the desk every ~50 ms
        detect_raising_or_lowering();
        vTaskDelayUntil(&last_wake, HEIGHT_SAMPLE_PERIOD);
    }
}

/**
 * @brief Create the height control FreeRTOS task.
 */
void height_control_task_init(void) {
    xTaskCreate(height_control_task, "HeightControlTask", 4096, NULL, PRIO_HEIGHT_CONTROL, NULL);
}