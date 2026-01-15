/******************************************************************************
 * @file    ipc.c
 * @brief   Inter-task IPC primitives (queues, mutex, shared structs)
 * @author  Max Bucks
 * @date    2026-01-14
 *****************************************************************************/

#include "ipc.h"

QueueHandle_t sensor_queue  = NULL;
QueueHandle_t control_queue = NULL;

// Instantiate global structs
volatile smart_desk_events_t    smart_desk_events    = {0};
volatile temperature_readings_t temperature_readings = {0};

/**
 * @brief Initialize IPC primitives: queues, mutex, and zero shared structs.
 */
void ipc_init(void) {

    // Create queues (sensor_queue holds the largest sensor message type)
    sensor_queue  = xQueueCreate(16, sizeof(sensor_msg_t));
    control_queue = xQueueCreate(1, sizeof(control_msg_t));

    // Zero shared structs
    memset((void*)&smart_desk_events, 0, sizeof(smart_desk_events));
    memset((void*)&temperature_readings, 0, sizeof(temperature_readings));
}