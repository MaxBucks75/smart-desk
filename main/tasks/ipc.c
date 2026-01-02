#include "ipc.h"
#include <string.h>

QueueHandle_t sensor_queue = NULL;
QueueHandle_t control_queue = NULL;

// Instantiate global structs
volatile smart_desk_events_t smart_desk_events = {0};
volatile raw_force_data_t raw_force_data = {0};
volatile temperature_readings_t temperature_readings = {0};
// Mutex to protect shared IPC structures
SemaphoreHandle_t ipc_mutex = NULL;

void ipc_init(void) {

    // Create queues (sensor_queue holds the largest sensor message type)
    sensor_queue = xQueueCreate(16, sizeof(sensor_msg_t));
    control_queue = xQueueCreate(1, sizeof(control_msg_t));

    // Create mutex for protecting shared structs
    ipc_mutex = xSemaphoreCreateMutex();

    // Zero all shared structs
    memset((void *)&smart_desk_events, 0, sizeof(smart_desk_events));
    memset((void *)&raw_force_data, 0, sizeof(raw_force_data));
    memset((void *)&temperature_readings, 0, sizeof(temperature_readings));
    
}