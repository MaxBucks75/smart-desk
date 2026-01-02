#pragma once

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

typedef struct {
    float prev_temp_central;
    float prev_temp_exhaust;
    float curr_temp_central;
    float curr_temp_exhaust;
} temperature_readings_t;

// Will be used for monitoring real-time force sensor data
typedef struct {
    uint16_t bl_sensor;
    uint16_t br_sensor;
    uint16_t tl_sensor;
    uint16_t tr_sensor;
} raw_force_data_t;

// Event flags for desk
typedef struct {
    unsigned int tl_sensor_activated : 1;
    unsigned int tr_sensor_activated : 1;
    unsigned int bl_sensor_activated : 1;
    unsigned int br_sensor_activated : 1;
    unsigned int desk_moving : 1;
    unsigned int finger_detected : 1;
    unsigned int computer_power : 1;
    unsigned int usb_hub_extended : 1;
    unsigned int usb_hub_moving : 1;
} smart_desk_events_t;

// Used for tracking the last sensor reading from queue
typedef enum {
    SENSOR_SOURCE_TEMP,
    SENSOR_SOURCE_FORCE,
    SENSOR_SOURCE_FP
} sensor_source_t;

// Lightweight sensor message that can carry temperature readings
// (using this avoids shared globals / mutexes for temperature data).
typedef struct {
    sensor_source_t source;
    uint64_t data;
} sensor_msg_t;

// Used for sending data through queue easily
typedef enum {
    SET_FAN_SPEED,
    SET_DESK_HEIGHT,
    UPDATE_POWER,
    UPDATE_LED
} control_cmd_t;

typedef struct {
    control_cmd_t cmd;
    uint64_t data;
} control_msg_t;

// Global structs
extern volatile smart_desk_events_t smart_desk_events;
extern volatile raw_force_data_t raw_force_data;
extern volatile temperature_readings_t temperature_readings;

// Queues for retrieving sensor data and acting on it accordingly
extern QueueHandle_t sensor_queue;
extern QueueHandle_t control_queue;

// Mutex to protect shared IPC structures (use xSemaphoreTake/xSemaphoreGive)
extern SemaphoreHandle_t ipc_mutex;

void ipc_init(void);