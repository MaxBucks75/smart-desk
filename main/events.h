#pragma once

// Event flags for desk controls
typedef struct smart_desk_events_t {
    unsigned int tl_sensor_activated : 1;
    unsigned int tr_sensor_activated : 1;
    unsigned int bl_sensor_activated : 1;
    unsigned int br_sensor_activated : 1;
    unsigned int usb_hub_extended : 1;
    unsigned int usb_hub_moving : 1;
} smart_desk_events_t;

extern volatile smart_desk_events_t smart_desk_events;