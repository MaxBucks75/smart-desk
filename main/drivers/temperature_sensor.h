#include "stdint.h"
#include "driver/i2c_master.h"
#include "driver/i2c_slave.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#pragma once

#define I2C_PORT        I2C_NUM_0
#define I2C_SDA_PIN     21
#define I2C_SCL_PIN     22
#define I2C_FREQ_HZ     100000

// Register pointers (page 16 of PDF)
#define MCP9808_CONFIG_REG          0x01 // Details on page 18
#define MCP9808_TEMPERATURE_REG     0x05 // Details on page 24
#define MCP9808_MANUFACTURER_ID_REG 0x06
#define MCP9808_DEVICE_ID_REG       0x07

#define MCP9808_DEVICE_ID           0x04
#define MCP9809_MANUFACTURER_ID     0x0054

// Bit breakdown for all registers page 17
#define MCP9808_EXHAUST_ADDRESS 0x18
#define MCP9808_CENTRAL_ADDRESS 0x1A

#define MCP9808_EXHAUST_START_FANS_THRESHOLD_C  25 // Temp (C) to start the fans PWM
#define MCP9808_EXHAUST_MAX_FANS_THRESHOLD_C    60 // Temp (C) to max the fans PWM
#define MCP9808_CENTRAL_START_FANS_THRESHOLD_C  20 // Temp (C) to start the fans PWM
#define MCP9808_CENTRAL_MAX_FANS_THRESHOLD_C    55 // Temp (C) to max the fans PWM

extern i2c_master_dev_handle_t temp_handle_exhaust;
extern i2c_master_dev_handle_t temp_handle_central;


// Section 5 contains register accessing data https://ww1.microchip.com/downloads/en/DeviceDoc/25095A.pdf

void i2c_init(void);
esp_err_t mcp9808_init(i2c_master_bus_handle_t bus, uint8_t address, i2c_master_dev_handle_t *out_handle);
float get_temperature_reading(i2c_master_dev_handle_t dev_handle);
void check_i2c_address(void);
i2c_master_bus_handle_t get_i2c_bus_handle(void);