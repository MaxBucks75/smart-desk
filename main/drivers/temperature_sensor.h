/******************************************************************************
 * @file    temperature_sensor.h
 * @brief   MCP9808 temperature sensor I2C helpers for Smart Desk
 *****************************************************************************/

#pragma once

#include "driver/i2c_master.h"
#include "driver/i2c_slave.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "stdint.h"

#define I2C_PORT I2C_NUM_0
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define I2C_FREQ_HZ 100000

// Register pointers (page 16 of PDF)
#define MCP9808_CONFIG_REG 0x01      // Details on page 18
#define MCP9808_TEMPERATURE_REG 0x05 // Details on page 24
#define MCP9808_MANUFACTURER_ID_REG 0x06
#define MCP9808_DEVICE_ID_REG 0x07

#define MCP9808_DEVICE_ID 0x04
#define MCP9809_MANUFACTURER_ID 0x0054

// Bit breakdown for all registers page 17
#define MCP9808_EXHAUST_ADDRESS 0x18
#define MCP9808_CENTRAL_ADDRESS 0x1A

extern i2c_master_dev_handle_t temp_handle_exhaust;
extern i2c_master_dev_handle_t temp_handle_central;

/**
 * @brief Initialize the I2C master bus and attach MCP9808 devices.
 */
void i2c_init(void);

/**
 * @brief Initialize a single MCP9808 device on the provided bus.
 *
 * @param bus I2C bus handle
 * @param address 7-bit device address
 * @param out_handle Returned device handle pointer
 * @return ESP_OK on success or error code
 */
esp_err_t mcp9808_init(i2c_master_bus_handle_t bus, uint8_t address,
                       i2c_master_dev_handle_t* out_handle);

/**
 * @brief Read temperature from specified MCP9808 device and return °C.
 *
 * Returns -1000.0f on error.
 */
float get_temperature_reading(i2c_master_dev_handle_t dev_handle);

/**
 * @brief Scan the I2C bus and log detected device addresses.
 */
void check_i2c_address(void);

/**
 * @brief Return the initialized I2C master bus handle.
 */
i2c_master_bus_handle_t get_i2c_bus_handle(void);