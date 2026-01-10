#include "temperature_sensor.h"


#define TAG "TEMP_SENSOR"

//#define TEMP_DEBUG

static i2c_master_bus_handle_t bus_handle = NULL;

i2c_master_dev_handle_t temp_handle_exhaust;
i2c_master_dev_handle_t temp_handle_central;

void i2c_init(void) {

    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT,
        .sda_io_num = GPIO_NUM_21,
        .scl_io_num = GPIO_NUM_22,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&i2c_mst_config, &bus_handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C bus: %s", esp_err_to_name(err));
    }

    mcp9808_init(get_i2c_bus_handle(), MCP9808_CENTRAL_ADDRESS, &temp_handle_central);
    mcp9808_init(get_i2c_bus_handle(), MCP9808_EXHAUST_ADDRESS, &temp_handle_exhaust);

}

esp_err_t mcp9808_init(i2c_master_bus_handle_t bus, uint8_t address, i2c_master_dev_handle_t *out_handle) {

    uint8_t id_buf[2];

    // Config the MCP9808
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = 100000,
    };

    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, out_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device @ 0x%02X: %s", address, esp_err_to_name(err));
        return err;
    }

    // Put device in normal mode (0x0000 -> config register)
    uint8_t config_write[] = {MCP9808_CONFIG_REG, 0x00, 0x00}; // normal mode
    err = i2c_master_transmit(*out_handle, config_write, sizeof(config_write), -1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write config to 0x%02X: %s", address, esp_err_to_name(err));
        return err;
    }

    // Read config register
    uint8_t config_read[2];
    uint8_t reg = MCP9808_CONFIG_REG;
    err = i2c_master_transmit_receive(*out_handle, &reg, 1, config_read, 2, -1);
    ESP_LOGI(TAG, "Config readback @0x%02X = 0x%02X 0x%02X", address, config_read[0], config_read[1]);

    // Read Manufacturer ID
    reg = MCP9808_MANUFACTURER_ID_REG;
    err = i2c_master_transmit_receive(*out_handle, &reg, 1, id_buf, 2, -1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read manufacturer ID from 0x%02X", address);
        return err;
    }
    uint16_t manuf_id = (id_buf[0] << 8) | id_buf[1];
    
    // Read device ID register
    reg = MCP9808_DEVICE_ID_REG;
    err = i2c_master_transmit_receive(*out_handle, &reg, 1, id_buf, 2, -1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read device ID from 0x%02X", address);
        return err;
    }
    uint16_t dev_id = (id_buf[0] << 8) | id_buf[1];

    // Check that the device and manufacturer ids match the datasheet defaults
    if (manuf_id != MCP9809_MANUFACTURER_ID || (dev_id >> 8) != MCP9808_DEVICE_ID) {
        ESP_LOGW(TAG, "Device at 0x%02X does not match expected MCP9808 IDs (Got MID=0x%04X DID=0x%04X)", address, manuf_id, dev_id);
    } else {
        ESP_LOGI(TAG, "Verified MCP9808 at 0x%02X (MID=0x%04X DID=0x%04X)", address, manuf_id, dev_id);
    }

    return ESP_OK;

}

float get_temperature_reading(i2c_master_dev_handle_t dev_handle) {

    uint8_t data[2] = {0};
    float temp = -1000.0;
    uint8_t reg = MCP9808_TEMPERATURE_REG;

    // Transmit the message to receive the temperature data
    esp_err_t err = i2c_master_transmit_receive(dev_handle, &reg, 1, data, 2, -1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C temp read failed: %s", esp_err_to_name(err));
        return temp;
    }
    
    #ifdef TEMP_DEBUG
    ESP_LOGI(TAG, "Raw: 0x%02X 0x%02X", data[0], data[1]);
    #endif

    // Convert raw bits to celsius (data[0] = MSB, data[1] = LSB)
    data[0] = data[0] & 0x1F; // Clear flag bits
    // Check sign bit
    if ((data[0] & 0x10) == 0x10) {
        data[0] = data[0] & 0x0F; // Clear sign bits
        temp = 256 - (data[0] * 16.0f + (float)data[1] / 16.0f);
    } else {
        temp = data[0] * 16.0f + (float)data[1] / 16.0f;
    }

    return temp;

}

void check_i2c_address(void) {

    esp_err_t err;
    for (uint8_t i = 3; i < 0x78; i++) {
        err = i2c_master_probe(bus_handle, i, 1000);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "I2C Scanner found device at address: 0x%X \n", i);
        }
    }

    ESP_LOGI(TAG, "I2C Scanner finished");

}

i2c_master_bus_handle_t get_i2c_bus_handle(void) {
    return bus_handle;
}
