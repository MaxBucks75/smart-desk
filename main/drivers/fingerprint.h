/*****************************************************************************
 * @file    fingerprint.h
 * @brief   R503 fingerprint sensor interface declarations for Smart Desk
 *****************************************************************************/

#pragma once

#include "driver/uart.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "stdint.h"
#include "tasks/ipc.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// Defaults
#define R503_RECEIVE_TIMEOUT 3000
#define R503_RESET_TIMEOUT 3000
#define R503_HEADER_BYTES 0xEF, 0x01
#define R503_ADDRESS_BYTES 0xFF, 0xFF, 0xFF, 0xFF
#define R503_ACKKNOWLEDGE_PACKAGE_IDENTIFIER 0x07
#define R503_CHAR_BUFFER_1 0x01
#define R503_CHAR_BUFFER_2 0x02
#define MAX_R503_PACKET_SIZE 64

// Commands
#define R503_COMMAND_PACKET 0x01
#define R503_GENERATE_IMAGE 0x01
#define R503_UPLOAD_IMAGE 0x0A
#define R503_DOWNLOAD_IMAGE 0x0B
#define R503_GENERATE_CHAR_FILE_FROM_IMAGE 0x02
#define R503_REGISTER_MODEL 0x05
#define R503_UPLOAD_CHARACTER_FILE 0x08
#define R503_DOWNLOAD_CHARACTER_FILE 0x09
#define R503_STORE 0x06
#define R503_SEARCH 0x04
#define R503_DELETE_TEMPLATE 0x0C
#define R503_CONTROL_LED 0x35
#define R503_READ_INDEX_TABLE 0x1F
#define R503_READ_SYSTEM_PARAMETERS 0x0F
#define R503_LOAD_CHAR 0x07
#define R503_MATCH 0x03
#define R503_READ_SYSTEM_REGISTER 0x0F

// LED options
#define R503_LED_BREATHING 0x01
#define R503_LED_FLASHING 0x02
#define R503_LED_ALWAYS_ON 0x03
#define R503_LED_ALWAYS_OFF 0x04
#define R503_LED_GRADUALLY_ON 0x05
#define R503_LED_GRADUALLY_OFF 0x06
#define R503_LED_COLOR_RED 0x01
#define R503_LED_COLOR_BLUE 0x02
#define R503_LED_COLOR_PURPLE 0x03

// Confirmation Codes
#define R503_SUCCESS 0x00
#define R503_ERROR_RECEIVING_PACKET 0x01
#define R503_NO_FINGER 0x02
#define R503_ERROR_TAKING_IMAGE 0x03
#define R503_IMAGE_MESSY 0x06
#define R503_FEATURE_FAIL 0x07
#define R503_NO_MATCH 0x08
#define R503_NO_MATCH_IN_LIBRARY 0x09
#define R503_WRONG_PASSWORD 0x13
#define R503_NO_IMAGE 0x15
#define R503_BAD_LOCATION 0x0B
#define R503_ERROR_WRITING_FLASH 0x18
#define R503_SENSOR_ABNORMAL 0x29
#define R503_ERROR_TRANSFER_DATA 0x0E

// Error Codes
#define R503_ADDRESS_MISMATCH 0xE1
#define R503_NOT_ENOUGH_MEMORY 0xE2
#define R503_CHECKSUM_MISMATCH 0xE3
#define R503_PACKET_MISMATCH 0xE5
#define R503_INVALID_START_CODE 0xE6
#define R503_INVALID_BAUDRATE 0xE8
#define R503_TIMEOUT 0xE9

/**
 * @brief Initialize UART and start the fingerprint UART event task.
 */
void R503_init(void);

/**
 * @brief FreeRTOS task that processes UART events from the R503 sensor.
 *
 * This task waits for UART events and assembles incoming packets into
 * 'R503_rx_buf', notifying the waiting command tasks when a response is
 * available.
 */
void uart_event_task(void* pvParameters);

/**
 * @brief Find the next free fingerprint template ID in the sensor library.
 *
 * @return index of free id or -1 if full
 */
uint16_t R503_find_free_id(void);

/**
 * @brief Send a packet to the R503 sensor.
 *
 * @param msg Pointer to payload bytes
 * @param package_identifier Packet type (command/data)
 * @param length Length of payload
 */
esp_err_t R503_send_package(uint8_t* msg, uint8_t package_identifier, uint16_t length);

/**
 * @brief Send command and wait for ACK from sensor.
 */
esp_err_t send_package_and_read_ack(uint8_t* cmd, uint16_t length);

/**
 * @brief Capture an image from the sensor and place it in the image buffer.
 */
esp_err_t generate_image(void);

/**
 * @brief Wait for a finger to be presented and capture it into a buffer.
 *
 * @param buffer_id Character buffer id to store captured data
 * @param timeout Tick timeout to wait for finger
 */
esp_err_t wait_for_finger_and_capture(uint8_t buffer_id, TickType_t timeout);

/**
 * @brief Convert the last captured image into a character file.
 */
esp_err_t generate_char_file_from_image(uint8_t buffer_id);

/**
 * @brief Combine character files into a template model.
 */
esp_err_t register_model(void);

/**
 * @brief Store a template into the sensor's persistent library.
 */
esp_err_t store_template(uint16_t location_id);

/**
 * @brief Delete fingerprint templates starting at location_id.
 */
esp_err_t delete_fingerprint(uint16_t location_id, uint16_t count);

/**
 * @brief Control the R503 LED patterns.
 */
esp_err_t set_led(uint8_t led_control, uint8_t led_speed, uint8_t led_color, uint8_t num_cycles);

/**
 * @brief Read the index bitmap page for stored templates.
 */
esp_err_t read_index_table(uint8_t page);

/**
 * @brief Load a character file from library into a buffer.
 */
esp_err_t load_char(uint8_t buffer_id, uint16_t location_id);

/**
 * @brief Read system parameters and populate local status flags.
 */
esp_err_t read_system_parameters(void);

/**
 * @brief Perform interactive enrollment sequence (two captures + store).
 */
esp_err_t auto_enroll(void);

/**
 * @brief Attempt to identify the currently presented finger against library.
 */
esp_err_t auto_identify(void);

/**
 * @brief Convert confirmation code to readable log messages.
 */
esp_err_t read_confirmation_code(uint8_t confirmation_code);