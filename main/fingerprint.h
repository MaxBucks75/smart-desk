#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// Defaults
#define R503_RECEIVE_TIMEOUT                    3000
#define R503_RESET_TIMEOUT                      3000
#define R503_HEADER_BYTES                       0xEF, 0x01
#define R503_ADDRESS_BYTES                      0xFF, 0xFF, 0xFF, 0xFF
#define R503_ACKKNOWLEDGE_PACKAGE_IDENTIFIER    0x07
#define R503_CHAR_BUFFER_1                      0x01
#define R503_CHAR_BUFFER_2                      0x02
#define MAX_R503_PACKET_SIZE                    64

// New auto-enroll and auto-identify commands (R503 v1.4.1+)
#define R503_AUTO_ENROLL                        0x32  // Enroll: 6 step image capture
#define R503_AUTO_IDENTIFY                      0x33  // Identify: capture + search

// Commands
#define R503_COMMAND_PACKET                     0x01
#define R503_GENERATE_IMAGE                     0x01
#define R503_UPLOAD_IMAGE                       0x0A
#define R503_DOWNLOAD_IMAGE                     0x0B
#define R503_GENERATE_CHAR_FILE_FROM_IMAGE      0x02
#define R503_REGISTER_MODEL                     0x05
#define R503_UPLOAD_CHARACTER_FILE              0x08
#define R503_DOWNLOAD_CHARACTER_FILE            0x09
#define R503_STORE                              0x06
#define R503_SEARCH                             0x04
#define R503_DELETE_TEMPLATE                    0x0C
#define R503_CONTROL_LED                        0x35
#define R503_TEMPLATE_COUNT                     0x1F

// LED options
#define R503_LED_BREATHING                      0x01
#define R503_LED_FLASHING                       0x02
#define R503_LED_ALWAYS_ON                      0x03
#define R503_LED_ALWAYS_OFF                     0x04
#define R503_LED_GRADUALLY_ON                   0x05
#define R503_LED_GRADUALLY_OFF                  0x06
#define R503_LED_COLOR_RED                      0x01
#define R503_LED_COLOR_BLUE                     0x02
#define R503_LED_COLOR_PURPLE                   0x03

// Confirmation Codes
#define R503_SUCCESS                            0x00
#define R503_ERROR_RECEIVING_PACKET             0x01
#define R503_NO_FINGER                          0x02
#define R503_ERROR_TAKING_IMAGE                 0x03
#define R503_IMAGE_MESSY                        0x06
#define R503_FEATURE_FAIL                       0x07
#define R503_NO_MATCH                           0x08
#define R503_NO_MATCH_IN_LIBRARY                0x09
#define R503_WRONG_PASSWORD                     0x13
#define R503_NO_IMAGE                           0x15
#define R503_BAD_LOCATION                       0x0B
#define R503_ERROR_WRITING_FLASH                0x18
#define R503_SENSOR_ABNORMAL                    0x29
#define R503_ERROR_TRANSFER_DATA                0x0E

// Error Codes
#define R503_ADDRESS_MISMATCH                   0xE1
#define R503_NOT_ENOUGH_MEMORY                  0xE2
#define R503_CHECKSUM_MISMATCH                  0xE3
#define R503_PACKET_MISMATCH                    0xE5
#define R503_INVALID_START_CODE                 0xE6
#define R503_INVALID_BAUDRATE                   0xE8
#define R503_TIMEOUT                            0xE9

void R503_init(void);
esp_err_t R503_send_package(uint8_t *msg, uint8_t package_identifier, uint16_t length);
esp_err_t read_confirmation_code(uint8_t confirmation_code);
esp_err_t R503_send_package_and_check_ack(uint8_t *cmd, uint16_t length);
esp_err_t generate_image(void);
esp_err_t upload_image(void);
esp_err_t download_image(void);
esp_err_t generate_char_file_from_image(uint8_t buffer_id);
esp_err_t register_model(void);
esp_err_t upload_character_file(uint8_t buffer_id);
esp_err_t download_character_file(uint8_t buffer_id);
esp_err_t store_template(uint8_t buffer_id, uint16_t location_id);
esp_err_t search_fingerprint(uint16_t start_page, uint16_t page_count);
esp_err_t delete_fingerprint(uint16_t location_id, uint16_t count);
esp_err_t set_led(bool enable, uint8_t led_control, uint8_t led_speed, uint8_t led_color, uint8_t num_cycles);
esp_err_t read_template_count(uint8_t page);
esp_err_t R503_auto_enroll(uint8_t slot);
esp_err_t R503_auto_identify(uint16_t start_page, uint16_t page_count);