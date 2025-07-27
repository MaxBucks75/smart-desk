#include "fingerprint.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_check.h"
#include <string.h>
#include "stdint.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"

// Uncomment to print debug statements
#define CONFIG_FP_DEBUG

// Define this macro globally to enable debug output
#ifdef CONFIG_FP_DEBUG
#define FP_DEBUG(fmt, ...) ESP_LOGI(TAG, fmt, ##__VA_ARGS__)
#else
#define FP_DEBUG(fmt, ...) do {} while(0)
#endif

#define TAG "FINGERPRINT"

// TODO add this documentation link somewhere in the project https://download.mikroe.com/documents/datasheets/R503_datasheet.pdf

const uart_port_t uart_num = UART_NUM_2;
const int R503_buffer_size = (512 * 2); // Allocate buffer size of 64 bytes for ~16 byte UART messages

/**
 * @brief Initializes UART communication with the R503 fingerprint sensor.
 * 
 * Sets up UART port, configures baud rate, and sets RX/TX pins.
 */
void R503_init(void) {

    // Set communication parameters
    const uart_config_t uart_config = {
        .baud_rate = 57600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    // Set UART pins (TX: 17, RX: 16, RTS: not used, CTS: not used)
    ESP_ERROR_CHECK(uart_set_pin(uart_num, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Install UART driver on port 2, TX size 32 bytes, RX size of 32 bytes, event queue of 10 stored in R503 queue, and 0 interrupt flags.
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, 1024, 1024, 0, NULL, 0));

    uart_flush_input(uart_num); // Ensure the UART is cleared

    ESP_LOGI(TAG, "UART initialized on TX=17, RX=16 @57600");

}

/**
 * @brief Sends a formatted command packet to the R503 fingerprint sensor.
 * 
 * @param msg Pointer to payload data.
 * @param package_identifier Packet type (e.g., command, data).
 * @param length Length of payload.
 * @return ESP_OK if transmission successful, or error code.
 */
esp_err_t R503_send_package(uint8_t *msg, uint8_t package_identifier, uint16_t length) {

    if (msg == NULL || length == 0) return ESP_ERR_INVALID_ARG;

    uint8_t package[20];
    int index = 0;

    // Add the header and address
    const uint8_t header[] = { R503_HEADER_BYTES, R503_ADDRESS_BYTES };
    memcpy(package, header, sizeof(header));
    index += sizeof(header);

    // Add the package identifier
    package[index++] = package_identifier;

    // Add the package length
    uint16_t package_length = 2 + length;
    package[index++] = (package_length >> 8) & 0xFF;
    package[index++] = package_length & 0xFF;

    // Add the package message
    memcpy(&package[index], msg, length);
    index += length;

    // Calculate and add checksum
    uint16_t checksum = package_identifier + package_length;
    for (int i = 0; i < length; i++) checksum += msg[i];
    package[index++] = (checksum >> 8) & 0xFF;
    package[index++] = checksum & 0xFF;

    #ifdef CONFIG_FP_DEBUG
    ESP_LOGI(TAG, "Writing %d bytes", index);
    for (int i = 0; i < index; i++) {
        printf("%02X ", package[i]);
    }
    printf("\n");
    #endif

    // Write
    esp_err_t ret = uart_write_bytes(uart_num, (const char*)package, index);
    if (ret < 0) return ESP_FAIL;

    return uart_wait_tx_done(uart_num, 100);
}

/**
 * @brief Reads a response packet from the R503 sensor into a buffer.
 * 
 * @param buffer Buffer to hold received data.
 * @return Number of bytes read.
 */
static int R503_read_package(uint8_t *buffer) {

    if (!buffer) return ESP_ERR_INVALID_ARG;

    uint8_t temp_buf[MAX_R503_PACKET_SIZE] = {0};
    int read_len = uart_read_bytes(uart_num, temp_buf, MAX_R503_PACKET_SIZE, portMAX_DELAY);

    #ifdef CONFIG_FP_DEBUG
    ESP_LOGI(TAG, "Raw packet:");
    for (int i = 0; i < read_len; i++) {
        printf("%02X ", temp_buf[i]);
    }
    printf("\n");
    #endif

    if (read_len < 11) {
        ESP_LOGE(TAG, "Response too short to be valid: %d bytes", read_len);
        return ESP_FAIL;
    }
    if (temp_buf[0] != 0xEF || temp_buf[1] != 0x01) {
        ESP_LOGE(TAG, "Invalid packet header");
        return ESP_FAIL;
    }

    memcpy(buffer, temp_buf, read_len);

    #ifdef CONFIG_FP_DEBUG
    ESP_LOGI(TAG, "Read %d bytes", read_len);
    for (int i = 0; i < read_len; i++) {
        printf("%02X ", buffer[i]);
    }
    printf("\n");
    #endif

    return read_len;

}


/**
 * @brief Helper function to read confirmation codes from R503 acknowledge messages
 * 
 * @param confirmation_code 
 * @return esp_err_t 
 */
esp_err_t read_confirmation_code(uint8_t confirmation_code) {

    switch (confirmation_code) {
        case R503_SUCCESS:
            ESP_LOGI(TAG, "Fingerprint: Success");
            return ESP_OK;

        case R503_ERROR_RECEIVING_PACKET:
            ESP_LOGE(TAG, "Fingerprint: Error receiving packet");
            break;
        case R503_NO_FINGER:
            ESP_LOGW(TAG, "Fingerprint: No finger detected");
            break;
        case R503_ERROR_TAKING_IMAGE:
            ESP_LOGE(TAG, "Fingerprint: Error taking image");
            break;
        case R503_IMAGE_MESSY:
            ESP_LOGW(TAG, "Fingerprint: Image too messy");
            break;
        case R503_FEATURE_FAIL:
            ESP_LOGE(TAG, "Fingerprint: Feature extraction failed");
            break;
        case R503_NO_MATCH:
            ESP_LOGI(TAG, "Fingerprint: No match found");
            break;
        case R503_NO_MATCH_IN_LIBRARY:
            ESP_LOGI(TAG, "Fingerprint: No match in library");
            break;
        case R503_WRONG_PASSWORD:
            ESP_LOGE(TAG, "Fingerprint: Wrong password");
            break;
        case R503_NO_IMAGE:
            ESP_LOGE(TAG, "Fingerprint: No image in buffer");
            break;
        case R503_BAD_LOCATION:
            ESP_LOGE(TAG, "Fingerprint: Invalid location ID");
            break;
        case R503_ERROR_WRITING_FLASH:
            ESP_LOGE(TAG, "Fingerprint: Error writing to flash");
            break;
        case R503_SENSOR_ABNORMAL:
            ESP_LOGE(TAG, "Fingerprint: Sensor abnormal");
            break;
        case R503_ERROR_TRANSFER_DATA:
            ESP_LOGE(TAG, "Fingerprint: Error transferring data");
            break;
        default:
            ESP_LOGE(TAG, "Fingerprint: Unknown error (0x%02X)", confirmation_code);
            break;
    }

    return ESP_FAIL;

}


/**
 * @brief Sends an R503 command, reads the ACK, and logs confirmation code.
 *
 * @param cmd Pointer to command bytes
 * @param length Length of command
 * @return esp_err_t ESP_OK if success (0x00), otherwise ESP_FAIL
 */
esp_err_t R503_send_package_and_check_ack(uint8_t *cmd, uint16_t length) {

    uint8_t response[MAX_R503_PACKET_SIZE];

    // Clear out any stray RX data
    uart_flush_input(uart_num);

    // Send the command
    esp_err_t err = R503_send_package(cmd, R503_COMMAND_PACKET, length);
    if (err != ESP_OK) return err;

    // Wait a few ms to let the sensor respond
    vTaskDelay(pdMS_TO_TICKS(30));

    // Read back the ACK packet and check that it's valid
    if (R503_read_package(response) < 0) return ESP_FAIL;
    if (response[6] != R503_ACKKNOWLEDGE_PACKAGE_IDENTIFIER) {
        ESP_LOGE(TAG, "Expected ACK, got PID=0x%02X", response[6]);
        return ESP_FAIL;
    }
    if (response[9] != R503_SUCCESS) {
        ESP_LOGE(TAG, "Command NACK (0x%02X)", response[9]);
        return ESP_FAIL;
    }

    // Wait then read the response packet and check that it's valid
    vTaskDelay(pdMS_TO_TICKS(30));
    if (R503_read_package(response) < 0) return ESP_FAIL;
    if (response[9] != R503_SUCCESS) {
        ESP_LOGE(TAG, "Operation failed (0x%02X)", response[9]);
        return ESP_FAIL;
    }

    return ESP_OK;

}

/**
 * @brief Detecting finger and store the detected finger in the image buffer.
 * 
 * @return esp_err_t 
 */
esp_err_t generate_image(void) {
    uint8_t cmd = R503_GENERATE_IMAGE; 
    return R503_send_package_and_check_ack(&cmd, 1);
}

/**
 * @brief Upload the image in the image buffer.
 * 
 * @return esp_err_t 
 */
esp_err_t upload_image(void) {
    uint8_t cmd = R503_UPLOAD_IMAGE; 
    return R503_send_package_and_check_ack(&cmd, 1);
}

/**
 * @brief Download the image from the library to image buffer.
 * 
 * @return esp_err_t 
 */
esp_err_t download_image(void) {
    uint8_t cmd = R503_DOWNLOAD_IMAGE; 
    return R503_send_package_and_check_ack(&cmd, 1);
}

/**
 * @brief Generate character file from image
 * 
 * @param buffer_id selects the character buffer to save to 1 = buffer_1 >1 = buffer_2
 * @return esp_err_t 
 */
esp_err_t generate_char_file_from_image(uint8_t buffer_id) {
    uint8_t cmd[] = {R503_GENERATE_CHAR_FILE_FROM_IMAGE, buffer_id}; 
    return R503_send_package_and_check_ack(cmd, 2);
}

/**
 * @brief Combine information of character files from character buffer 1 and 2
 * and generate template which is stored back into character buffers.
 * 
 * @return esp_err_t 
 */
esp_err_t register_model(void) {
    uint8_t cmd = R503_REGISTER_MODEL; 
    return R503_send_package_and_check_ack(&cmd, 1);
}

/**
 * @brief Uploads the character fle from buffers 1 and 2 to the library
 * 
 * @param buffer_id selects the character buffer to upload to 1 = buffer_1 >1 = buffer_2
 * @return esp_err_t 
 */
esp_err_t upload_character_file(uint8_t buffer_id) {
    uint8_t cmd[] = {R503_UPLOAD_CHARACTER_FILE, buffer_id};
    return R503_send_package_and_check_ack(cmd, 2);
}

/**
 * @brief Uploads the character fle from buffers 1 and 2 to the library
 * 
 * @param buffer_id selects the character buffer to upload to 1 = buffer_1 >1 = buffer_2
 * @return esp_err_t 
 */
esp_err_t download_character_file(uint8_t buffer_id) {
    uint8_t cmd[] = {R503_DOWNLOAD_CHARACTER_FILE, buffer_id};
    return R503_send_package_and_check_ack(cmd, 2);
}

/**
 * @brief Store the template of either buffer into a location in flash library.
 * 
 * @param buffer_id selects the higher or lower buffer
 * @param location_id The index that the template will be saved to
 * @return esp_err_t 
 */
esp_err_t store_template(uint8_t buffer_id, uint16_t location_id) {
 
    uint8_t cmd[4];
    cmd[0] = R503_STORE;
    cmd[1] = buffer_id;
    cmd[2] = (location_id >> 8) & 0xFF;
    cmd[3] = location_id & 0xFF;
 
    return R503_send_package_and_check_ack(cmd, 4);

}

/**
 * @brief Searches the fingerprint against stored templates (buffer 1).
 * 
 * @param start_page Starting page ID (usually 0)
 * @param page_count Number of pages to search (e.g. 100)
 * @return esp_err_t ESP_OK if match found, ESP_FAIL otherwise
 */
esp_err_t search_fingerprint(uint16_t start_page, uint16_t page_count) {
    uint8_t cmd[5] = {
        R503_SEARCH,
        0x01,  // Use buffer 1
        (start_page >> 8) & 0xFF,
        start_page & 0xFF,
        page_count
    };
    return R503_send_package_and_check_ack(cmd, 5);
}

/**
 * @brief Deletes a stored fingerprint template.
 * 
 * @param location_id Template ID to delete
 * @param count Number of templates to delete (usually 1)
 * @return esp_err_t ESP_OK if successful
 */
esp_err_t delete_fingerprint(uint16_t location_id, uint16_t count) {
    uint8_t cmd[5] = {
        R503_DELETE_TEMPLATE,
        (location_id >> 8) & 0xFF,
        location_id & 0xFF,
        (count >> 8) & 0xFF,
        count & 0xFF
    };
    return R503_send_package_and_check_ack(cmd, 5);
}

/**
 * @brief Enables or disables the fingerprint LED ring.
 * 
 * @param enable True to turn on, false to turn off
 * @return esp_err_t ESP_OK if successful
 */
esp_err_t set_led(bool enable, uint8_t led_control, uint8_t led_speed, uint8_t led_color, uint8_t num_cycles) {

    uint8_t cmd[5] = {
        R503_CONTROL_LED,
        led_control,           // control code: turn on/off
        led_speed,             // 0x00 - 0xFF for light breathing controls
        led_color,             // LED color
        num_cycles             // 0x00 - 0xFF for number of breatthing cycles
    };
    return R503_send_package_and_check_ack(cmd, 5);

}

/**
 * @brief Requests the number of stored fingerprint templates.
 * 
 * @param page determines page to check (groups of 128 fingerprints)
 * @return esp_err_t ESP_OK if successful
 */
esp_err_t read_template_count(uint8_t page) {
    uint8_t cmd[2] = {R503_TEMPLATE_COUNT, page};
    return R503_send_package_and_check_ack(cmd, 2);
}

/**
 * @brief Let the module handle the entire enroll sequence (detect, char, reg, store)
 * @param slot  the template ID to save into (0…255)
 */
esp_err_t R503_auto_enroll(uint8_t slot) {
    uint8_t cmd[] = {
        R503_AUTO_ENROLL,
        0x00, slot,
        0x01, // enroll once
        0x01  // store into buffer 1
    };
    uint8_t response[MAX_R503_PACKET_SIZE];
    // send enroll command
    if (R503_send_package(cmd, R503_COMMAND_PACKET, sizeof(cmd)) != ESP_OK) {
        return ESP_FAIL;
    }
    // read initial ACK packet
    if (R503_read_package(response) < 0 || response[9] != R503_SUCCESS) {
        ESP_LOGE(TAG, "Auto‑enroll ACK failed (code=0x%02X)", response[9]);
        return ESP_FAIL;
    }
    // read final confirmation packet
    if (R503_read_package(response) < 0 || response[9] != R503_SUCCESS) {
        ESP_LOGE(TAG, "Auto‑enroll result failed (code=0x%02X)", response[9]);
        return ESP_FAIL;
    }
    return ESP_OK;
}

/**
 * @brief Let the module handle entire 1:N match sequence
 * @param start_page first ID to scan (usually 0)
 * @param page_count how many IDs to scan
 */
esp_err_t R503_auto_identify(uint16_t start_page, uint16_t page_count) {
    uint8_t cmd[] = {
        R503_AUTO_IDENTIFY,
        0x01, // use buffer 1
        (start_page >> 8) & 0xFF, start_page & 0xFF,
        (page_count >> 8) & 0xFF, page_count & 0xFF
    };
    uint8_t response[MAX_R503_PACKET_SIZE];
    // send identify command
    if (R503_send_package(cmd, R503_COMMAND_PACKET, sizeof(cmd)) != ESP_OK) {
        return ESP_FAIL;
    }
    // read ACK packet
    if (R503_read_package(response) < 0 || response[9] != R503_SUCCESS) {
        ESP_LOGE(TAG, "Auto‑identify ACK failed (code=0x%02X)", response[9]);
        return ESP_FAIL;
    }
    // read search result packet
    if (R503_read_package(response) < 0) {
        ESP_LOGE(TAG, "Auto‑identify read result error");
        return ESP_FAIL;
    }
    // response[9] & [10] hold the matching ID (0xFFFF if no match)
    uint16_t match_id = (response[9] << 8) | response[10];
    if (match_id == 0xFFFF) {
        return ESP_FAIL;
    }
    return ESP_OK;
}