#include "fingerprint.h"


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

static uint8_t R503_rx_buf[MAX_R503_PACKET_SIZE]; // Holds the value from the last R503 rx transmission
static uint16_t R503_rx_index;                    // Holds the index for the number of bytes in the last rx transmission

QueueHandle_t uart_queue;              // UART event queue
static QueueHandle_t response_queue;   // R503 data queue

// System status register values
static bool busy;
static bool pass;
static bool password;
static bool image_buffer_contains_valid_image;

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
    ESP_ERROR_CHECK(uart_driver_install(uart_num, 1024, 1024, 10, &uart_queue, 0));

    uart_flush_input(uart_num); // clear UART

    response_queue = xQueueCreate(1, sizeof(size_t));

    xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, 12, NULL); // Start the UART task to poll for response

    ESP_LOGI(TAG, "UART initialized on TX=17, RX=16 @57600");

}

/**
 * 
 */
void uart_event_task(void *pvParameters) {
    
    uart_event_t event;

    while (1) {

        // TODO: should i use pdTRUE????
        if (xQueueReceive(uart_queue, &event, portMAX_DELAY) == pdTRUE) {
            
            // Wait for UART data to be received
            if (event.type == UART_DATA) {

                // Get the number of bytes from the incoming receive event
                int length = uart_read_bytes(uart_num, (R503_rx_buf + R503_rx_index), event.size, portMAX_DELAY);

                // Increment the index of the current message we are building
                R503_rx_index += length;

                // Wait until the minimum number of bytes have arrived
                if (R503_rx_index >= 9) {

                    // Get the total length of the ack packet (data from word at index 7, plus the 9 non-payload bytes)
                    uint16_t payload_length = ((R503_rx_buf[7] << 8) | R503_rx_buf[8]);
                    uint16_t total_length = payload_length + 9;

                    // Wait until all bytes have been received
                    if (R503_rx_index >= total_length) {
                        size_t packet_length = total_length;                   
                        R503_rx_index = 0;                                  // Reset local index variable
                        xQueueOverwrite(response_queue, &packet_length);    // Notify command task we done!
                    }

                }

            }

        }

    }

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
    // Print the fully assembled package
    ESP_LOGI(TAG, "Writing %d bytes", index);
    for (int i = 0; i < index; i++) {
        printf("%02X ", package[i]);
    }
    printf("\n");
    #endif

    // Write the full package
    esp_err_t ret = uart_write_bytes(uart_num, (const char*)package, index);
    if (ret < 0) return ESP_FAIL;

    return uart_wait_tx_done(uart_num, 100);
}

// /**
//  * @brief Reads a response packet from the R503 sensor into a buffer.
//  * 
//  * @param buffer Buffer to hold received data.
//  * @return Number of bytes read.
//  */
// static int read_package(uint8_t *buffer) {

//     if (!buffer) return ESP_ERR_INVALID_ARG;

//     uint8_t temp_buf[9];

//     int read_len = uart_read_bytes(uart_num, temp_buf, sizeof(temp_buf), pdMS_TO_TICKS(500));
    
//     // Check that the header is the correct size
//     if (read_len != sizeof(temp_buf)) {
//         ESP_LOGE(TAG, "Timeout or short header (%d bytes)", read_len);
//         return ESP_FAIL;
//     }

//     // Check that the header is correct 0xEF01
//     if (temp_buf[0] != 0xEF || temp_buf[1] != 0x01) {
//         ESP_LOGE(TAG, "Invalid header: %02X %02X", temp_buf[0], temp_buf[1]);
//         return ESP_FAIL;
//     }

//     // Parse payload length (last two bytes of header)
//     uint16_t length = (temp_buf[7] << 8) | temp_buf[8];

//     int total_len = sizeof(temp_buf) + length;
//     memcpy(buffer, temp_buf, sizeof(temp_buf));

//     ESP_LOGI(TAG, "Got full packet (%d bytes)", total_len);

//     // Read rest of packet to check if the length we receive matches what we sent
//     int payload_read = uart_read_bytes(uart_num, buffer + sizeof(temp_buf), length, pdMS_TO_TICKS(500));
//     if (payload_read != length) {
//         ESP_LOGE(TAG, "Timeout reading payload (%d/%d)", payload_read, length);
//         return ESP_FAIL;
//     }

//     #ifdef CONFIG_FP_DEBUG
//     // Print the raw packet for debugging
//         for (int i = 0; i < total_len; i++) printf("%02X ", buffer[i]);
//         printf("\n");
//     #endif

//     return total_len;
// }


/**
 * @brief Sends an R503 command, reads the ACK, and logs confirmation code.
 *
 * @param cmd Pointer to command bytes
 * @param length Length of command
 * @return esp_err_t ESP_OK if success (0x00), otherwise ESP_FAIL
 */
esp_err_t send_package_and_read_ack(uint8_t *cmd, uint16_t length) {

    // Clear queue
    memset(R503_rx_buf, 0, sizeof(R503_rx_buf));    // Clear the RX buffer
    R503_rx_index = 0;                              // Clear the index variable
    xQueueReset(response_queue);                    // Send notifcation to uart event task

    // Send the command
    esp_err_t err = R503_send_package(cmd, R503_COMMAND_PACKET, length);
    if (err != ESP_OK) return err;

    // Wait for the R503 response packet
    size_t rx_length;
    if (!xQueueReceive(response_queue, &rx_length, pdMS_TO_TICKS(2000))) {
        ESP_LOGE(TAG, "Timeout waiting for ACK packet");
        return ESP_ERR_TIMEOUT;
    }

    // Ensure the header is correct (0xEF01)
    if (R503_rx_buf[0] != 0xEF || R503_rx_buf[1] != 0x01) {
        ESP_LOGE(TAG, "Invalid header: %02X %02X", R503_rx_buf[0], R503_rx_buf[1]);
        return ESP_FAIL;
    }

    // Ensure the identifier nibble is correct (0x07)
    if (R503_rx_buf[6] != R503_ACKKNOWLEDGE_PACKAGE_IDENTIFIER) {
        ESP_LOGE(TAG, "Invalid ACK packet identifier: %02X", R503_rx_buf[6]);
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

    if (send_package_and_read_ack(&cmd, 1) != ESP_OK) {
        return ESP_FAIL;
    }

    if (R503_rx_buf[9] == R503_SUCCESS) {
        return ESP_OK;
    } else if (R503_rx_buf[9] == R503_NO_FINGER) {
        return ESP_ERR_NOT_FOUND;
    } 

    ESP_LOGE(TAG, "Generate image failed error code: 0x%02X", R503_rx_buf[9]);
    return ESP_FAIL;
    
}



esp_err_t wait_for_finger_and_capture(uint8_t buffer_id, TickType_t timeout) {

    TickType_t start_time = xTaskGetTickCount();

    // Wait for attempt to generate image from start_time -> start_time + timeout
    while ((xTaskGetTickCount() - start_time) < timeout) {

        esp_err_t err = generate_image();

        // If we get a good image, put it into the specified buffer
        if (err == ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(200));
            return generate_char_file_from_image(buffer_id);
        }

        // If we get no finger, wait then try again
        if (err == ESP_ERR_NOT_FOUND) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

    }

    // If it takes too long to capture the finger, timeout
    return ESP_ERR_TIMEOUT;

}

/**
 * @brief Generate character file from image
 * 
 * @param buffer_id selects the character buffer to save to 1 = buffer_1 >1 = buffer_2
 * @return esp_err_t 
 */
esp_err_t generate_char_file_from_image(uint8_t buffer_id) {

    uint8_t command[2] = {R503_GENERATE_CHAR_FILE_FROM_IMAGE, buffer_id}; 

    if (send_package_and_read_ack(command, 2) != ESP_OK)
        return ESP_FAIL;

    uint8_t code = R503_rx_buf[9];

    switch (code)
    {
        case R503_SUCCESS:       return ESP_OK;
        case R503_IMAGE_MESSY:   return ESP_ERR_INVALID_STATE;
        default:
            ESP_LOGE(TAG, "image2Tz failed: 0x%02X", code);
            return ESP_FAIL;
    }

}

/**
 * @brief Combine information of character files from character buffer 1 and 2
 * and generate template which is stored back into character buffers.
 * 
 * @return esp_err_t 
 */
esp_err_t register_model(void) {

    uint8_t cmd = R503_REGISTER_MODEL; 

    if (send_package_and_read_ack(&cmd, 1) != ESP_OK)
        return ESP_FAIL;

    return (R503_rx_buf[9] == R503_SUCCESS) ? ESP_OK : ESP_FAIL;

}

/**
 * @brief Store the template of either buffer into a location in flash library.
 * 
 * @param buffer_id selects the higher or lower buffer
 * @param location_id The index that the template will be saved to
 * @return esp_err_t 
 */
esp_err_t store_template(uint16_t location_id) {
 
    uint8_t cmd[4];
    cmd[0] = R503_STORE;
    cmd[1] = R503_CHAR_BUFFER_1;
    cmd[2] = (location_id >> 8) & 0xFF;
    cmd[3] = location_id & 0xFF;
 
    return send_package_and_read_ack(cmd, 4);

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

    return send_package_and_read_ack(cmd, 5);

}

/**
 * @brief Enables or disables the fingerprint LED ring.
 * 
 * @param enable True to turn on, false to turn off
 * @return esp_err_t ESP_OK if successful
 */
esp_err_t set_led(uint8_t led_control, uint8_t led_speed, uint8_t led_color, uint8_t num_cycles) {

    uint8_t cmd[5] = {
        R503_CONTROL_LED,
        led_control,           // Controls the current effect (0x01 - 0x06)
        led_speed,             // 0x00 - 0xFF for light breathing controls
        led_color,             // LED color
        num_cycles             // 0x00 - 0xFF for number of cycles effect will play for 0 -> infinite
    };

    return send_package_and_read_ack(cmd, 5);

}

/**
 * @brief Requests the number of stored fingerprint templates.
 * 
 * @param page determines page to check (groups of 128 fingerprints)
 * @return esp_err_t ESP_OK if successful
 */
esp_err_t read_index_table(uint8_t page) {
    uint8_t cmd[2] = {R503_READ_INDEX_TABLE, page};
    return send_package_and_read_ack(cmd, 2);
}

//
esp_err_t load_char(uint8_t buffer_id, uint16_t location_id) {

    uint8_t cmd[4] = {
        R503_LOAD_CHAR,
        buffer_id,
        (location_id >> 8) & 0xFF,
        location_id & 0xFF
    };

    return send_package_and_read_ack(cmd, 4);

}

esp_err_t read_system_parameters(void) {

    uint8_t cmd = R503_READ_SYSTEM_REGISTER;
    esp_err_t err = send_package_and_read_ack(&cmd, 1);

    if (err != ESP_OK) {
        return err;
    }
    
    // status registers from the R503
    uint16_t system_status_reg = ((uint16_t)R503_rx_buf[10] << 8 | R503_rx_buf[11]);
    uint16_t system_id_code = ((uint16_t)R503_rx_buf[12] << 8 | R503_rx_buf[13]);
    uint16_t finger_library_size = ((uint16_t)R503_rx_buf[14] << 8 | R503_rx_buf[15]);
    uint16_t security_level = ((uint16_t)R503_rx_buf[16] << 8 | R503_rx_buf[17]);
    uint32_t device_address = ((uint32_t)R503_rx_buf[18] << 24 | (uint32_t)R503_rx_buf[19] << 16 | (uint32_t)R503_rx_buf[20] << 8 | R503_rx_buf[21]);
    uint16_t data_packet_size = ((uint16_t)R503_rx_buf[22] << 8 | R503_rx_buf[23]);
    uint16_t baud_rate_N = ((uint16_t)R503_rx_buf[24] << 8 | R503_rx_buf[25]);

    // Masks for the system status register
    // busy = 0x0001;
    // pass = 0x0002;
    // password = 0x0004;
    // image_buffer_contains_valid_image = 0x0008;

    busy = system_status_reg & 0x0001;
    pass = system_status_reg & 0x0002;
    password = system_status_reg & 0x0004;
    image_buffer_contains_valid_image = system_status_reg & 0x0008;

    int baud_rate = baud_rate_N * 9600;

    ESP_LOGI(TAG, "------------SYSTEM PARAMETERS-------------");
    ESP_LOGI(TAG, "Busy: %d", busy);
    ESP_LOGI(TAG, "Pass: %d", pass);
    ESP_LOGI(TAG, "Password: %d", password);
    ESP_LOGI(TAG, "Image buffer contains valid image: %d", image_buffer_contains_valid_image);
    ESP_LOGI(TAG, "System status register: %04X", system_status_reg);
    ESP_LOGI(TAG, "System id code: %04X", system_id_code);
    ESP_LOGI(TAG, "Finger library size: %04X", finger_library_size);
    ESP_LOGI(TAG, "Security level: %04X", security_level);
    ESP_LOGI(TAG, "Device address: 0x%08lX", device_address);
    ESP_LOGI(TAG, "Data packet size: %04X", data_packet_size);
    ESP_LOGI(TAG, "Baud rate: %d", baud_rate);

    return err;

}


uint16_t R503_find_free_id(void) {

    // Parse all 4 pages
    for (uint8_t page_index = 0; page_index < 4; page_index++) {

        // Send the command to read the index table
        if (read_index_table(page_index) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read page: %02X", page_index);
            return -1;
        }

        // Get the pointer for the starting byte of the 32 byte page
        uint8_t *bitmap = &R503_rx_buf[10];

        // Parse the 32 bytes for an available template
        for (int i = 0; i < 32; i++) {

            uint8_t byte = bitmap[i];

            for (int bit = 0; bit < 8; bit++) {

                // Check byte to find the first zero
                if (!(byte & (1 << bit))) {
                    uint16_t valid_id = page_index * 256 + i * 8 + bit;
                    return valid_id;
                }
            }

        }

    }

    ESP_LOGW(TAG, "All ids are full!");
    return -1;

}

esp_err_t auto_enroll(void) {

    esp_err_t err;

    ESP_LOGI(TAG, "Beginning auto enroll, please place finger on sensor");
    set_led(R503_LED_BREATHING, 0xC0, R503_LED_COLOR_BLUE, 0x00);

    // Capture fingerprint for the first time
    if (wait_for_finger_and_capture(R503_CHAR_BUFFER_1, pdMS_TO_TICKS(10000)) != ESP_OK) {
        ESP_LOGE(TAG, "Timeout waiting for finger or messy image");
        return ESP_ERR_TIMEOUT;
    } else {
        ESP_LOGI(TAG, "First image taken! Please remove finger");
        set_led(R503_LED_ALWAYS_ON, 0x80, R503_LED_COLOR_RED, 0x00);
    }

    int iterations = 0;
    // Continuously generate image until a finger isn't recognized or we've tried generating an image 10 times
    do {
        vTaskDelay(pdMS_TO_TICKS(300));
        err = generate_image();
        iterations++;
    } while (err == ESP_OK && iterations < 10);

    if (iterations >= 10) {
        ESP_LOGI(TAG, "Failed to remove finger from sensor");
        return ESP_ERR_TIMEOUT;
    } else {
        ESP_LOGI(TAG, "Please place the same finger on sensor once more");
        set_led(R503_LED_BREATHING, 0xC0, R503_LED_COLOR_BLUE, 0x00);
    }

    // Capture fingerprint for the second time
    if (wait_for_finger_and_capture(R503_CHAR_BUFFER_2, pdMS_TO_TICKS(10000)) != ESP_OK) {
        ESP_LOGE(TAG, "Timeout waiting for finger or messy image");
        return ESP_ERR_TIMEOUT;
    }

    ESP_LOGI(TAG, "Second image taken! Attempting to store...");

    // Register model
    if (register_model() != ESP_OK) {
        ESP_LOGE(TAG, "Register model failed, fingerprints don't match");
        return ESP_FAIL;
    }

    // Find the next available id to store fingerprint
    int id = R503_find_free_id();
    if (id == -1) {
        return ESP_FAIL;
    }

    // Store the fingerprint at the specified id
    if (store_template((uint16_t)id) != ESP_OK) {
        ESP_LOGE(TAG, "Store template failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Fingerprint successfully enrolled at id: %04X", (uint16_t)id);

    return ESP_OK;

}


esp_err_t auto_identify(void) {

    // Collect finger and store in char buffer one
    if (wait_for_finger_and_capture(R503_CHAR_BUFFER_1, pdMS_TO_TICKS(10000)) != ESP_OK) {
        ESP_LOGE(TAG, "Timeout waiting for finger or messy image");
        return ESP_ERR_TIMEOUT;
    }

    uint8_t cmd[6] = {
        R503_SEARCH,
        R503_CHAR_BUFFER_1,
        0x00, 0x00, // Bytes 2 and 3 of command are the start page parameter (0x0000)
        0xFF, 0xFF  // Bytes 4 and 5 of command are the end page parameter (0xFFFF)
    }; 

    send_package_and_read_ack(cmd, 6);

    if (R503_rx_buf[9] == R503_SUCCESS) {
        uint16_t page_id = (R503_rx_buf[10] << 8 | R503_rx_buf[11]);
        uint16_t match_score = (R503_rx_buf[12] << 8 | R503_rx_buf[13]);
        ESP_LOGI(TAG, "Fingerprint match found at page ID: 0x%04X, with match score: 0x%04X", page_id, match_score);
        return ESP_OK;

    } else if (R503_rx_buf[9] == R503_NO_MATCH_IN_LIBRARY) {
        ESP_LOGI(TAG, "Fingerprint match not found in library");
        return ESP_ERR_NOT_FOUND;

    } else {
        return ESP_FAIL;
    }

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