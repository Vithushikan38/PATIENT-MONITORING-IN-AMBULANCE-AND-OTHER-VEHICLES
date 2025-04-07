#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define RXD2 16  // GPIO pin for RX
#define TXD2 17  // GPIO pin for TX

#define UART_PORT UART_NUM_2  // Use UART2
#define BUF_SIZE (1024)       // Buffer size for UART data

static const char *TAG = "Location:";

// Function to convert NMEA latitude/longitude to decimal degrees
float convert_to_decimal(char *nmea_coord, char *direction) {
    float raw_value = atof(nmea_coord);
    int degrees = (int)(raw_value / 100);
    float minutes = raw_value - (degrees * 100);
    float decimal_degrees = degrees + (minutes / 60.0);

    // Adjust for hemisphere (S or W should be negative)
    if (direction[0] == 'S' || direction[0] == 'W') {
        decimal_degrees *= -1;
    }
    return decimal_degrees;
}

// Function to parse NMEA sentence and extract latitude & longitude
void parse_nmea_sentence(char *sentence) {
    if (strncmp(sentence, "$GPGGA", 6) == 0 || strncmp(sentence, "$GPRMC", 6) == 0) {
        char *token;
        char *nmea_data[15];  // Array to hold parsed NMEA fields
        int index = 0;

        // Tokenize the sentence
        token = strtok(sentence, ",");
        while (token != NULL && index < 15) {
            nmea_data[index++] = token;
            token = strtok(NULL, ",");
        }

        // Extract latitude and longitude from the appropriate NMEA fields
        if (index > 6 && nmea_data[3] != NULL && nmea_data[5] != NULL) {
            float latitude = convert_to_decimal(nmea_data[3], nmea_data[4]);
            float longitude = convert_to_decimal(nmea_data[5], nmea_data[6]);

            ESP_LOGI(TAG, "Latitude: %.6f, Longitude: %.6f", latitude, longitude);
        }
    }
}

void uart_task(void *arg) {
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    if (data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory!");
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        int len = uart_read_bytes(UART_PORT, data, BUF_SIZE - 1, 100 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = '\0';  // Null-terminate to handle as string
            char *line = strtok((char *)data, "\r\n");  // Process each NMEA sentence
            while (line != NULL) {
                parse_nmea_sentence(line);
                line = strtok(NULL, "\r\n");
            }
        }
    }

    free(data);
    vTaskDelete(NULL);
}

void app_main(void) {
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, TXD2, RXD2, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Create a FreeRTOS task to handle UART reading
    xTaskCreate(uart_task, "uart_task", 4096, NULL, 5, NULL);
}
