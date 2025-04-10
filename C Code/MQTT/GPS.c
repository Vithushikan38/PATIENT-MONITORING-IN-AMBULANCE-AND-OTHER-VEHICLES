#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "mqtt_client.h"

// Hardware Configuration
#define GPS_UART_NUM UART_NUM_2
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define BUF_SIZE 1024
#define GPS_BAUD_RATE 9600

// WiFi Configuration
#define WIFI_SSID "No Internet"
#define WIFI_PASSWORD "#VithuEng38@"

// MQTT Configuration
#define MQTT_BROKER_URI "mqtt://test.mosquitto.org"
#define MQTT_TOPIC "gpslocation/001"
#define MQTT_QOS 1

// GPS Data Structure
typedef struct {
    double latitude;
    double longitude;
    double altitude;
    int satellites;
    float hdop;
    int hour;
    int minute;
    int second;
    bool valid;
} gps_data_t;

static const char *TAG = "GPS_Tracker";
static esp_mqtt_client_handle_t mqtt_client;
static QueueHandle_t gps_queue;
static bool wifi_connected = false;
static bool mqtt_connected = false;

// NMEA Parser Functions
static float convert_to_decimal(const char *nmea_coord, const char direction) {
    if (!nmea_coord || *nmea_coord == '\0') return 0.0;
    
    float raw_value = atof(nmea_coord);
    int degrees = (int)(raw_value / 100);
    float minutes = raw_value - (degrees * 100);
    float decimal_degrees = degrees + (minutes / 60.0);

    if (direction == 'S' || direction == 'W') {
        decimal_degrees *= -1;
    }
    return decimal_degrees;
}

static void parse_gpgga(const char *sentence, gps_data_t *gps) {
    char *token;
    char *saveptr;
    int field_index = 0;
    
    token = strtok_r((char *)sentence, ",", &saveptr);
    while (token != NULL && field_index < 15) {
        switch (field_index) {
            case 2:  // Latitude
                gps->latitude = convert_to_decimal(token, *(saveptr + 1));
                break;
            case 4:  // Longitude
                gps->longitude = convert_to_decimal(token, *(saveptr + 1));
                break;
            case 9:  // Altitude
                gps->altitude = atof(token);
                break;
            case 7:  // Satellites
                gps->satellites = atoi(token);
                break;
            case 8:  // HDOP
                gps->hdop = atof(token);
                break;
        }
        token = strtok_r(NULL, ",", &saveptr);
        field_index++;
    }
}

static void parse_gprmc(const char *sentence, gps_data_t *gps) {
    char *token;
    char *saveptr;
    int field_index = 0;
    
    token = strtok_r((char *)sentence, ",", &saveptr);
    while (token != NULL && field_index < 10) {
        if (field_index == 1) {  // Time
            if (strlen(token) >= 6) {
                char time_str[7];
                strncpy(time_str, token, 6);
                time_str[6] = '\0';
                gps->hour = atoi(strndup(time_str, 2));
                gps->minute = atoi(strndup(time_str + 2, 2));
                gps->second = atoi(strndup(time_str + 4, 2));
            }
        }
        token = strtok_r(NULL, ",", &saveptr);
        field_index++;
    }
}

// UART Initialization
static void uart_init() {
    uart_config_t uart_config = {
        .baud_rate = GPS_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    
    ESP_ERROR_CHECK(uart_param_config(GPS_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
}

// WiFi Event Handler
static void wifi_event_handler(void* arg, esp_event_base_t event_base, 
                             int32_t event_id, void* event_data) {
    if (event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_connected = false;
        ESP_LOGI(TAG, "Wi-Fi disconnected, reconnecting...");
        esp_wifi_connect();
    } else if (event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        wifi_connected = true;
    }
}

// WiFi Initialization
static void wifi_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

// MQTT Event Handler
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, 
                             int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT Connected");
            mqtt_connected = true;
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT Disconnected");
            mqtt_connected = false;
            break;
        default:
            break;
    }
}

// MQTT Initialization
static void mqtt_app_start(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}

// GPS Processing Task
static void gps_processing_task(void *pvParameters) {
    uint8_t *data = malloc(BUF_SIZE);
    char *buffer = malloc(BUF_SIZE * 2);
    size_t buffer_index = 0;
    gps_data_t current_data = {0};

    while (1) {
        int len = uart_read_bytes(GPS_UART_NUM, data, BUF_SIZE, pdMS_TO_TICKS(100));
        if (len > 0) {
            // Buffer management
            if (buffer_index + len >= BUF_SIZE * 2) buffer_index = 0;
            memcpy(buffer + buffer_index, data, len);
            buffer_index += len;
            buffer[buffer_index] = '\0';

            // Process complete sentences
            char *line = strtok(buffer, "\r\n");
            while (line != NULL) {
                if (strncmp(line, "$GPGGA", 6) == 0) {
                    parse_gpgga(line, &current_data);
                } else if (strncmp(line, "$GPRMC", 6) == 0) {
                    parse_gprmc(line, &current_data);
                    current_data.valid = (current_data.satellites >= 3);
                    if (current_data.valid) {
                        xQueueSend(gps_queue, &current_data, portMAX_DELAY);
                    }
                }
                line = strtok(NULL, "\r\n");
            }

            // Handle remaining buffer
            if (line) {
                size_t remaining = buffer + buffer_index - line;
                memmove(buffer, line, remaining);
                buffer_index = remaining;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    free(data);
    free(buffer);
    vTaskDelete(NULL);
}

// MQTT Publish Task
static void mqtt_publish_task(void *pvParameters) {
    gps_data_t gps_data;
    char payload[150];
    
    while (1) {
        if (xQueueReceive(gps_queue, &gps_data, portMAX_DELAY)) {
            if (wifi_connected && mqtt_connected && gps_data.valid) {
                snprintf(payload, sizeof(payload),
                        "{\"lat\":%.6f,\"lng\":%.6f,\"alt\":%.1f,"
                        "\"sat\":%d,\"hdop\":%.1f,\"time\":\"%02d:%02d:%02d\"}",
                        gps_data.latitude,
                        gps_data.longitude,
                        gps_data.altitude,
                        gps_data.satellites,
                        gps_data.hdop,
                        gps_data.hour,
                        gps_data.minute,
                        gps_data.second);

                int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC, payload, 0, MQTT_QOS, 0);
                if (msg_id < 0) {
                    ESP_LOGE(TAG, "MQTT publish failed");
                } else {
                    ESP_LOGI(TAG, "Published: %s", payload);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    vTaskDelete(NULL);
}

void app_main(void) {
    // Initialize NVS
    ESP_ERROR_CHECK(nvs_flash_init());
    
    // Initialize components
    uart_init();
    wifi_init();
    mqtt_app_start();

    // Create GPS data queue
    gps_queue = xQueueCreate(5, sizeof(gps_data_t));
    
    // Create tasks
    xTaskCreate(gps_processing_task, "gps_task", 4096, NULL, 5, NULL);
    xTaskCreate(mqtt_publish_task, "mqtt_task", 4096, NULL, 5, NULL);
}
