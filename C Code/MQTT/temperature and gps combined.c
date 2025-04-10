/* ESP-IDF DS18B20 Temperature Sensor and GPS to MQTT
 * 
 * This application reads temperature from DS18B20 sensor and GPS data
 * then publishes both to an MQTT broker under the same topic.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h> // For PRId32 macros
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "mqtt_client.h"

/* DS18B20 Configuration */
#define ONE_WIRE_GPIO 4
#define DS18B20_FAMILY_CODE 0x28
#define DS18B20_CONVERT_T 0x44
#define DS18B20_READ_SCRATCHPAD 0xBE

/* GPS Configuration */
#define GPS_UART_NUM UART_NUM_2
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define BUF_SIZE 1024
#define GPS_BAUD_RATE 9600

/* WiFi Configuration */
#define WIFI_SSID "No Internet"
#define WIFI_PASSWORD "#VithuEng38@"
#define MAXIMUM_RETRY 5

/* MQTT Configuration */
#define MQTT_BROKER "test.mosquitto.org"
#define MQTT_PORT 1883
#define MQTT_TOPIC "sensors/combined_data"
#define MQTT_QOS 1

static const char *TAG = "TEMP_GPS_MQTT";

/* FreeRTOS event group to signal when we are connected */
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int s_retry_num = 0;
static esp_mqtt_client_handle_t mqtt_client = NULL;
static QueueHandle_t gps_queue;
static QueueHandle_t temp_queue;
static bool wifi_connected = false;
static bool mqtt_connected = false;

/* GPS Data Structure */
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

/* Temperature Data Structure */
typedef struct {
    float temperature;
    bool valid;
} temp_data_t;

/* Combined Data Structure for MQTT */
typedef struct {
    float temperature;
    double latitude;
    double longitude;
    double altitude;
    int satellites;
    float hdop;
    int hour;
    int minute;
    int second;
    bool temp_valid;
    bool gps_valid;
} combined_data_t;

/* OneWire functions for DS18B20 */

// Reset the OneWire bus and check for device presence
static bool onewire_reset(gpio_num_t pin) {
    // Configure GPIO for output
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    
    // Pull low for at least 480us
    gpio_set_level(pin, 0);
    esp_rom_delay_us(500);
    
    // Release the bus and switch to input
    gpio_set_level(pin, 1);
    gpio_set_direction(pin, GPIO_MODE_INPUT);
    
    // Wait for presence pulse (15-60µs after release)
    esp_rom_delay_us(70);
    
    // Read presence (0 = device present, 1 = no device)
    bool presence = gpio_get_level(pin) == 0;
    
    // Wait for the rest of the presence pulse
    esp_rom_delay_us(410);
    
    return presence;
}

// Write a bit to the OneWire bus
static void onewire_write_bit(gpio_num_t pin, bool bit) {
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    
    // Pull low
    gpio_set_level(pin, 0);
    
    if (bit) {
        // For a '1' bit: release within 15µs
        esp_rom_delay_us(10);
        gpio_set_level(pin, 1);
        esp_rom_delay_us(55);
    } else {
        // For a '0' bit: keep low for at least 60µs
        esp_rom_delay_us(65);
        gpio_set_level(pin, 1);
        esp_rom_delay_us(5);
    }
}

// Read a bit from the OneWire bus
static bool onewire_read_bit(gpio_num_t pin) {
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    
    // Pull low briefly
    gpio_set_level(pin, 0);
    esp_rom_delay_us(3);
    
    // Release the bus and switch to input
    gpio_set_level(pin, 1);
    gpio_set_direction(pin, GPIO_MODE_INPUT);
    
    // Wait a bit and then sample
    esp_rom_delay_us(10);
    bool bit = gpio_get_level(pin);
    
    // Wait for the rest of the read slot
    esp_rom_delay_us(53);
    
    return bit;
}

// Write a byte to the OneWire bus
static void onewire_write_byte(gpio_num_t pin, uint8_t data) {
    for (int i = 0; i < 8; i++) {
        onewire_write_bit(pin, data & 0x01);
        data >>= 1;
    }
}

// Read a byte from the OneWire bus
static uint8_t onewire_read_byte(gpio_num_t pin) {
    uint8_t data = 0;
    for (int i = 0; i < 8; i++) {
        // Shift one bit right, set highest bit if needed
        data >>= 1;
        if (onewire_read_bit(pin)) {
            data |= 0x80;
        }
    }
    return data;
}

// Calculate CRC8 for OneWire
static uint8_t onewire_crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0;
    
    for (size_t i = 0; i < len; i++) {
        uint8_t inbyte = data[i];
        for (int j = 0; j < 8; j++) {
            uint8_t mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if (mix) crc ^= 0x8C;
            inbyte >>= 1;
        }
    }
    
    return crc;
}

// Read temperature from DS18B20
static esp_err_t ds18b20_read_temp(gpio_num_t pin, float *temperature) {
    uint8_t scratchpad[9];
    int16_t raw_temp;
    
    // Reset and check presence
    if (!onewire_reset(pin)) {
        ESP_LOGE(TAG, "DS18B20 not detected");
        return ESP_ERR_NOT_FOUND;
    }
    
    // Skip ROM command
    onewire_write_byte(pin, 0xCC);
    
    // Start temperature conversion
    onewire_write_byte(pin, DS18B20_CONVERT_T);
    
    // Wait for conversion to complete
    vTaskDelay(pdMS_TO_TICKS(750));
    
    // Reset and check presence again
    if (!onewire_reset(pin)) {
        ESP_LOGE(TAG, "DS18B20 lost during conversion");
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    // Skip ROM command again
    onewire_write_byte(pin, 0xCC);
    
    // Read scratchpad command
    onewire_write_byte(pin, DS18B20_READ_SCRATCHPAD);
    
    // Read scratchpad data (9 bytes)
    for (int i = 0; i < 9; i++) {
        scratchpad[i] = onewire_read_byte(pin);
    }
    
    // Verify CRC
    if (onewire_crc8(scratchpad, 8) != scratchpad[8]) {
        ESP_LOGE(TAG, "DS18B20 CRC check failed");
        return ESP_ERR_INVALID_CRC;
    }
    
    // Convert to temperature
    raw_temp = (scratchpad[1] << 8) | scratchpad[0];
    
    // Different conversion methods based on the sensor's resolution configuration
    // Default is 12-bit (0.0625°C)
    *temperature = (float)raw_temp * 0.0625;
    
    return ESP_OK;
}

/* NMEA Parser Functions */
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

/* UART Initialization for GPS */
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

/* WiFi Event Handler */
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_connected = false;
        if (s_retry_num < MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "Connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        wifi_connected = true;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/* WiFi Initialization */
void wifi_init_sta(void) {
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                       ESP_EVENT_ANY_ID,
                                                       &wifi_event_handler,
                                                       NULL,
                                                       NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                       IP_EVENT_STA_GOT_IP,
                                                       &wifi_event_handler,
                                                       NULL,
                                                       NULL));

    // Set WiFi configuration
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by wifi_event_handler() */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s", WIFI_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s", WIFI_SSID);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

/* MQTT Event Handler */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRId32, base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        mqtt_connected = true;
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        mqtt_connected = false;
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

/* MQTT Initialization */
static void mqtt_app_start(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://" MQTT_BROKER,
        .broker.address.port = MQTT_PORT,
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}

/* Temperature Sensor Task */
void temp_sensor_task(void *pvParameters) {
    temp_data_t temp_data;
    
    // Configure GPIO for OneWire
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ONE_WIRE_GPIO),
        .mode = GPIO_MODE_INPUT_OUTPUT,  // GPIO can be both input and output for OneWire
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    
    // Main loop to read temperature
    while (1) {
        if (ds18b20_read_temp(ONE_WIRE_GPIO, &temp_data.temperature) == ESP_OK) {
            ESP_LOGI(TAG, "Temperature: %.2f °C", temp_data.temperature);
            temp_data.valid = true;
        } else {
            ESP_LOGE(TAG, "Error reading temperature");
            temp_data.valid = false;
        }
        
        // Send temperature data to queue
        xQueueSend(temp_queue, &temp_data, pdMS_TO_TICKS(100));
        
        vTaskDelay(pdMS_TO_TICKS(5000)); // Read temperature every 5 seconds
    }
}

/* GPS Processing Task */
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

/* MQTT Publish Combined Data Task */
static void mqtt_publish_task(void *pvParameters) {
    gps_data_t gps_data = {0};
    temp_data_t temp_data = {0};
    combined_data_t combined_data = {0};
    char payload[200];
    
    // Initialize combined data as invalid
    combined_data.temp_valid = false;
    combined_data.gps_valid = false;
    
    while (1) {
        // Check for new temperature data
        if (xQueueReceive(temp_queue, &temp_data, 0)) {
            combined_data.temperature = temp_data.temperature;
            combined_data.temp_valid = temp_data.valid;
            ESP_LOGI(TAG, "Got new temperature: %.2f °C", temp_data.temperature);
        }
        
        // Check for new GPS data
        if (xQueueReceive(gps_queue, &gps_data, 0)) {
            combined_data.latitude = gps_data.latitude;
            combined_data.longitude = gps_data.longitude;
            combined_data.altitude = gps_data.altitude;
            combined_data.satellites = gps_data.satellites;
            combined_data.hdop = gps_data.hdop;
            combined_data.hour = gps_data.hour;
            combined_data.minute = gps_data.minute;
            combined_data.second = gps_data.second;
            combined_data.gps_valid = gps_data.valid;
            ESP_LOGI(TAG, "Got new GPS data: lat=%.6f, lng=%.6f", gps_data.latitude, gps_data.longitude);
        }
        
        // Publish combined data if connected and we have at least one valid data type
        if (wifi_connected && mqtt_connected && (combined_data.temp_valid || combined_data.gps_valid)) {
            // Construct JSON payload based on what data is valid
            if (combined_data.temp_valid && combined_data.gps_valid) {
                snprintf(payload, sizeof(payload),
                        "{\"temp\":%.2f,\"lat\":%.6f,\"lng\":%.6f,\"alt\":%.1f,"
                        "\"sat\":%d,\"hdop\":%.1f,\"time\":\"%02d:%02d:%02d\"}",
                        combined_data.temperature,
                        combined_data.latitude,
                        combined_data.longitude,
                        combined_data.altitude,
                        combined_data.satellites,
                        combined_data.hdop,
                        combined_data.hour,
                        combined_data.minute,
                        combined_data.second);
            } else if (combined_data.temp_valid) {
                snprintf(payload, sizeof(payload),
                        "{\"temp\":%.2f,\"gps_valid\":false}",
                        combined_data.temperature);
            } else if (combined_data.gps_valid) {
                snprintf(payload, sizeof(payload),
                        "{\"temp_valid\":false,\"lat\":%.6f,\"lng\":%.6f,\"alt\":%.1f,"
                        "\"sat\":%d,\"hdop\":%.1f,\"time\":\"%02d:%02d:%02d\"}",
                        combined_data.latitude,
                        combined_data.longitude,
                        combined_data.altitude,
                        combined_data.satellites,
                        combined_data.hdop,
                        combined_data.hour,
                        combined_data.minute,
                        combined_data.second);
            }

            int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC, payload, 0, MQTT_QOS, 0);
            if (msg_id < 0) {
                ESP_LOGE(TAG, "MQTT publish failed");
            } else {
                ESP_LOGI(TAG, "Published combined data: %s", payload);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(2000)); // Try to publish every 2 seconds
    }
    vTaskDelete(NULL);
}

void app_main(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize UART for GPS
    uart_init();
    
    // Initialize WiFi
    wifi_init_sta();
    
    // Initialize MQTT
    mqtt_app_start();
    
    // Create data queues
    gps_queue = xQueueCreate(5, sizeof(gps_data_t));
    temp_queue = xQueueCreate(5, sizeof(temp_data_t));
    
    // Create tasks
    xTaskCreate(temp_sensor_task, "temp_sensor_task", 4096, NULL, 5, NULL);
    xTaskCreate(gps_processing_task, "gps_task", 4096, NULL, 5, NULL);
    xTaskCreate(mqtt_publish_task, "mqtt_task", 4096, NULL, 5, NULL);
}