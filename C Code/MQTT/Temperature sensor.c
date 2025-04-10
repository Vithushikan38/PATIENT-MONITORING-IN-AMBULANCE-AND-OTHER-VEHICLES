/* ESP-IDF DS18B20 Temperature Sensor to MQTT
 * 
 * This application implements direct OneWire communication with DS18B20
 * and publishes temperature readings to an MQTT broker.
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h> // For PRId32 macros
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "mqtt_client.h"

/* DS18B20 Configuration */
#define ONE_WIRE_GPIO 4
#define DS18B20_FAMILY_CODE 0x28
#define DS18B20_CONVERT_T 0x44
#define DS18B20_READ_SCRATCHPAD 0xBE

/* WiFi Configuration */
#define WIFI_SSID "No Internet"
#define WIFI_PASSWORD "#VithuEng38@"
#define MAXIMUM_RETRY 5

/* MQTT Configuration */
#define MQTT_BROKER "test.mosquitto.org"
#define MQTT_PORT 1883
#define MQTT_TOPIC "home/temperature/dallas"

static const char *TAG = "TEMP_MQTT";

/* FreeRTOS event group to signal when we are connected */
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int s_retry_num = 0;
static esp_mqtt_client_handle_t mqtt_client = NULL;

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

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"Connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
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

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    // FIXED: proper format specifier for int32_t
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRId32, base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
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

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://" MQTT_BROKER,
        .broker.address.port = MQTT_PORT,
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}

void temp_sensor_task(void *pvParameters)
{
    char temp_str[10];
    float temperature;
    
    // FIXED: Properly configure GPIO using newer ESP-IDF API
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ONE_WIRE_GPIO),
        .mode = GPIO_MODE_INPUT_OUTPUT,  // GPIO can be both input and output for OneWire
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    
    // Main loop to read and publish temperature
    while (1) {
        if (ds18b20_read_temp(ONE_WIRE_GPIO, &temperature) == ESP_OK) {
            ESP_LOGI(TAG, "Temperature: %.2f °C", temperature);
            
            // Convert temperature to string
            snprintf(temp_str, sizeof(temp_str), "%.2f", temperature);
            
            // Publish to MQTT if connected
            if (mqtt_client != NULL) {
                int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC, temp_str, 0, 1, 0);
                if (msg_id != -1) {
                    ESP_LOGI(TAG, "Temperature published successfully, msg_id=%d", msg_id);
                } else {
                    ESP_LOGE(TAG, "Failed to publish temperature");
                }
            }
        } else {
            ESP_LOGE(TAG, "Error reading temperature");
        }
        
        vTaskDelay(pdMS_TO_TICKS(5000)); // Read and publish every 5 seconds
    }
}

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize WiFi
    wifi_init_sta();
    
    // Initialize MQTT
    mqtt_app_start();
    
    // Create task for temperature sensing
    xTaskCreate(temp_sensor_task, "temp_sensor_task", 4096, NULL, 5, NULL);
}