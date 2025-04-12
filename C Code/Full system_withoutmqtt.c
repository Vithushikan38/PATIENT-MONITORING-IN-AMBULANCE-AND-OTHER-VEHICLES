// main.c for ESP-IDF framework
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_system.h"
#include "sdkconfig.h"

static const char *TAG = "VitalSync";

// GPIO Pin Definitions
#define BUTTON_PIN          13
#define BUZZER_PIN          25
#define LED_PIN             26
#define LED_PINA            32
#define LED_PINB            18
#define ONE_WIRE_BUS        4

// I2C Configuration
#define I2C_MASTER_SCL      22    // SCL pin
#define I2C_MASTER_SDA      21    // SDA pin
#define I2C_MASTER_PORT     I2C_NUM_0
#define I2C_MASTER_FREQ_HZ  400000

// UART Configuration for GPS
#define GPS_UART_NUM        UART_NUM_2
#define GPS_UART_RX_PIN     16
#define GPS_UART_TX_PIN     17
#define GPS_UART_BAUD_RATE  9600
#define GPS_UART_BUF_SIZE   1024

// OLED Display Configuration
#define SCREEN_WIDTH        128
#define SCREEN_HEIGHT       64
#define OLED_ADDR           0x3C

// SSD1306 Commands
#define SSD1306_ADDR                    0x3C
#define SSD1306_CMD_DISPLAY_OFF         0xAE
#define SSD1306_CMD_DISPLAY_ON          0xAF
#define SSD1306_CMD_SET_CONTRAST        0x81
#define SSD1306_CMD_DISPLAY_ALL_ON_RESUME 0xA4
#define SSD1306_CMD_DISPLAY_ALL_ON      0xA5
#define SSD1306_CMD_NORMAL_DISPLAY      0xA6
#define SSD1306_CMD_INVERT_DISPLAY      0xA7
#define SSD1306_CMD_SET_MULTIPLEX       0xA8
#define SSD1306_CMD_SET_DISPLAY_OFFSET  0xD3
#define SSD1306_CMD_SET_START_LINE      0x40
#define SSD1306_CMD_SET_SEGMENT_REMAP   0xA0
#define SSD1306_CMD_SET_COM_SCAN_DIR    0xC0
#define SSD1306_CMD_SET_COM_PINS        0xDA
#define SSD1306_CMD_SET_MEMORY_ADDR_MODE 0x20
#define SSD1306_CMD_SET_COLUMN_ADDR     0x21
#define SSD1306_CMD_SET_PAGE_ADDR       0x22
#define SSD1306_CMD_CHARGE_PUMP         0x8D

// MAX30102 registers
#define MAX30105_ADDRESS           0x57
#define MAX30105_REG_INTR_STATUS_1 0x00
#define MAX30105_REG_INTR_STATUS_2 0x01
#define MAX30105_REG_INTR_ENABLE_1 0x02
#define MAX30105_REG_INTR_ENABLE_2 0x03
#define MAX30105_REG_FIFO_WR_PTR   0x04
#define MAX30105_REG_FIFO_OVF      0x05
#define MAX30105_REG_FIFO_RD_PTR   0x06
#define MAX30105_REG_FIFO_DATA     0x07
#define MAX30105_REG_MODE_CONFIG   0x09
#define MAX30105_REG_SPO2_CONFIG   0x0A
#define MAX30105_REG_LED1_PA       0x0C
#define MAX30105_REG_LED2_PA       0x0D
#define MAX30105_REG_LED3_PA       0x0E
#define MAX30105_REG_MULTI_LED     0x11
#define MAX30105_REG_TEMP_INTR     0x1F
#define MAX30105_REG_TEMP_DATA     0x20
#define MAX30105_REG_REV_ID        0xFE
#define MAX30105_REG_PART_ID       0xFF

// Variables for storing SpO2 and heart rate readings
uint32_t irBuffer[100];  // infrared LED sensor data
uint32_t redBuffer[100]; // red LED sensor data
int32_t bufferLength = 100; // data length
int32_t spo2 = 0; // SPO2 value
int8_t validSPO2 = 0; // indicator to show if the SPO2 calculation is valid
int32_t heartRate = 0; // heart rate value
int8_t validHeartRate = 0; // indicator to show if the heart rate calculation is valid

// GPS data structure
typedef struct {
    double latitude;
    double longitude;
    uint8_t day;
    uint8_t month;
    uint16_t year;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    bool location_valid;
    bool time_valid;
    bool date_valid;
} gps_data_t;

gps_data_t gps_data = {0};

// Function prototypes
static esp_err_t i2c_master_init(void);
static esp_err_t ssd1306_init(void);
static esp_err_t ssd1306_clear(void);
static esp_err_t ssd1306_display_text(const char *text, int x, int y, int size);
static esp_err_t max30105_init(void);
static esp_err_t max30105_read_sensor(void);
static float ds18b20_read_temp(void);
static void parse_gps_data(const char* buffer, size_t len);
static void show_message(const char* message, int size, int x, int y, int delay_time);
static void show_patient_report(float temp_c, int32_t bpm, int32_t spo2_value);
static void show_gps_data(void);

// I2C master initialization
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA,
        .scl_io_num = I2C_MASTER_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    esp_err_t ret = i2c_param_config(I2C_MASTER_PORT, &conf);
    if (ret != ESP_OK) {
        return ret;
    }
    
    return i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0);
}

// Write to I2C device
static esp_err_t i2c_write_byte(uint8_t device_addr, uint8_t reg_addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Read from I2C device
static esp_err_t i2c_read_byte(uint8_t device_addr, uint8_t reg_addr, uint8_t *data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Initialize SSD1306 OLED display
static esp_err_t ssd1306_init(void) {
    esp_err_t ret;
    
    // Send initialization commands
    uint8_t init_cmds[] = {
        SSD1306_CMD_DISPLAY_OFF,
        SSD1306_CMD_SET_MULTIPLEX, 0x3F,
        SSD1306_CMD_SET_DISPLAY_OFFSET, 0x00,
        SSD1306_CMD_SET_START_LINE | 0x00,
        SSD1306_CMD_SET_SEGMENT_REMAP | 0x01,
        SSD1306_CMD_SET_COM_SCAN_DIR | 0x08,
        SSD1306_CMD_SET_COM_PINS, 0x12,
        SSD1306_CMD_SET_CONTRAST, 0x7F,
        SSD1306_CMD_DISPLAY_ALL_ON_RESUME,
        SSD1306_CMD_NORMAL_DISPLAY,
        SSD1306_CMD_CHARGE_PUMP, 0x14,
        SSD1306_CMD_DISPLAY_ON
    };
    
    for (int i = 0; i < sizeof(init_cmds); i++) {
        ret = i2c_write_byte(OLED_ADDR, 0x00, init_cmds[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "OLED init command %d failed", i);
            return ret;
        }
    }
    
    return ESP_OK;
}

// Clear SSD1306 display
static esp_err_t ssd1306_clear(void) {
    esp_err_t ret;
    
    // Set addressing mode
    ret = i2c_write_byte(OLED_ADDR, 0x00, SSD1306_CMD_SET_MEMORY_ADDR_MODE);
    if (ret != ESP_OK) return ret;
    ret = i2c_write_byte(OLED_ADDR, 0x00, 0x00); // Horizontal addressing mode
    if (ret != ESP_OK) return ret;
    
    // Set column address range
    ret = i2c_write_byte(OLED_ADDR, 0x00, SSD1306_CMD_SET_COLUMN_ADDR);
    if (ret != ESP_OK) return ret;
    ret = i2c_write_byte(OLED_ADDR, 0x00, 0x00); // Start
    if (ret != ESP_OK) return ret;
    ret = i2c_write_byte(OLED_ADDR, 0x00, SCREEN_WIDTH - 1); // End
    if (ret != ESP_OK) return ret;
    
    // Set page address range
    ret = i2c_write_byte(OLED_ADDR, 0x00, SSD1306_CMD_SET_PAGE_ADDR);
    if (ret != ESP_OK) return ret;
    ret = i2c_write_byte(OLED_ADDR, 0x00, 0x00); // Start
    if (ret != ESP_OK) return ret;
    ret = i2c_write_byte(OLED_ADDR, 0x00, (SCREEN_HEIGHT / 8) - 1); // End
    if (ret != ESP_OK) return ret;
    
    // Write zeros to clear the display
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OLED_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x40, true); // Data mode
    
    // Write 1024 bytes (128x64 / 8)
    for (int i = 0; i < (SCREEN_WIDTH * SCREEN_HEIGHT / 8); i++) {
        i2c_master_write_byte(cmd, 0x00, true);
    }
    
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

// Simple implementation to display text on OLED
// This is a simplified version - in a real application, you'd need a font table
static esp_err_t ssd1306_display_text(const char *text, int x, int y, int size) {
    // This is a placeholder for actual text display implementation
    // In a real implementation, you would:
    // 1. Convert text to bitmap using font data
    // 2. Set column and page address for the location
    // 3. Write bitmap data to the display
    
    ESP_LOGI(TAG, "Display text at (%d,%d): %s", x, y, text);
    return ESP_OK;
}

// Initialize MAX30105 sensor
static esp_err_t max30105_init(void) {
    esp_err_t ret;
    uint8_t data;
    
    // Reset the sensor
    ret = i2c_write_byte(MAX30105_ADDRESS, MAX30105_REG_MODE_CONFIG, 0x40);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(100)); // Wait for reset
    
    // Read Part ID to check if sensor is present
    ret = i2c_read_byte(MAX30105_ADDRESS, MAX30105_REG_PART_ID, &data);
    if (ret != ESP_OK) return ret;
    
    if (data != 0x15) { // MAX30105 Part ID should be 0x15
        ESP_LOGE(TAG, "MAX30105 not found, read ID: 0x%02x", data);
        return ESP_FAIL;
    }
    
    // Configure sensor
    // Set FIFO configuration
    ret = i2c_write_byte(MAX30105_ADDRESS, MAX30105_REG_FIFO_WR_PTR, 0x00);
    if (ret != ESP_OK) return ret;
    ret = i2c_write_byte(MAX30105_ADDRESS, MAX30105_REG_FIFO_OVF, 0x00);
    if (ret != ESP_OK) return ret;
    ret = i2c_write_byte(MAX30105_ADDRESS, MAX30105_REG_FIFO_RD_PTR, 0x00);
    if (ret != ESP_OK) return ret;
    
    // Set mode to SpO2
    ret = i2c_write_byte(MAX30105_ADDRESS, MAX30105_REG_MODE_CONFIG, 0x03); // SpO2 mode
    if (ret != ESP_OK) return ret;
    
    // SpO2 configuration
    ret = i2c_write_byte(MAX30105_ADDRESS, MAX30105_REG_SPO2_CONFIG, 0x27); // SPO2_ADC=4096nA, 100Hz, 411us
    if (ret != ESP_OK) return ret;
    
    // LED pulse amplitude
    ret = i2c_write_byte(MAX30105_ADDRESS, MAX30105_REG_LED1_PA, 0x3F); // Red LED ~ 10mA
    if (ret != ESP_OK) return ret;
    ret = i2c_write_byte(MAX30105_ADDRESS, MAX30105_REG_LED2_PA, 0x3F); // IR LED ~ 10mA
    if (ret != ESP_OK) return ret;
    
    return ESP_OK;
}

// Read MAX30105 sensor data
static esp_err_t max30105_read_sensor(void) {
    // Read FIFO Write Pointer
    uint8_t write_ptr;
    esp_err_t ret = i2c_read_byte(MAX30105_ADDRESS, MAX30105_REG_FIFO_WR_PTR, &write_ptr);
    if (ret != ESP_OK) return ret;
    
    // Read FIFO Read Pointer
    uint8_t read_ptr;
    ret = i2c_read_byte(MAX30105_ADDRESS, MAX30105_REG_FIFO_RD_PTR, &read_ptr);
    if (ret != ESP_OK) return ret;
    
    // Calculate number of samples
    int num_samples = (write_ptr - read_ptr) & 0x1F;
    if (num_samples == 0) num_samples = 32; // Buffer is full
    
    // Read samples
    for (int i = 0; i < num_samples; i++) {
        // Each sample is 3 bytes (24 bits) for each LED
        // We need to read RED and IR LED data
        uint8_t red_data[3], ir_data[3];
        
        // Start reading FIFO data
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (MAX30105_ADDRESS << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, MAX30105_REG_FIFO_DATA, true);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (MAX30105_ADDRESS << 1) | I2C_MASTER_READ, true);
        
        // Read RED (first channel in multi-LED mode)
        i2c_master_read(cmd, red_data, 3, I2C_MASTER_ACK);
        
        // Read IR (second channel in multi-LED mode)
        i2c_master_read(cmd, ir_data, 3, I2C_MASTER_LAST_NACK);
        
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        
        if (ret != ESP_OK) return ret;
        
        // Combine bytes to get 24-bit value
        redBuffer[i] = ((uint32_t)red_data[0] << 16) | ((uint32_t)red_data[1] << 8) | red_data[2];
        irBuffer[i] = ((uint32_t)ir_data[0] << 16) | ((uint32_t)ir_data[1] << 8) | ir_data[2];
    }
    
    // Simplified algorithm to calculate heart rate and SpO2
    // In a real application, you'd use a proper algorithm like maxim_heart_rate_and_oxygen_saturation
    // For this example, we'll just set some plausible values
    
    // Check if there's enough signal
    bool signal_detected = false;
    uint32_t ir_sum = 0;
    
    for (int i = 0; i < num_samples; i++) {
        ir_sum += irBuffer[i];
    }
    
    float ir_avg = ir_sum / num_samples;
    
    // Check if IR signal is strong enough
    if (ir_avg > 50000) { // Arbitrary threshold
        signal_detected = true;
        
        // Calculate heartRate (simplified)
        heartRate = 60 + rand() % 40; // Random value between 60-100 for demonstration
        validHeartRate = 1;
        
        // Calculate SpO2 (simplified)
        spo2 = 95 + rand() % 5; // Random value between 95-99 for demonstration
        validSPO2 = 1;
    } else {
        validHeartRate = 0;
        validSPO2 = 0;
    }
    
    return ESP_OK;
}

// Read temperature from DS18B20 (simplified implementation)
static float ds18b20_read_temp(void) {
    // In a real implementation, you would:
    // 1. Set up 1-Wire protocol for DS18B20
    // 2. Send temperature conversion command
    // 3. Wait for conversion to complete
    // 4. Read temperature data
    
    // For this example, we'll simulate a reading
    return 36.5f + ((float)(rand() % 20) / 10.0f); // 36.5Â°C to 38.5Â°C
}

// Parse GPS NMEA data
static void parse_gps_data(const char* buffer, size_t len) {
    // This is a simplified NMEA parser
    // In a real application, you'd use a proper NMEA parser
    
    // Example of parsing a GGA sentence (for position)
    // $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
    
    if (strstr(buffer, "$GPGGA") || strstr(buffer, "$GNGGA")) {
        char* sentence = strdup(buffer);
        if (sentence) {
            char* token = strtok(sentence, ",");
            int field = 0;
            
            while (token != NULL) {
                field++;
                
                if (field == 2) { // Time
                    int time = atoi(token);
                    if (time > 0) {
                        gps_data.hour = time / 10000;
                        gps_data.minute = (time / 100) % 100;
                        gps_data.second = time % 100;
                        gps_data.time_valid = true;
                    }
                } else if (field == 3) { // Latitude
                    double lat = atof(token);
                    int degrees = (int)(lat / 100);
                    double minutes = lat - (degrees * 100);
                    gps_data.latitude = degrees + (minutes / 60.0);
                } else if (field == 4) { // N/S
                    if (token[0] == 'S') {
                        gps_data.latitude = -gps_data.latitude;
                    }
                } else if (field == 5) { // Longitude
                    double lon = atof(token);
                    int degrees = (int)(lon / 100);
                    double minutes = lon - (degrees * 100);
                    gps_data.longitude = degrees + (minutes / 60.0);
                } else if (field == 6) { // E/W
                    if (token[0] == 'W') {
                        gps_data.longitude = -gps_data.longitude;
                    }
                } else if (field == 7) { // Fix quality
                    if (atoi(token) > 0) {
                        gps_data.location_valid = true;
                    } else {
                        gps_data.location_valid = false;
                    }
                }
                
                token = strtok(NULL, ",");
            }
            
            free(sentence);
        }
    }
    
    // Parse RMC sentence for date
    // $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
    
    if (strstr(buffer, "$GPRMC") || strstr(buffer, "$GNRMC")) {
        char* sentence = strdup(buffer);
        if (sentence) {
            char* token = strtok(sentence, ",");
            int field = 0;
            
            while (token != NULL) {
                field++;
                
                if (field == 9) { // Date
                    int date = atoi(token);
                    if (date > 0) {
                        gps_data.day = date / 10000;
                        gps_data.month = (date / 100) % 100;
                        gps_data.year = 2000 + (date % 100); // Assuming 21st century
                        gps_data.date_valid = true;
                    }
                }
                
                token = strtok(NULL, ",");
            }
            
            free(sentence);
        }
    }
}

// Show message on OLED
static void show_message(const char* message, int size, int x, int y, int delay_time) {
    ssd1306_clear();
    ssd1306_display_text(message, x, y, size);
    vTaskDelay(pdMS_TO_TICKS(delay_time));
}

// Display patient report
static void show_patient_report(float temp_c, int32_t bpm, int32_t spo2_value) {
    char buffer[64];
    
    ssd1306_clear();
    ssd1306_display_text("Patient Report:", 10, 5, 1);
    
    snprintf(buffer, sizeof(buffer), "Temp: %.1fÂ°C", temp_c);
    ssd1306_display_text(buffer, 10, 20, 1);
    
    snprintf(buffer, sizeof(buffer), "HR: %ld BPM", bpm);
    ssd1306_display_text(buffer, 10, 35, 1);
    
    snprintf(buffer, sizeof(buffer), "SpO2: %ld %%", spo2_value);
    ssd1306_display_text(buffer, 10, 50, 1);
    
    vTaskDelay(pdMS_TO_TICKS(3000));
}

// Display GPS info
static void show_gps_data(void) {
    char buffer[64];
    
    ssd1306_clear();
    ssd1306_display_text("GPS Details:", 10, 5, 1);
    
    if (gps_data.location_valid) {
        snprintf(buffer, sizeof(buffer), "Lat: %.6f", gps_data.latitude);
        ssd1306_display_text(buffer, 10, 20, 1);
        
        snprintf(buffer, sizeof(buffer), "Lon: %.6f", gps_data.longitude);
        ssd1306_display_text(buffer, 10, 30, 1);
    } else {
        ssd1306_display_text("GPS: No Fix", 10, 20, 1);
    }
    
    if (gps_data.date_valid) {
        snprintf(buffer, sizeof(buffer), "Date: %02d/%02d/%04d", 
                gps_data.day, gps_data.month, gps_data.year);
        ssd1306_display_text(buffer, 10, 40, 1);
    } else {
        ssd1306_display_text("Date: N/A", 10, 40, 1);
    }
    
    if (gps_data.time_valid) {
        // Adjust for IST (UTC+5:30)
        int hour = gps_data.hour + 5;
        int minute = gps_data.minute + 30;
        
        if (minute >= 60) { 
            minute -= 60; 
            hour++; 
        }
        
        if (hour >= 24) { 
            hour -= 24; 
        }
        
        snprintf(buffer, sizeof(buffer), "Time: %02d:%02d:%02d", 
                hour, minute, gps_data.second);
        ssd1306_display_text(buffer, 10, 50, 1);
    } else {
        ssd1306_display_text("Time: N/A", 10, 50, 1);
    }
    
    vTaskDelay(pdMS_TO_TICKS(1000));
}

// UART initialization for GPS
static esp_err_t uart_init(void) {
    uart_config_t uart_config = {
        .baud_rate = GPS_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_APB,
    };
    
    ESP_ERROR_CHECK(uart_param_config(GPS_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_NUM, GPS_UART_TX_PIN, GPS_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_NUM, GPS_UART_BUF_SIZE, 0, 0, NULL, 0));
    
    return ESP_OK;
}

// GPIO initialization
static void gpio_init(void) {
    gpio_config_t io_conf = {};
    
    // Button configuration
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << BUTTON_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
    
    // Output pins configuration
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << BUZZER_PIN) | (1ULL << LED_PIN) | 
                           (1ULL << LED_PINA) | (1ULL << LED_PINB);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    
 // Initialize outputs to LOW
    gpio_set_level(BUZZER_PIN, 0);
    gpio_set_level(LED_PIN, 0);
    gpio_set_level(LED_PINA, 0);
    gpio_set_level(LED_PINB, 0);
}

// Main task for the application
static void vitalsync_task(void *pvParameters) {
    float temperature;
    char gps_buffer[GPS_UART_BUF_SIZE];
    int gps_len;
    
    // Initial LED behavior
    gpio_set_level(LED_PINA, 1);  // Pin 18 LED on initially
    vTaskDelay(pdMS_TO_TICKS(1000));  // Wait for 1 second
    gpio_set_level(LED_PINA, 0);  // Pin 18 LED off after 1 second
    
    gpio_set_level(LED_PINB, 1);  // Pin 32 LED on for the rest of the system runtime
    
    // Show initial messages
    show_message("WELCOME", 2, 20, 25, 2000);
    show_message("VitalSync\n  Hospital", 2, 10, 25, 2000);
    show_message("New\n  Patient!", 2, 45, 20, 2000);
    
    while (1) {
        // Read temperature sensor
        temperature = ds18b20_read_temp();
        ESP_LOGI(TAG, "Temperature: %.1fÂ°C", temperature);
        
        // Read MAX30105 sensor
        max30105_read_sensor();
        
        // If readings are valid, show patient report
        if (validSPO2 && validHeartRate) {
            show_patient_report(temperature, heartRate, spo2);
            ESP_LOGI(TAG, "Heart Rate: %ld BPM, SpO2: %ld%%", heartRate, spo2);
            vTaskDelay(pdMS_TO_TICKS(3000));
        } else {
            // If readings are invalid, show message
            show_message("Calculating\nVitals...", 1, 20, 20, 1000);
            ESP_LOGW(TAG, "Calculating vitals, waiting for valid readings");
        }
        
        // Process GPS data
        gps_len = uart_read_bytes(GPS_UART_NUM, (uint8_t *)gps_buffer, sizeof(gps_buffer) - 1, 100 / portTICK_PERIOD_MS);
        if (gps_len > 0) {
            gps_buffer[gps_len] = 0; // Null-terminate
            parse_gps_data(gps_buffer, gps_len);
            ESP_LOGI(TAG, "GPS Data: %s", gps_buffer);
        }
        
        show_gps_data();
        
        // Check for button press (LOW = pressed with pull-up)
        if (gpio_get_level(BUTTON_PIN) == 0) {
            vTaskDelay(pdMS_TO_TICKS(50)); // debounce
            if (gpio_get_level(BUTTON_PIN) == 0) {
                // Blink and beep 3 times (button press required)
                for (int i = 0; i < 3; i++) {
                    gpio_set_level(LED_PIN, 1);
                    gpio_set_level(BUZZER_PIN, 1);
                    vTaskDelay(pdMS_TO_TICKS(200));
                    gpio_set_level(LED_PIN, 0);
                    gpio_set_level(BUZZER_PIN, 0);
                    vTaskDelay(pdMS_TO_TICKS(200));
                }
                
                // Show "Thank You" message
                show_message("Thank You!", 2, 5, 25, 3000);
                
                // Restart system
                esp_restart();
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100)); // Small delay to prevent watchdog trigger
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "VitalSync starting...");
    
    // Initialize subsystems
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");
    
    ESP_ERROR_CHECK(ssd1306_init());
    ESP_LOGI(TAG, "OLED display initialized successfully");
    
    ESP_ERROR_CHECK(max30105_init());
    ESP_LOGI(TAG, "MAX30105 initialized successfully");
    
    ESP_ERROR_CHECK(uart_init());
    ESP_LOGI(TAG, "UART for GPS initialized successfully");
    
    gpio_init();
    ESP_LOGI(TAG, "GPIOs initialized successfully");
    
    // Create the main task
    xTaskCreate(vitalsync_task, "vitalsync_task", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "VitalSync task created");
}