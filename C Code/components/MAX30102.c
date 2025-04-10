#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"
#include "math.h"

#define MAX30102_ADDR           0x57 // I2C address of MAX30102

// MAX30102 registers
#define REG_INTR_STATUS_1       0x00
#define REG_INTR_STATUS_2       0x01
#define REG_INTR_ENABLE_1       0x02
#define REG_INTR_ENABLE_2       0x03
#define REG_FIFO_WR_PTR         0x04
#define REG_FIFO_OVF_CNT        0x05
#define REG_FIFO_RD_PTR         0x06
#define REG_FIFO_DATA           0x07
#define REG_FIFO_CONFIG         0x08
#define REG_MODE_CONFIG         0x09
#define REG_SPO2_CONFIG         0x0A
#define REG_LED1_PA             0x0C
#define REG_LED2_PA             0x0D
#define REG_PILOT_PA            0x10
#define REG_MULTI_LED_CTRL1     0x11
#define REG_MULTI_LED_CTRL2     0x12
#define REG_TEMP_INTR           0x1F
#define REG_TEMP_FRAC           0x20
#define REG_TEMP_CONFIG         0x21
#define REG_PROX_INT_THRESH     0x30
#define REG_REV_ID              0xFE
#define REG_PART_ID             0xFF

// I2C configuration
#define I2C_MASTER_SCL_IO       22    // GPIO for I2C SCL
#define I2C_MASTER_SDA_IO       21    // GPIO for I2C SDA
#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_FREQ_HZ      400000
#define I2C_MASTER_TIMEOUT_MS   1000

// Define buffer sizes and settings
#define BUFFER_SIZE             150    // Increased buffer size for better peak detection
#define SAMPLE_RATE             100    // Sample rate in Hz
#define PROCESSING_INTERVAL     1000   // Process data every 1000ms
#define MAX_HISTORY_SIZE        10     // For smoothing heart rate values

// Heart rate and SpO2 history for filtering
typedef struct {
    float values[MAX_HISTORY_SIZE];
    int head;
    int count;
} value_history_t;

static const char *TAG = "MAX30102";

// Circular buffer for storing samples
typedef struct {
    uint32_t red[BUFFER_SIZE];
    uint32_t ir[BUFFER_SIZE];
    int head;
    int count;
} signal_buffer_t;

signal_buffer_t sig_buf = {
    .head = 0,
    .count = 0
};

value_history_t hr_history = {
    .head = 0,
    .count = 0
};

value_history_t spo2_history = {
    .head = 0,
    .count = 0
};

// Function prototypes
esp_err_t i2c_master_init(void);
esp_err_t i2c_write_reg(uint8_t reg_addr, uint8_t data);
esp_err_t i2c_read_reg(uint8_t reg_addr, uint8_t *data);
esp_err_t i2c_read_fifo(uint32_t *red_led, uint32_t *ir_led);
esp_err_t max30102_init(void);
void max30102_reset(void);
void clear_fifo(void);
void add_sample_to_buffer(uint32_t red, uint32_t ir);
void add_value_to_history(value_history_t *history, float value);
float get_median_value(value_history_t *history);
float get_mean_value(value_history_t *history);
void calculate_heart_rate_and_spo2(float *heart_rate, float *spo2);

// Initialize I2C master
esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        return err;
    }
    
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// Write to MAX30102 register
esp_err_t i2c_write_reg(uint8_t reg_addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Read from MAX30102 register
esp_err_t i2c_read_reg(uint8_t reg_addr, uint8_t *data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Read FIFO data for one sample
esp_err_t i2c_read_fifo(uint32_t *red_led, uint32_t *ir_led) {
    uint8_t buffer[6];
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, REG_FIFO_DATA, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MAX30102_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, buffer, 6, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret == ESP_OK) {
        // Convert bytes to values (18-bit resolution)
        *red_led = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];
        *red_led &= 0x3FFFF; // Mask off the first 18 bits
        
        *ir_led = ((uint32_t)buffer[3] << 16) | ((uint32_t)buffer[4] << 8) | buffer[5];
        *ir_led &= 0x3FFFF; // Mask off the first 18 bits
    }
    
    return ret;
}

// Clear the FIFO buffer in the sensor
void clear_fifo(void) {
    i2c_write_reg(REG_FIFO_WR_PTR, 0);
    i2c_write_reg(REG_FIFO_RD_PTR, 0);
    i2c_write_reg(REG_FIFO_OVF_CNT, 0);
}

// Reset MAX30102
void max30102_reset(void) {
    i2c_write_reg(REG_MODE_CONFIG, 0x40); // Reset bit set
    vTaskDelay(100 / portTICK_PERIOD_MS);
}

// Initialize MAX30102
esp_err_t max30102_init(void) {
    esp_err_t ret;
    uint8_t data;
    
    // Verify part ID
    ret = i2c_read_reg(REG_PART_ID, &data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read PART_ID");
        return ret;
    }
    
    if (data != 0x15) { // MAX30102 should return 0x15
        ESP_LOGE(TAG, "Wrong PART_ID: 0x%02x", data);
        return ESP_FAIL;
    }
    
    // Reset the sensor
    max30102_reset();
    
    // Clear FIFO
    clear_fifo();
    
    // Set FIFO configuration
    i2c_write_reg(REG_FIFO_CONFIG, 0x0F); // Sample averaging 1, FIFO rollover, FIFO almost full = 17
    
    // Set mode configuration (SpO2 mode)
    i2c_write_reg(REG_MODE_CONFIG, 0x03); // SpO2 mode
    
    // Set SpO2 configuration
    i2c_write_reg(REG_SPO2_CONFIG, 0x27); // SPO2 ADC range 4096, Sample rate 100 Hz, LED pulse width 411us
    
    // Set LED pulse amplitude - increased for better signals
    i2c_write_reg(REG_LED1_PA, 0x3F); // Red LED current ~12mA
    i2c_write_reg(REG_LED2_PA, 0x3F); // IR LED current ~12mA
    
    // Enable interrupts
    i2c_write_reg(REG_INTR_ENABLE_1, 0xC0); // FIFO almost full & data ready
    
    ESP_LOGI(TAG, "MAX30102 initialized successfully");
    return ESP_OK;
}

// Add a sample to the circular buffer
void add_sample_to_buffer(uint32_t red, uint32_t ir) {
    sig_buf.red[sig_buf.head] = red;
    sig_buf.ir[sig_buf.head] = ir;
    
    sig_buf.head = (sig_buf.head + 1) % BUFFER_SIZE;
    
    if (sig_buf.count < BUFFER_SIZE) {
        sig_buf.count++;
    }
}

// Add a value to history circular buffer for smoothing
void add_value_to_history(value_history_t *history, float value) {
    // Only add non-zero values to heart rate history
    if (value > 0) {
        history->values[history->head] = value;
        history->head = (history->head + 1) % MAX_HISTORY_SIZE;
        
        if (history->count < MAX_HISTORY_SIZE) {
            history->count++;
        }
    }
}

// Calculate median value from history
float get_median_value(value_history_t *history) {
    if (history->count == 0) {
        return 0;
    }
    
    // Copy values to temp array
    float temp[MAX_HISTORY_SIZE];
    int valid_count = 0;
    
    for (int i = 0; i < history->count; i++) {
        int idx = (history->head - 1 - i + MAX_HISTORY_SIZE) % MAX_HISTORY_SIZE;
        if (history->values[idx] > 0) {
            temp[valid_count++] = history->values[idx];
        }
    }
    
    if (valid_count == 0) {
        return 0;
    }
    
    // Simple bubble sort
    for (int i = 0; i < valid_count - 1; i++) {
        for (int j = 0; j < valid_count - i - 1; j++) {
            if (temp[j] > temp[j + 1]) {
                float swap = temp[j];
                temp[j] = temp[j + 1];
                temp[j + 1] = swap;
            }
        }
    }
    
    // Return median
    if (valid_count % 2 == 0) {
        return (temp[valid_count/2 - 1] + temp[valid_count/2]) / 2.0f;
    } else {
        return temp[valid_count/2];
    }
}

// Calculate mean value from history (for SpO2)
float get_mean_value(value_history_t *history) {
    if (history->count == 0) {
        return 0;
    }
    
    float sum = 0;
    int valid_count = 0;
    
    for (int i = 0; i < history->count; i++) {
        int idx = (history->head - 1 - i + MAX_HISTORY_SIZE) % MAX_HISTORY_SIZE;
        if (history->values[idx] > 0) {
            sum += history->values[idx];
            valid_count++;
        }
    }
    
    if (valid_count == 0) {
        return 0;
    }
    
    return sum / (float)valid_count;
}

// Calculate heart rate and SpO2 from the buffered data
void calculate_heart_rate_and_spo2(float *heart_rate, float *spo2) {
    if (sig_buf.count < BUFFER_SIZE) {
        *heart_rate = 0;
        *spo2 = 0;
        return;
    }
    
    // Apply a moving average filter to raw signals to reduce noise
    float filtered_ir[BUFFER_SIZE];
    float filtered_red[BUFFER_SIZE];
    
    // Calculate DC (average) components for both signals
    float ir_mean = 0;
    float red_mean = 0;
    
    for (int i = 0; i < BUFFER_SIZE; i++) {
        int idx = (sig_buf.head - 1 - i + BUFFER_SIZE) % BUFFER_SIZE;
        ir_mean += sig_buf.ir[idx];
        red_mean += sig_buf.red[idx];
    }
    ir_mean /= BUFFER_SIZE;
    red_mean /= BUFFER_SIZE;
    
    // Apply moving average filter and normalize
    const int ma_size = 5; // Moving average window size
    
    for (int i = 0; i < BUFFER_SIZE; i++) {
        float ir_sum = 0;
        float red_sum = 0;
        int count = 0;
        
        // Calculate moving average
        for (int j = -ma_size/2; j <= ma_size/2; j++) {
            int index = (sig_buf.head - 1 - i + j + BUFFER_SIZE) % BUFFER_SIZE;
            ir_sum += sig_buf.ir[index];
            red_sum += sig_buf.red[index];
            count++;
        }
        
        filtered_ir[i] = (ir_sum / count) - ir_mean;  // AC component with DC removed
        filtered_red[i] = (red_sum / count) - red_mean;  // AC component with DC removed
    }
    
    // Find peaks for heart rate calculation - improved algorithm
    int peaks[50];  // Store peak positions
    int peak_count = 0;
    bool lookForMax = true;
    float lastMax = -1000000;
    float lastMin = 1000000;
    float threshold = 0;
    
    // Calculate dynamic threshold as 30% of the signal range
    float max_val = -1000000;
    float min_val = 1000000;
    
    for (int i = 0; i < BUFFER_SIZE; i++) {
        if (filtered_ir[i] > max_val) max_val = filtered_ir[i];
        if (filtered_ir[i] < min_val) min_val = filtered_ir[i];
    }
    threshold = (max_val - min_val) * 0.25; // 25% of peak-to-peak
    
    // Improved peak detection
    for (int i = 1; i < BUFFER_SIZE - 1; i++) {
        // Find local maxima
        if (lookForMax) {
            // Check if this point is higher than the previous
            if (filtered_ir[i] > filtered_ir[i-1]) {
                // Possible peak, continue
                lastMax = filtered_ir[i];
            } else {
                // We've passed a peak, check if it's significant
                if (lastMax - lastMin > threshold) {
                    // This was a significant peak
                    if (peak_count < 50) {
                        peaks[peak_count++] = i - 1; // Record the peak position
                    }
                }
                lookForMax = false;
            }
        } else {
            // Looking for minima
            if (filtered_ir[i] < filtered_ir[i-1]) {
                // Possible minimum, continue
                lastMin = filtered_ir[i];
            } else {
                // Passed a minimum, switch back to looking for max
                lookForMax = true;
            }
        }
    }
    
    // Calculate heart rate from peak intervals
    if (peak_count >= 2) {
        float total_interval = 0;
        for (int i = 1; i < peak_count; i++) {
            total_interval += peaks[i] - peaks[i-1];
        }
        
        float avg_interval = total_interval / (peak_count - 1);
        float beats_per_sample = BUFFER_SIZE / avg_interval;
        float raw_heart_rate = beats_per_sample * (60.0f * SAMPLE_RATE / BUFFER_SIZE);
        
        // Add to history for smoothing
        add_value_to_history(&hr_history, raw_heart_rate);
        
        // Get median for stable heart rate
        float filtered_hr = get_median_value(&hr_history);
        
        // Apply realistic heart rate range
        if (filtered_hr >= 40.0f && filtered_hr <= 200.0f) {
            *heart_rate = filtered_hr;
        } else {
            *heart_rate = 0;
        }
    } else {
        // Not enough peaks detected - try to use last values
        *heart_rate = get_median_value(&hr_history);
    }
    
    // Calculate SpO2
    float red_rms = 0;
    float ir_rms = 0;
    
    for (int i = 0; i < BUFFER_SIZE; i++) {
        red_rms += filtered_red[i] * filtered_red[i];
        ir_rms += filtered_ir[i] * filtered_ir[i];
    }
    
    red_rms = sqrtf(red_rms / BUFFER_SIZE);
    ir_rms = sqrtf(ir_rms / BUFFER_SIZE);
    
    if (ir_rms > 0 && red_mean > 0 && ir_mean > 0) {
        // R = (AC_red/DC_red)/(AC_ir/DC_ir)
        float R = (red_rms / red_mean) / (ir_rms / ir_mean);
        
        // Empirical formula for SpO2
        float raw_spo2 = 110.0f - 25.0f * R;
        
        // Add to history for smoothing
        add_value_to_history(&spo2_history, raw_spo2);
        
        // Get mean for stable SpO2
        float filtered_spo2 = get_mean_value(&spo2_history);
        
        // Clamp to valid range (0-100%)
        if (filtered_spo2 > 100.0f) filtered_spo2 = 100.0f;
        if (filtered_spo2 < 0.0f) filtered_spo2 = 0.0f;
        
        *spo2 = filtered_spo2;
    } else {
        // Get last valid reading
        *spo2 = get_mean_value(&spo2_history);
    }
}

// MAX30102 data collection task
void max30102_task(void *pvParameters) {
    // Initialize I2C
    ESP_ERROR_CHECK(i2c_master_init());
    
    // Initialize MAX30102
    ESP_ERROR_CHECK(max30102_init());
    
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait for sensor to stabilize
    
    uint32_t last_processing_time = 0;
    uint32_t current_time;
    float heart_rate = 0;
    float spo2 = 0;
    
    // Clear buffers
    sig_buf.count = 0;
    sig_buf.head = 0;
    hr_history.count = 0;
    hr_history.head = 0;
    spo2_history.count = 0;
    spo2_history.head = 0;
    
    while (1) {
        uint32_t red_value, ir_value;
        
        // Read data from sensor
        if (i2c_read_fifo(&red_value, &ir_value) == ESP_OK) {
            // Ignore samples that are too small (finger not present)
            if (ir_value > 5000) {  // Threshold for finger detection
                add_sample_to_buffer(red_value, ir_value);
            }
            
            // Get current time
            current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
            
            // Process data at regular intervals
            if (current_time - last_processing_time >= PROCESSING_INTERVAL) {
                calculate_heart_rate_and_spo2(&heart_rate, &spo2);
                
                // Output ONLY heart rate and SpO2 values
                printf("Heart Rate: %.0f, SpO2: %.0f\n", heart_rate, spo2);
                
                last_processing_time = current_time;
            }
        }
        
        vTaskDelay(10 / portTICK_PERIOD_MS); // Sample every 10ms
    }
}

// Test function with simulated data - useful for testing without sensor
void raw_data_task(void *pvParameters) {
    float heart_rate = 0;
    float spo2 = 0;
    uint32_t last_update_time = 0;
    uint32_t current_time;
    
    // Initialize history buffers
    hr_history.count = 0;
    hr_history.head = 0;
    spo2_history.count = 0;
    spo2_history.head = 0;
    
    // Create realistic simulated heartbeat pattern
    const int pattern_length = 100;
    uint32_t red_pattern[pattern_length];
    uint32_t ir_pattern[pattern_length];
    
    // Initialize with baseline
    uint32_t red_base = 7631368;
    uint32_t ir_base = 8392788;
    
    // Create heartbeat pattern (simple sine wave for demo)
    for (int i = 0; i < pattern_length; i++) {
        float angle = (float)i / pattern_length * 2.0f * 3.14159f;
        float sine_val = sinf(angle);
        
        // Add sine wave to baseline with some variability
        red_pattern[i] = red_base + (uint32_t)(sine_val * 100000.0f);
        ir_pattern[i] = ir_base + (uint32_t)(sine_val * 150000.0f);
    }
    
    int pattern_idx = 0;
    
    while (1) {
        // Add simulated sample to buffer
        add_sample_to_buffer(red_pattern[pattern_idx], ir_pattern[pattern_idx]);
        pattern_idx = (pattern_idx + 1) % pattern_length;
        
        current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        // Process and output at regular intervals
        if (current_time - last_update_time >= PROCESSING_INTERVAL) {
            calculate_heart_rate_and_spo2(&heart_rate, &spo2);
            
            // Output ONLY heart rate and SpO2 values
            printf("Heart Rate: %.0f, SpO2: %.0f\n", heart_rate, spo2);
            
            last_update_time = current_time;
        }
        
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void app_main(void) {
    // Uncomment one of these based on your needs:
    
    // Normal operation - reads from actual sensor:
    xTaskCreate(&max30102_task, "max30102_task", 4096, NULL, 5, NULL);
    
    // Test mode - uses simulated heartbeat pattern:
    // xTaskCreate(&raw_data_task, "raw_data_task", 4096, NULL, 5, NULL);
}