#include <stdio.h>
#include <math.h>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_NUM         I2C_NUM_0
#define I2C_MASTER_SDA_IO      21
#define I2C_MASTER_SCL_IO      22
#define I2C_MASTER_FREQ_HZ     100000
#define I2C_TIMEOUT_MS         1000

#define MAX30100_ADDR          0x57
#define REG_MODE_CONFIG        0x06
#define REG_SPO2_CONFIG        0x07
#define REG_LED_CONFIG         0x09
#define REG_FIFO_DATA          0x05

#define SAMPLE_DELAY_MS        200
#define SAMPLE_RATE_HZ         (1000 / SAMPLE_DELAY_MS)
#define BUFFER_SIZE            100         // ~2 seconds
#define UPDATE_INTERVAL        20          // update every 20 samples (~0.4s)

uint16_t ir_buf[BUFFER_SIZE];
uint16_t red_buf[BUFFER_SIZE];
int sample_index = 0;

esp_err_t max30100_write(uint8_t reg, uint8_t val) {
    uint8_t data[2] = {reg, val};
    return i2c_master_write_to_device(I2C_MASTER_NUM, MAX30100_ADDR, data, 2, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

esp_err_t max30100_read(uint8_t reg, uint8_t *buf, size_t len) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, MAX30100_ADDR, &reg, 1, buf, len, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

void i2c_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void max30100_init() {
    max30100_write(REG_MODE_CONFIG, 0x03);  // SpO2 mode
    max30100_write(REG_SPO2_CONFIG, 0x27);  // High-res, 100Hz
    max30100_write(REG_LED_CONFIG, 0x24);   // ~7mA current
}

void max30100_read_fifo(uint16_t *ir, uint16_t *red) {
    uint8_t data[4];
    if (max30100_read(REG_FIFO_DATA, data, 4) == ESP_OK) {
        *ir = (data[0] << 8) | data[1];
        *red = (data[2] << 8) | data[3];
    }
}

void smooth_signal(uint16_t *buf, int size) {
    for (int i = 2; i < size - 2; i++) {
        buf[i] = (buf[i - 2] + 2 * buf[i - 1] + 3 * buf[i] + 2 * buf[i + 1] + buf[i + 2]) / 9;
    }
}

float estimate_heart_rate(uint16_t *ir_buf, int size, float sample_rate_hz) {
    int peak_indices[20] = {0};
    int peak_count = 0;
    float avg = 0;

    for (int i = 0; i < size; i++) avg += ir_buf[i];
    avg /= size;

    float threshold = avg * 1.1f;

    for (int i = 1; i < size - 1; i++) {
        if (ir_buf[i] > threshold &&
            ir_buf[i] > ir_buf[i - 1] &&
            ir_buf[i] > ir_buf[i + 1]) {
            if (peak_count == 0 || (i - peak_indices[peak_count - 1]) > (sample_rate_hz * 0.4f)) {
                peak_indices[peak_count++] = i;
                if (peak_count >= 20) break;
            }
        }
    }

    if (peak_count < 2) return 0;

    float total_interval = 0;
    for (int i = 1; i < peak_count; i++) {
        total_interval += (peak_indices[i] - peak_indices[i - 1]);
    }

    float avg_interval = total_interval / (peak_count - 1);
    return (60.0f * sample_rate_hz) / avg_interval;
}

float estimate_spo2(uint16_t *ir_buf, uint16_t *red_buf, int size) {
    float ir_dc = 0, red_dc = 0;
    float ir_ac = 0, red_ac = 0;

    for (int i = 0; i < size; i++) {
        ir_dc += ir_buf[i];
        red_dc += red_buf[i];
    }
    ir_dc /= size;
    red_dc /= size;

    for (int i = 0; i < size; i++) {
        ir_ac += fabsf(ir_buf[i] - ir_dc);
        red_ac += fabsf(red_buf[i] - red_dc);
    }

    float R = (red_ac / red_dc) / (ir_ac / ir_dc);
    float spo2 = 110.0f - 25.0f * R;

    if (spo2 > 100.0f) spo2 = 100.0f;
    if (spo2 < 0.0f) spo2 = 0.0f;

    return spo2;
}

void shift_buffer(uint16_t *buffer, int size) {
    for (int i = 1; i < size; i++) {
        buffer[i - 1] = buffer[i];
    }
}

void app_main() {
    i2c_init();
    max30100_init();

    int update_counter = 0;

    while (1) {
        uint16_t ir, red;
        max30100_read_fifo(&ir, &red);

        // Shift buffer and append new sample
        shift_buffer(ir_buf, BUFFER_SIZE);
        shift_buffer(red_buf, BUFFER_SIZE);
        ir_buf[BUFFER_SIZE - 1] = ir;
        red_buf[BUFFER_SIZE - 1] = red;

        update_counter++;

        if (update_counter >= UPDATE_INTERVAL) {
            update_counter = 0;

            // Apply smoothing
            smooth_signal(ir_buf, BUFFER_SIZE);

            float hr = estimate_heart_rate(ir_buf, BUFFER_SIZE, SAMPLE_RATE_HZ);
            float spo2 = estimate_spo2(ir_buf, red_buf, BUFFER_SIZE);

            printf("Heart rate: %.2fbpm  SpO2: %.0f%%\n", hr, spo2);
        }

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_DELAY_MS));
    }
}
