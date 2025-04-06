#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"

// Define GPIO pin for the DS18B20
#define DS18B20_GPIO_PIN  4

// 1-Wire commands
#define OW_CMD_SKIP_ROM      0xCC  // Skip ROM command (for single sensor)
#define OW_CMD_CONVERT_T     0x44  // Start temperature conversion
#define OW_CMD_READ_SCRATCHPAD 0xBE  // Read Scratchpad command

// Function prototypes
void ow_init(gpio_num_t pin);
uint8_t ow_reset(gpio_num_t pin);
void ow_write_bit(gpio_num_t pin, uint8_t bit);
uint8_t ow_read_bit(gpio_num_t pin);
void ow_write_byte(gpio_num_t pin, uint8_t byte);
uint8_t ow_read_byte(gpio_num_t pin);
float read_temperature(gpio_num_t pin);

// Function to initialize 1-Wire communication
void ow_init(gpio_num_t pin) {
    gpio_reset_pin(pin);
    gpio_set_direction(pin, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY);
}

// Function to reset 1-Wire bus
uint8_t ow_reset(gpio_num_t pin) {
    gpio_set_level(pin, 0);
    esp_rom_delay_us(480);  // Pull low for 480us
    gpio_set_level(pin, 1);
    esp_rom_delay_us(70);   // Release and wait for response (wait 70us)
    uint8_t presence = gpio_get_level(pin);  // Read presence pulse (if 1-Wire sensor is present)
    esp_rom_delay_us(410);  // Wait for the rest of the 1-Wire reset time
    return (presence == 0);
}

// Function to write a bit
void ow_write_bit(gpio_num_t pin, uint8_t bit) {
    gpio_set_level(pin, 0);
    esp_rom_delay_us(bit ? 6 : 60);
    gpio_set_level(pin, 1);
    esp_rom_delay_us(bit ? 60 : 10);
}

// Function to read a bit
uint8_t ow_read_bit(gpio_num_t pin) {
    uint8_t bit;
    gpio_set_level(pin, 0);
    esp_rom_delay_us(2);
    gpio_set_level(pin, 1);
    esp_rom_delay_us(10);
    bit = gpio_get_level(pin);
    esp_rom_delay_us(50);
    return bit;
}

// Function to write a byte
void ow_write_byte(gpio_num_t pin, uint8_t byte) {
    for (int i = 0; i < 8; i++) {
        ow_write_bit(pin, (byte >> i) & 0x01);
    }
}

// Function to read a byte
uint8_t ow_read_byte(gpio_num_t pin) {
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        byte |= (ow_read_bit(pin) << i);
    }
    return byte;
}

// Function to read the temperature from DS18B20
float read_temperature(gpio_num_t pin) {
    uint8_t scratchpad[9];

    if (ow_reset(pin) == 0) {
        ESP_LOGE("DS18B20", "Sensor not detected!");
        return -1000; // Return error value
    }

    ow_write_byte(pin, OW_CMD_SKIP_ROM);   // Skip ROM command
    ow_write_byte(pin, OW_CMD_CONVERT_T);  // Start temperature conversion
    vTaskDelay(pdMS_TO_TICKS(750));        // Wait for conversion (750ms)

    ow_reset(pin);
    ow_write_byte(pin, OW_CMD_SKIP_ROM);
    ow_write_byte(pin, OW_CMD_READ_SCRATCHPAD);  // Read Scratchpad command

    for (int i = 0; i < 9; i++) {
        scratchpad[i] = ow_read_byte(pin);
    }

    int16_t raw_temp = (scratchpad[1] << 8) | scratchpad[0];
    return (float)raw_temp / 16.0;
}

// Main task to run the sensor
void app_main() {
    ow_init(DS18B20_GPIO_PIN);

    while (1) {
        float temperature = read_temperature(DS18B20_GPIO_PIN);
        if (temperature == -1000) {
            printf("Failed to read temperature!\n");
        } else {
            printf("Temperature: %.2f*C\n", temperature);
        }
        vTaskDelay(pdMS_TO_TICKS(2000));  // Delay for 2 seconds
}
}
