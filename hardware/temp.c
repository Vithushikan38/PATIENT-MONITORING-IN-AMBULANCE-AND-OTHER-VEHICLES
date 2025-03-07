#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "owb.h"
#include "ds18b20.h"
#include "owb_rmt.h"


#define ONE_WIRE_BUS GPIO_NUM_4  // GPIO4

void app_main() {
    // Initialize OneWireBus
    owb_rmt_driver_info rmt_driver = {0};
    OneWireBus *owb = owb_rmt_initialize(&rmt_driver, ONE_WIRE_BUS, RMT_CHANNEL_1, RMT_CHANNEL_0);
    owb_use_crc(owb, true); // Enable CRC check

    // Search for DS18B20 devices
    OneWireBus_ROMCode rom_code;
    bool found_device = false;
    owb_search_first(owb, &rom_code, &found_device);
    
    if (!found_device) {
        printf("No DS18B20 devices found!\n");
        return;
    }

    // Initialize DS18B20 with the found ROM code
    DS18B20_Info *ds18b20 = ds18b20_malloc();
    ds18b20_init(ds18b20, owb, rom_code);
    ds18b20_use_crc(ds18b20, true);
    ds18b20_set_resolution(ds18b20, DS18B20_RESOLUTION_12_BIT);

    while (1) {
        float tempC = 0;
        ds18b20_convert_and_read_temp(ds18b20, &tempC);
        printf("Temperature: %.2f °C\n", tempC);
        vTaskDelay(pdMS_TO_TICKS(1000)); // Read temperature every second
    }
}