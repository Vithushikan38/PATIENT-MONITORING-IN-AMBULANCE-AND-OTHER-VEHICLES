#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "MAX30100_PulseOximeter.h"

// DS18B20 Temperature Sensor Setup
#define ONE_WIRE_BUS 4  // GPIO4 for DS18B20 data
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// MAX30100 Pulse Oximeter Setup
PulseOximeter pox;
#define REPORTING_PERIOD_MS 1000
uint32_t tsLastReport = 0;

// OLED Display Setup (I2C: SDA=21, SCL=22 for ESP32)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, 22, 21);

// Variables to store patient details
float patientTemp = 0.0;
float heartRate = 0.0;
float spo2 = 0.0;

// Callback function for pulse detection
void onBeatDetected() {
    Serial.println("Beat detected!");
}

void setup() {
    Serial.begin(115200);

    // Initialize sensors
    sensors.begin();  // Temperature sensor
    u8g2.begin();     // OLED display

    // Read temperature once
    sensors.requestTemperatures();
    patientTemp = sensors.getTempCByIndex(0);

    // Initialize MAX30100
    Serial.print("Initializing Pulse Oximeter...");
    if (!pox.begin()) {
        Serial.println("FAILED");
        while (1);  // Stop execution if failed
    } else {
        Serial.println("SUCCESS");
    }
    pox.setOnBeatDetectedCallback(onBeatDetected); // Register callback
}

void loop() {
    // Update Pulse Oximeter
    pox.update();

    // Read heart rate & SpO2 every second
    if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
        heartRate = pox.getHeartRate();
        spo2 = pox.getSpO2();
        tsLastReport = millis();

        // Print to Serial Monitor
        Serial.print("Temperature: ");
        Serial.print(patientTemp);
        Serial.println(" °C");
        Serial.print("Heart rate: ");
        Serial.print(heartRate);
        Serial.println(" bpm");
        Serial.print("SpO2: ");
        Serial.print(spo2);
        Serial.println(" %");

        // Display on OLED
        // Clear the buffer before drawing
        u8g2.clearBuffer();

        // Set a smaller font for the title
        u8g2.setFont(u8g2_font_6x12_tf);
        u8g2.drawStr(35, 8, "VITAL SYNC");  // Centered title (adjust x if needed)

        // Display subtitle "Patient Data"
        u8g2.drawStr(30, 18, "Patient Data");

        // Display Temperature
        char tempStr[16];
        snprintf(tempStr, sizeof(tempStr), "Temp: %.2f °C", (double)patientTemp);
        u8g2.drawStr(10, 30, tempStr);

        // Display Heart Rate
        char hrStr[16];
        snprintf(hrStr, sizeof(hrStr), "HR: %.1f bpm", heartRate);
        u8g2.drawStr(10, 42, hrStr);

        // Display SpO2
        char spo2Str[16];
        snprintf(spo2Str, sizeof(spo2Str), "SpO2: %.1f %%", spo2);
        u8g2.drawStr(10, 54, spo2Str);

        // Update OLED display
        u8g2.sendBuffer();

    }
}
