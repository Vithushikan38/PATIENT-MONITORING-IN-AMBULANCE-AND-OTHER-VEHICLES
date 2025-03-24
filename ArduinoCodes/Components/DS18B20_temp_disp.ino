#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// OLED Display Settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// DS18B20 Sensor Pin
#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

void setup() {
    Serial.begin(115200);

    // Initialize OLED Display
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println("SSD1306 OLED Display not found!");
        while (1);
    }

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 10);
    display.println("Initializing...");
    display.display();
    delay(2000);

    // Initialize DS18B20 Sensor
    sensors.begin();
}

void loop() {
    sensors.requestTemperatures();
    float tempC = sensors.getTempCByIndex(0);

    // Display temperature on OLED
    display.clearDisplay();
    display.setTextSize(1); // Set text size
    display.setCursor(0, 20);
    display.print("Temperature: ");
    display.print(tempC, 1); // Display 1 decimal place

    // Manually draw degree symbol
    display.drawCircle(105, 20, 2, WHITE);  // Small degree symbol

    // Print 'C' for Celsius
    display.setCursor(110, 20);
    display.print(" C");

    display.display();
    delay(1000);
}
