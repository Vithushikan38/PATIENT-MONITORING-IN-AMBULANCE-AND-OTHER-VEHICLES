#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MAX30100_PulseOximeter.h>

// OLED Display Settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// MAX30100 Sensor
PulseOximeter pox;
#define REPORTING_INTERVAL_MS 1000
uint32_t lastReportTime = 0;

void onBeatDetected() {
  Serial.println("♥ Heartbeat detected!");
}

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

  // Initialize MAX30100 Sensor
  if (!pox.begin()) {
    Serial.println("Failed to initialize MAX30100!");
    display.clearDisplay();
    display.setCursor(0, 20);
    display.println("MAX30100 Error!");
    display.display();
    while (1);
  } else {
    Serial.println("MAX30100 Initialized!");
    pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA); // Set LED power
  }

  pox.setOnBeatDetectedCallback(onBeatDetected);
}

void loop() {
  pox.update();

  if (millis() - lastReportTime > REPORTING_INTERVAL_MS) {
    lastReportTime = millis();

    float bpm = pox.getHeartRate();
    float spo2 = pox.getSpO2();

    Serial.print("Heart Rate: ");
    Serial.print(bpm);
    Serial.print(" BPM | SpO2: ");
    Serial.print(spo2);
    Serial.println(" %");

    // Display data on OLED
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(10, 10);
    display.print("HR: ");
    display.print(bpm, 0);
    display.println(" BPM");

    display.setCursor(10, 20);
    display.print("SpO2: ");
    display.print(spo2, 0);
    display.println(" %");

    display.display();
  }

  delay(100);
}
