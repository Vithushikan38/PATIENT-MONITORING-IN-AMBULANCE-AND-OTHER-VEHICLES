#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPS++.h>

// OLED Display Settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// GPS Settings
#define RXPin 16  // GPS TX → ESP32 RX
#define TXPin 17  // GPS RX → ESP32 TX
#define GPSBaud 9600

HardwareSerial gpsSerial(2);  // Use UART2 for ESP32
TinyGPSPlus gps;

void setup() {
  Serial.begin(115200);
  
  // Start GPS serial
  gpsSerial.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);

  // Initialize OLED Display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 OLED Display not found!");
    while (1);
  }
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(10, 20);
  display.println("GPS Tracker");
  display.display();
  delay(2000);
}

void displayGPSData() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(5, 10);

  if (gps.location.isValid()) {
    display.print("Lat: ");
    display.println(gps.location.lat(), 6);
    display.print("Lon: ");
    display.println(gps.location.lng(), 6);
  } else {
    display.println("GPS Location: N/A");
  }

  display.print("Date: ");
  if (gps.date.isValid()) {
    display.print(gps.date.day());
    display.print("/");
    display.print(gps.date.month());
    display.print("/");
    display.println(gps.date.year());
  } else {
    display.println("Date: N/A");
  }

  display.print("Time: ");
  if (gps.time.isValid()) {
    int hour = gps.time.hour(); // Get UTC hour
    int minute = gps.time.minute();
    int second = gps.time.second();

    // Convert UTC to Local Time (Adjust for your timezone)
    int timeZoneOffset = 5; // Hours (For Sri Lanka, it's UTC+5:30)
    int minuteOffset = 30;  // Additional minutes

    hour += timeZoneOffset;
    minute += minuteOffset;

    // Handle overflow
    if (minute >= 60) {
      minute -= 60;
      hour++;
    }
    if (hour >= 24) {
      hour -= 24; // Adjust for next day
    }

    if (hour < 10) display.print("0");
    display.print(hour);
    display.print(":");
    if (minute < 10) display.print("0");
    display.print(minute);
    display.print(":");
    if (second < 10) display.print("0");
    display.println(second);
  } else {
    display.println("Time: N/A");
  }

  display.display();
}


void loop() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  displayGPSData();
  delay(1000);
}
