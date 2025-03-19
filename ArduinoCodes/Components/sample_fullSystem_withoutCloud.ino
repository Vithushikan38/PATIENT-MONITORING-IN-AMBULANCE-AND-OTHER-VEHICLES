#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <MAX30100_PulseOximeter.h>
#include <TinyGPS++.h>

// OLED Display Settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// DS18B20 Temperature Sensor
#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// MAX30100 Pulse Oximeter
PulseOximeter pox;

// GPS Settings
#define RXPin 16  
#define TXPin 17  
#define GPSBaud 9600
HardwareSerial gpsSerial(2);  
TinyGPSPlus gps;

// Function to show messages on OLED
void showMessage(const char* message, int size, int x, int y, int delayTime) {
    display.clearDisplay();
    display.setTextSize(size);
    display.setCursor(x, y);
    display.println(message);
    display.display();
    delay(delayTime);
}

// Function to display patient report
void showPatientReport(float tempC, float bpm, float spo2) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(10, 5);
    display.println("Patient Report:");
    
    display.setCursor(10, 20);
    display.print("Temp: ");
    display.print(tempC, 1);
    display.write(248);
    display.println("C");

    display.setCursor(10, 35);
    display.print("HR: ");
    display.print(bpm, 0);
    display.println(" BPM");

    display.setCursor(10, 50);
    display.print("SpO2: ");
    display.print(spo2, 0);
    display.println(" %");

    display.display();
    delay(3000);
}

// Function to display GPS details
void showGPSData() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(10, 5);
    display.println("GPS Details:");

    if (gps.location.isValid()) {
        display.setCursor(10, 20);
        display.print("Lat: ");
        display.println(gps.location.lat(), 6);

        display.setCursor(10, 30);
        display.print("Lon: ");
        display.println(gps.location.lng(), 6);
    } else {
        display.setCursor(10, 20);
        display.println("GPS: No Fix");
    }

    display.setCursor(10, 40);
    display.print("Date: ");
    if (gps.date.isValid()) {
        display.print(gps.date.day());
        display.print("/");
        display.print(gps.date.month());
        display.print("/");
        display.println(gps.date.year());
    } else {
        display.println("N/A");
    }

    display.setCursor(10, 50);
    display.print("Time: ");
    if (gps.time.isValid()) {
        int hour = gps.time.hour() + 5;  
        int minute = gps.time.minute() + 30;  
        if (minute >= 60) { minute -= 60; hour++; }
        if (hour >= 24) { hour -= 24; }

        if (hour < 10) display.print("0");
        display.print(hour);
        display.print(":");
        if (minute < 10) display.print("0");
        display.print(minute);
        display.print(":");
        if (gps.time.second() < 10) display.print("0");
        display.println(gps.time.second());
    } else {
        display.println("N/A");
    }

    display.display();
    delay(3000);
}

void setup() {
    gpsSerial.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);

    // Initialize OLED Display
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        while (1);  
    }

    display.clearDisplay();
    display.display();
    delay(1000);

    sensors.begin();  
    pox.begin();  

    showMessage("WELCOME", 2, 20, 25, 2000);

    showMessage("VitalSync Hospital", 1, 10, 25, 2000);

}

void loop() {
    
    showMessage("New\nPatient!", 2, 20, 20, 2000);

    sensors.requestTemperatures();
    float tempC = sensors.getTempCByIndex(0);
    pox.update();
    float bpm = pox.getHeartRate();
    float spo2 = pox.getSpO2();

    showPatientReport(tempC, bpm, spo2);

    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }
    showGPSData();

   
    showMessage("Thank You!", 2, 20, 25, 3000);
}
