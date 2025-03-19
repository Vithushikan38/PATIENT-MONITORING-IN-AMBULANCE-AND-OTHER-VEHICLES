#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1 
#define SCREEN_ADDRESS 0x3C  // Default I2C address

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
    Wire.begin();
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;);
    }
    display.clearDisplay();
    showWelcome();
    delay(2000);
    showNewHospital("VitalSync Hospital");
    delay(2000);
}

void loop() {
    // Example usage
    showWaiting();
    delay(2000);
    showStart();
    delay(2000);
    showParameters(36.5, 75, 98);
    delay(2000);
    showStop();
    delay(2000);
    showEnd();
    delay(2000);
}

void showWelcome() {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(20, 25);
    display.println("WELCOME");
    display.display();
}

void showWaiting() {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(10, 25);
    display.println("Waiting..");
    display.display();
}

void showStart() {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(20, 20);
    display.println("New");
    display.setCursor(10, 40);
    display.println("Patient!");
    display.display();
}

void showParameters(float temp, int hrate, int spO2) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(20, 20);
    display.printf("Patient Report:");
    display.setCursor(10, 30);
    display.printf("Temp: %.1f C", temp);
    display.setCursor(10, 40);
    display.printf("Heart Rate: %d BPM", hrate);
    display.setCursor(10, 50);
    display.printf("SpO2: %d%%", spO2);
    display.display();
}

void showNewHospital(const char* new_hospital) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(10, 25);
    display.println(new_hospital);
    display.display();
}

void showStop() {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(20, 25);
    display.println("END RIDE");
    display.display();
}

void showEnd() {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(20, 25);
    display.println("GOOD BYE");
    display.display();
}
