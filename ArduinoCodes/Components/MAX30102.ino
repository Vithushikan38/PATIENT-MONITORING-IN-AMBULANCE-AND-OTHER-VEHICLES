#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

MAX30105 particleSensor;

// Variables for storing SpO2 and heart rate readings
uint32_t irBuffer[100]; // infrared LED sensor data
uint32_t redBuffer[100]; // red LED sensor data
int32_t bufferLength = 100; // data length
int32_t spo2; // SPO2 value
int8_t validSPO2; // indicator to show if the SPO2 calculation is valid
int32_t heartRate; // heart rate value
int8_t validHeartRate; // indicator to show if the heart rate calculation is valid

void setup() {
  Serial.begin(115200);
  
  Serial.println("MAX30102 Heart Rate and SpO2 Monitor");

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 was not found. Please check wiring/power.");
    while (1);
  }

  // Configure sensor settings
  byte ledBrightness = 60; // Options: 0=Off to 255=Max
  byte sampleAverage = 4; // Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; // Options: 1=Red only, 2=Red+IR, 3=Red+IR+Green
  int sampleRate = 100; // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; // Options: 69, 118, 215, 411
  int adcRange = 4096; // Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
}

void loop() {
  // Buffer for new readings
  bufferLength = 100;
  
  // Read the first 100 samples for calculating SpO2
  for (byte i = 0; i < bufferLength; i++) {
    while (particleSensor.available() == false) {
      particleSensor.check(); // Check if new data is available
      delay(10);
    }
    
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); // We're done with this sample so move to next sample
  }

  // Calculate heart rate and SpO2 after collecting sufficient data
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  
  // Only print Heart Rate and SpO2 values when they are valid
  if (validSPO2 && validHeartRate) {
    Serial.print("HR=");
    Serial.print(heartRate);
    Serial.print(",SPO2=");
    Serial.println(spo2);
  }
  
  // Small delay before next reading
  delay(1000);
}