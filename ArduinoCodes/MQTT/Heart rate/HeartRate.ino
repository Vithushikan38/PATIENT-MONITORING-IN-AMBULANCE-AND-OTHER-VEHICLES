#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <WiFi.h>
#include <PubSubClient.h>

// MAX30102 Sensor Configuration
MAX30105 particleSensor;

// WiFi Configuration
const char* ssid = "No Internet";        // Your Wi-Fi SSID
const char* password = "#VithuEng38@";   // Your Wi-Fi password

// MQTT Configuration
const char* mqttServer = "test.mosquitto.org";  // MQTT broker address (replace with your broker's IP or URL)
const int mqttPort = 1883;                     // MQTT port (default 1883)
const char* mqttTopic = "home/MAX30105/heartRate";  // Topic to publish heart rate and SpO2

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

const byte RATE_SIZE = 4;  // Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; // Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; // Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;

long redValue;
long irValue;
float Spo2;  // Changed to float for better precision

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing...");

  // Initialize MAX30102 sensor
  Wire.begin(21, 22);  // SDA (21) and SCL (22) for ESP32
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 not detected. Please check wiring/power.");
    while (1);
  }

  // Allow sensor to stabilize
  Serial.println("Place your finger on the sensor.");
  delay(5000);  // Wait for 5 seconds to stabilize the sensor after setup

  particleSensor.setup();  // Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x1F); // Max Red LED amplitude
  particleSensor.setPulseAmplitudeIR(0x1F);  // Max IR LED amplitude
  particleSensor.setPulseAmplitudeGreen(0);  // Turn off Green LED

  // Connect to Wi-Fi
  connectToWiFi();

  // Initialize MQTT
  mqttClient.setServer(mqttServer, mqttPort);
  while (!mqttClient.connected()) {
    reconnectMQTT();
  }
}

void loop() {
  irValue = particleSensor.getIR();  // Get IR value from sensor
  redValue = particleSensor.getRed();  // Get Red value from sensor

  // Check if there is a valid signal
  if (irValue < 50000 || redValue < 5000) {
    // If no valid signal, publish a message indicating no finger detected
    Serial.println("No finger detected or poor signal.");
    mqttClient.publish(mqttTopic, "No finger detected");
    return;  // Skip the rest of the loop if signal is poor
  }

  if (checkForBeat(irValue) == true) {  // checkForBeat is a function from heartRate.h library
    // We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();
    beatsPerMinute = 60 / (delta / 1000.0);  // Calculate BPM (beats per minute)
    
    if (beatsPerMinute < 255 && beatsPerMinute > 20) {
      rates[rateSpot++] = (byte)beatsPerMinute;  // Store this reading in the array
      rateSpot %= RATE_SIZE;  // Wrap variable
      
      // Take average of readings
      beatAvg = 0;
      for (byte x = 0; x < RATE_SIZE; x++) {
        beatAvg += rates[x];
      }
      beatAvg /= RATE_SIZE;
    }
  }

  // SpO2 Calculation
  // Improved SpO2 calculation (still approximate)
  if (redValue > 0 && irValue > 0) {
    float ratio = (float)redValue / (float)irValue;
    Spo2 = (ratio * 100);  // Using the ratio of red to IR value for a basic estimation
  } else {
    Spo2 = 0;  // If values are not valid, set SpO2 to 0
  }

  // Print the values to the Serial Monitor
  Serial.print("BPM=");
  Serial.print(beatsPerMinute);
  Serial.print(", SpO2=");
  Serial.print(Spo2);
  Serial.println("%");

  // Publish to MQTT
  String payload = "BPM: " + String(beatsPerMinute) + ", SpO2: " + String(Spo2);
  mqttClient.publish(mqttTopic, payload.c_str());

  // Check if the finger is removed and send message to MQTT
  if (irValue < 50000) {
    Serial.println("No finger detected.");
    mqttClient.publish(mqttTopic, "No finger detected");
  }

  mqttClient.loop();  // Maintain MQTT connection
  delay(1000);  // 1-second delay for next reading
}

void connectToWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("Connecting to Wi-Fi...");
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.print(".");
    }

    Serial.println("Connected to Wi-Fi");
  }
}

void reconnectMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Connecting to MQTT...");
    String clientId = "ESP32_HeartRate_Sensor-" + String(random(0xffff), HEX);
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("Connected to MQTT");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(mqttClient.state());
      delay(2000); // Wait for 2 seconds before retrying
    }
  }
}
