#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <PubSubClient.h>

// DS18B20 Temperature Sensor Configuration
#define ONE_WIRE_BUS 4  // Pin where the DS18B20 data line is connected
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// WiFi Configuration
const char* ssid = "No Internet";        // Your Wi-Fi SSID
const char* password = "#VithuEng38@"; // Your Wi-Fi password

// MQTT Configuration
const char* mqttServer = "test.mosquitto.org";  // MQTT broker address (replace with your broker's IP or URL)
const int mqttPort = 1883;                   // MQTT port (default 1883)
const char* mqttTopic = "home/temperature/dallas";  // Topic to publish temperature readings

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

void setup() {
  Serial.begin(115200);
  sensors.begin();  // Initialize the DS18B20 sensor

  // Connect to Wi-Fi
  connectToWiFi();
  
  // Initialize MQTT
  mqttClient.setServer(mqttServer, mqttPort);
}

void loop() {
  // Handle Wi-Fi connection
  if (WiFi.status() != WL_CONNECTED) {
    connectToWiFi();
  }

  // Handle MQTT connection
  if (!mqttClient.connected()) {
    reconnectMQTT();
  } else {
    mqttClient.loop();
  }

  // Read and publish temperature data
  readAndPublishTemperature();
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
    String clientId = "ESP32_Temperature_Sensor-" + String(random(0xffff), HEX);
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("Connected to MQTT");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(mqttClient.state());
      delay(5000); // Wait for 5 seconds before retrying
    }
  }
}

void readAndPublishTemperature() {
  sensors.requestTemperatures();  // Request temperature reading

  // Get temperature in Celsius
  float tempC = sensors.getTempCByIndex(0);  

  // Check if the temperature is valid
  if (tempC != DEVICE_DISCONNECTED_C) {
    Serial.print("Temperature: ");
    Serial.print(tempC);
    Serial.println(" °C");

    // Prepare the temperature payload
    String tempString = String(tempC, 2);  // Format the temperature value to 2 decimal places

    // Publish temperature to the MQTT broker
    if (mqttClient.publish(mqttTopic, tempString.c_str())) {
      Serial.println("Temperature published successfully!");
    } else {
      Serial.println("Failed to publish temperature");
    }
  } else {
    Serial.println("Error: Could not read temperature data");
  }

  delay(500);  // Wait for 500 milli seconds before reading again
}
