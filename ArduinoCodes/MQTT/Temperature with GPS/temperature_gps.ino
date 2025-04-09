#include <TinyGPS++.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// GPS Hardware Serial Configuration
#define GPS_RX 16
#define GPS_TX 17
HardwareSerial neogps(2);  // UART2 for GPS

TinyGPSPlus gps;

// DS18B20 Temperature Sensor Configuration
#define ONE_WIRE_BUS 4  // Pin where the DS18B20 data line is connected
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// WiFi Configuration
const char* ssid = "No Internet";        // Wi-Fi SSID
const char* password = "#VithuEng38@"; // Wi-Fi password

// MQTT Configuration
const char* mqttServer = "test.mosquitto.org";  // MQTT broker address
const int mqttPort = 1883;
const char* mqttTopic = "home/data";  // Common topic for both GPS and temperature data

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// Timing Variables
unsigned long lastReconnectAttempt = 0;
const unsigned long reconnectInterval = 10000; // 10 seconds interval to reconnect
unsigned long lastGPSUpdate = 0;
const unsigned long gpsInterval = 1000; // GPS update interval

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for serial to initialize

  // Initialize serial port for GPS
  neogps.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  
  // Initialize DS18B20 sensor
  sensors.begin();

  Serial.println("\nStarting GPS and Temperature Tracker...");
  
  // Connect to Wi-Fi
  connectToWiFi();
  
  // Initialize MQTT
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(mqttCallback);
}

void loop() {
  // Handle Wi-Fi connection
  if (WiFi.status() != WL_CONNECTED) {
    connectToWiFi();
  }

  // Handle MQTT connection
  if (!mqttClient.connected()) {
    unsigned long now = millis();
    if (now - lastReconnectAttempt > reconnectInterval) {
      lastReconnectAttempt = now;
      reconnectMQTT();
    }
  } else {
    mqttClient.loop();
  }

  // Process both GPS data and temperature data, then send together
  sendData();
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
  if (!mqttClient.connected()) {
    Serial.print("Connecting to MQTT...");

    // Create a unique client ID
    String clientId = "ESP32_Sensor_" + String(random(0xffff), HEX);

    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("Connected to MQTT!");
      mqttClient.subscribe(mqttTopic);
    } else {
      Serial.print("Failed (");
      Serial.print(mqttClient.state());
      Serial.println(")");
    }
  }
}

void sendData() {
  // Ensure GPS data is being processed correctly
  while (neogps.available()) {
    gps.encode(neogps.read()); // Continuously process GPS data
  }

  // Read GPS data
  float lat = gps.location.lat();
  float lng = gps.location.lng();
  float alt = gps.altitude.meters();

  // Read temperature data
  sensors.requestTemperatures();  // Request temperature reading
  float tempC = sensors.getTempCByIndex(0);

  // Prepare the data for GPS and temperature
  String payload = "{";

  // Add GPS data (if available)
  if (gps.location.isValid()) {
    payload += "\"lat\": " + String(lat, 6) + ",";
    payload += "\"lng\": " + String(lng, 6) + ",";
    payload += "\"alt\": " + String(alt) + ",";
  } else {
    payload += "\"lat\": null,";
    payload += "\"lng\": null,";
    payload += "\"alt\": null,";
  }

  // Add temperature data
  if (tempC != DEVICE_DISCONNECTED_C) {
    payload += "\"temperature\": " + String(tempC, 2);
  } else {
    payload += "\"temperature\": null";
  }

  // Close JSON object
  payload += "}";

  // Publish the combined data to MQTT
  if (mqttClient.connected()) {
    if (mqttClient.publish(mqttTopic, payload.c_str())) {
      Serial.println("Published Combined Data (GPS + Temperature)");
    } else {
      Serial.println("Failed to publish combined data");
    }
  } else {
    Serial.println("MQTT not connected, failed to send data");
  }

  delay(500);  // Wait for 500 milliseconds before reading again
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("MQTT Message [");
  Serial.print(topic);
  Serial.print("]: ");
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}
