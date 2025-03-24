#include <OneWire.h>
#include <DallasTemperature.h>

// Define the pin for DS18B20 data
#define ONE_WIRE_BUS 4  

// Setup OneWire and DallasTemperature
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

void setup() {
    Serial.begin(115200);
    sensors.begin();  
}

void loop() {
    // REQ temperature
    sensors.requestTemperatures();  
    // Get temperature in Celsius
    float tempC = sensors.getTempCByIndex(0);  
    
    Serial.print("Temperature: ");
    Serial.print(tempC);
    Serial.println(" °C");
    
    delay(1000); 
}
