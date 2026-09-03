#include <Wire.h>
#include <Adafruit_BMP085.h>

// --- Pin Definitions ---
const int lm35Pin = 5;       // LM35 Temperature Sensor
const int waterLevelPin = 4; // Water Level Sensor Analog Input

// --- Threshold for Water Detection ---
// 1778 is 0% (dry). We set the threshold slightly above it (e.g., 1850) 
// so anything above dry triggers "ON".
const int waterThreshold = 1850;

// --- Create BMP180 Object ---
Adafruit_BMP085 bmp;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Set ADC attenuation for ESP32-S3 analog pins (0-3.3V range)
  analogSetPinAttenuation(lm35Pin, ADC_11db);
  analogSetPinAttenuation(waterLevelPin, ADC_11db);
  
  // Initialize I2C for BMP180 (ESP32-S3 default SDA = 8, SCL = 9)
  Wire.begin(8, 9); 
  
  // Check BMP180 connection
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP180 sensor, check wiring!");
    while (1) {}
  }
  
  Serial.println("System Initialized Successfully!");
  Serial.println("----------------------------------------");
}

void loop() {
  // --- 1. Read LM35 Temperature (°C) ---
  int mVolt = analogReadMilliVolts(lm35Pin);
  float lm35TempC = mVolt / 10.0; // 10 mV per degree Celsius
  
  // --- 2. Read BMP180 Pressure (PSI) ---
  float pressurePa = bmp.readPressure();
  float pressurePsi = pressurePa / 6894.76; // Convert Pa to PSI

  // --- 3. Read Water Level Sensor (ON/OFF) ---
  int waterLevelValue = analogRead(waterLevelPin);
  String waterStatus = (waterLevelValue > waterThreshold) ? "ON (Water Present)" : "OFF (Dry)";

  // --- 4. Print All Results ---
  Serial.print("LM35 Temp:       ");
  Serial.print(lm35TempC, 2);
  Serial.println(" °C");
  
  Serial.print("BMP180 Pressure: ");
  Serial.print(pressurePsi, 4);
  Serial.println(" PSI");

  Serial.print("Water Status:    ");
  Serial.println(waterStatus);
  
  Serial.println("----------------------------------------");
  delay(2000);
}
