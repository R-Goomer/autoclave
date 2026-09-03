#include <Wire.h>
#include <Adafruit_BMP085.h>

// --- Sensor Pins ---
const int lm35Pin        = 5;
const int waterLevelPin  = 4;
const int waterThreshold = 1850; // Above 1850 = Water detected

// --- Relay Pins (Top to Bottom) ---
const int RELAY_STEAM_INLET = 6;
const int RELAY_DRAIN       = 42;
const int RELAY_EXHAUST     = 41;
const int RELAY_AIR_VALVE   = 40;
const int RELAY_HEATER      = 39;
const int RELAY_DOOR_LOCK   = 38;

// --- Relay Logic Configuration (Active LOW: LOW = ON, HIGH = OFF) ---
const int RELAY_ON  = HIGH;
const int RELAY_OFF = LOW;

// --- Process Parameters ---
const float TARGET_TEMP_C     = 120.0;
const float TARGET_PSI        = 15.0;
const unsigned long HOLD_TIME = 15UL * 60UL * 1000UL;

// --- Global Safety Limits ---
const float MAX_SAFE_PSI      = 20.0;  // Immediate blowoff if pressure crosses this
const float MAX_SAFE_TEMP_C   = 150.0; // Emergency heater cutoff threshold

// --- State Machine States ---
enum CycleState {
  STATE_IDLE,
  STATE_CHECK_WATER,
  STATE_LOCK_DOOR,
  STATE_PURGE,
  STATE_HEATING,
  STATE_STERILIZING,
  STATE_EXHAUST,
  STATE_COMPLETE,
  STATE_EMERGENCY_SHUTDOWN
};

CycleState currentState = STATE_CHECK_WATER;
unsigned long stateStartTime = 0;

Adafruit_BMP085 bmp;

void setAllRelays(int state) {
  digitalWrite(RELAY_STEAM_INLET, state);
  digitalWrite(RELAY_DRAIN, state);
  digitalWrite(RELAY_EXHAUST, state);
  digitalWrite(RELAY_AIR_VALVE, state);
  digitalWrite(RELAY_HEATER, state);
  digitalWrite(RELAY_DOOR_LOCK, state);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  int relayPins[] = {RELAY_STEAM_INLET, RELAY_DRAIN, RELAY_EXHAUST, 
                     RELAY_AIR_VALVE, RELAY_HEATER, RELAY_DOOR_LOCK};
                     
  for (int pin : relayPins) {
    digitalWrite(pin, RELAY_OFF);
    pinMode(pin, OUTPUT);
  }

  analogSetPinAttenuation(lm35Pin, ADC_11db);
  analogSetPinAttenuation(waterLevelPin, ADC_11db);

  Wire.begin(8, 9);
  if (!bmp.begin()) {
    Serial.println("CRITICAL: BMP180 sensor initialization failed!");
    currentState = STATE_EMERGENCY_SHUTDOWN;
  }

  Serial.println("Autoclave Controller Initialized with Continuous Safety Interlocks.");
}

void loop() {
  // --- Continuous Metric Acquisition ---
  float tempC = analogReadMilliVolts(lm35Pin) / 10.0;
  float pressurePsi = bmp.readPressure() / 6894.76;
  bool hasWater = (analogRead(waterLevelPin) > waterThreshold);

  // =========================================================================
  // GLOBAL SAFETY CHECKS (Executed on every loop cycle, irrespective of state)
  // =========================================================================
  
  // 1. OVER-PRESSURE INTERLOCK
  if (pressurePsi >= MAX_SAFE_PSI) {
    Serial.println("EMERGENCY: OVER-PRESSURE DETECTED! Opening exhaust & killing heaters.");
    digitalWrite(RELAY_HEATER, RELAY_OFF);
    digitalWrite(RELAY_STEAM_INLET, RELAY_OFF);
    digitalWrite(RELAY_EXHAUST, RELAY_ON); // Relieve pressure immediately
    currentState = STATE_EMERGENCY_SHUTDOWN;
  }

  // 2. OVER-TEMPERATURE INTERLOCK
  if (tempC >= MAX_SAFE_TEMP_C) {
    Serial.println("EMERGENCY: OVER-TEMPERATURE DETECTED! Shutting down heat sources.");
    digitalWrite(RELAY_HEATER, RELAY_OFF);
    digitalWrite(RELAY_STEAM_INLET, RELAY_OFF);
    currentState = STATE_EMERGENCY_SHUTDOWN;
  }

  // 3. DRY-RUN PROTECTION (Applies whenever the heater or inlet is energized)
  if (!hasWater && (currentState == STATE_PURGE || currentState == STATE_HEATING || currentState == STATE_STERILIZING)) {
    Serial.println("EMERGENCY: WATER LOSS DETECTED DURING RUN! Immediate cutoff.");
    digitalWrite(RELAY_HEATER, RELAY_OFF);
    digitalWrite(RELAY_STEAM_INLET, RELAY_OFF);
    currentState = STATE_EMERGENCY_SHUTDOWN;
  }

  // =========================================================================
  // NORMAL STATE MACHINE
  // =========================================================================
  switch (currentState) {

    case STATE_CHECK_WATER:
      Serial.println("[STATE] Checking Water Level...");
      if (hasWater) {
        currentState = STATE_LOCK_DOOR;
      } else {
        Serial.println("ALERT: Insufficient water. Awaiting refill...");
      }
      break;

    case STATE_LOCK_DOOR:
      Serial.println("[STATE] Engaging Door Lock...");
      digitalWrite(RELAY_DOOR_LOCK, RELAY_ON);
      delay(2000);
      
      stateStartTime = millis();
      currentState = STATE_PURGE;
      break;

    case STATE_PURGE:
      Serial.println("[STATE] Purging cool air from vessel...");
      digitalWrite(RELAY_EXHAUST, RELAY_ON);
      digitalWrite(RELAY_HEATER, RELAY_ON);

      // Purge for 30s or until steam reaches 90°C
      if (millis() - stateStartTime > 30000 || tempC >= 90.0) {
        digitalWrite(RELAY_EXHAUST, RELAY_OFF);
        currentState = STATE_HEATING;
      }
      break;

    case STATE_HEATING:
      Serial.println("[STATE] Pressurizing and Heating...");
      digitalWrite(RELAY_HEATER, RELAY_ON);
      digitalWrite(RELAY_STEAM_INLET, RELAY_ON);

      // Transition to sterilization once both thresholds are reached
      if (tempC >= TARGET_TEMP_C && pressurePsi >= TARGET_PSI) {
        stateStartTime = millis();
        currentState = STATE_STERILIZING;
      }
      break;

    case STATE_STERILIZING:
      Serial.println("[STATE] Sterilization Hold Running...");

      // Regulate temperature (Hysteresis band: 120°C - 121°C)
      if (tempC < TARGET_TEMP_C) {
        digitalWrite(RELAY_HEATER, RELAY_ON);
      } else {
        digitalWrite(RELAY_HEATER, RELAY_OFF);
      }

      // Pressure regulation: Close steam inlet if pressure is adequate
      if (pressurePsi >= TARGET_PSI) {
        digitalWrite(RELAY_STEAM_INLET, RELAY_OFF);
      } else {
        digitalWrite(RELAY_STEAM_INLET, RELAY_ON);
      }

      // Check hold time
      if (millis() - stateStartTime >= HOLD_TIME) {
        digitalWrite(RELAY_HEATER, RELAY_OFF);
        digitalWrite(RELAY_STEAM_INLET, RELAY_OFF);
        currentState = STATE_EXHAUST;
      }
      break;

    case STATE_EXHAUST:
      Serial.println("[STATE] Depressurizing and Draining...");
      digitalWrite(RELAY_EXHAUST, RELAY_ON);
      digitalWrite(RELAY_DRAIN, RELAY_ON);

      // Safe unlocking pressure limit (<= 1.0 PSI)
      if (pressurePsi <= 1.0) {
        digitalWrite(RELAY_EXHAUST, RELAY_OFF);
        digitalWrite(RELAY_DRAIN, RELAY_OFF);
        currentState = STATE_COMPLETE;
      }
      break;

    case STATE_COMPLETE:
      Serial.println("[SUCCESS] Cycle Completed. Depressurized. Unlocking Door.");
      digitalWrite(RELAY_DOOR_LOCK, RELAY_OFF);
      currentState = STATE_IDLE;
      break;

    case STATE_IDLE:
      setAllRelays(RELAY_OFF);
      break;

    case STATE_EMERGENCY_SHUTDOWN:
      // Heater and steam inlet remain killed; vent chamber safely
      digitalWrite(RELAY_HEATER, RELAY_OFF);
      digitalWrite(RELAY_STEAM_INLET, RELAY_OFF);

      digitalWrite(RELAY_EXHAUST, RELAY_ON);
      digitalWrite(RELAY_DRAIN, RELAY_ON);
      digitalWrite(RELAY_AIR_VALVE, RELAY_ON);

      // Keep door locked until chamber drops below 1.0 PSI
      if (pressurePsi > 1.0) {
        digitalWrite(RELAY_DOOR_LOCK, RELAY_ON);
      } else {
        digitalWrite(RELAY_DOOR_LOCK, RELAY_OFF);
      }

      Serial.println("SYSTEM IN EMERGENCY SHUTDOWN. Manual reset required.");
      break;
  }

  // --- Telemetry Display ---
  Serial.print("T: ");
  Serial.print(tempC, 1);
  Serial.print(" °C | P: ");
  Serial.print(pressurePsi, 2);
  Serial.print(" PSI | Water: ");
  Serial.println(hasWater ? "OK" : "EMPTY");

  delay(500); // 500ms cycle gives rapid feedback to over-pressure scenarios
}
