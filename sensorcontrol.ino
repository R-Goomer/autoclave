#include <Adafruit_BMP085.h>
#include <Wire.h>

// --- Input Pins ---
const int btnStartPin = 14;
const int btnResetPin = 13;

// --- Sensor Pins ---
const int lm35Pin = 5;
const int waterLevelPin = 4;
const int waterThreshold = 1850; // Above 1850 = Water detected

// --- Relay Pins (Top to Bottom) ---
const int RELAY_STEAM_INLET = 6;
const int RELAY_DRAIN = 42;
const int RELAY_EXHAUST = 41;
const int RELAY_AIR_VALVE = 40;
const int RELAY_HEATER = 39;
const int RELAY_DOOR_LOCK = 38;

// --- Relay Logic Configuration
const int RELAY_ON = HIGH;
const int RELAY_OFF = LOW;

// --- Process Parameters ---
const float TARGET_TEMP_C = 120.0;
const float TARGET_PSI = 15.0;
const unsigned long PURGE_TIME = 120000; // 2 min
const unsigned long HOLD_TIME = 15UL * 60UL * 1000UL;

// --- Global Safety Limits ---
const float MAX_SAFE_PSI = 20.0; // Immediate blowoff if pressure crosses this
const float MAX_SAFE_TEMP_C = 150.0; // Emergency heater cutoff threshold

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

CycleState currentState = STATE_IDLE;
unsigned long stateStartTime = 0;
float baselinePressurePa = 101325.0; // Ambient atmospheric baseline

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

  int relayPins[] = {RELAY_STEAM_INLET, RELAY_DRAIN,  RELAY_EXHAUST,
                     RELAY_AIR_VALVE,   RELAY_HEATER, RELAY_DOOR_LOCK};

  for (int pin : relayPins) {
    digitalWrite(pin, RELAY_OFF);
    pinMode(pin, OUTPUT);
  }

  pinMode(btnStartPin, INPUT_PULLUP);
  pinMode(btnResetPin, INPUT_PULLUP);

  analogSetPinAttenuation(lm35Pin, ADC_11db);
  analogSetPinAttenuation(waterLevelPin, ADC_11db);

  Wire.begin(8, 9);
  if (!bmp.begin()) {
    Serial.println("CRITICAL: BMP180 sensor initialization failed!");
    currentState = STATE_EMERGENCY_SHUTDOWN;
  }

  Serial.println(
      "Autoclave Controller Initialized with Continuous Safety Interlocks.");
}

void loop() {
  // --- Continuous Metric Acquisition ---
  float tempC = analogReadMilliVolts(lm35Pin) / 10.0;
  float currentPressurePa = bmp.readPressure();
  float pressurePsi = (currentPressurePa - baselinePressurePa) / 6894.76;
  if (pressurePsi < 0.0)
    pressurePsi = 0.0; // Clamped against minor ambient fluctuations
  bool hasWater = (analogRead(waterLevelPin) > waterThreshold);

  // =========================================================================
  // GLOBAL SAFETY CHECKS (Executed on every loop cycle, irrespective of state)
  // =========================================================================

  // 1. OVER-PRESSURE INTERLOCK
  if (pressurePsi >= MAX_SAFE_PSI) {
    Serial.println("EMERGENCY: OVER-PRESSURE DETECTED! Opening exhaust & "
                   "killing heaters.");
    digitalWrite(RELAY_HEATER, RELAY_OFF);
    digitalWrite(RELAY_STEAM_INLET, RELAY_OFF);
    digitalWrite(RELAY_EXHAUST, RELAY_ON); // Relieve pressure immediately
    currentState = STATE_EMERGENCY_SHUTDOWN;
  }

  // 2. OVER-TEMPERATURE INTERLOCK
  if (tempC >= MAX_SAFE_TEMP_C) {
    Serial.println(
        "EMERGENCY: OVER-TEMPERATURE DETECTED! Shutting down heat sources.");
    digitalWrite(RELAY_HEATER, RELAY_OFF);
    digitalWrite(RELAY_STEAM_INLET, RELAY_OFF);
    currentState = STATE_EMERGENCY_SHUTDOWN;
  }

  // 3. DRY-RUN PROTECTION (Applies whenever the heater or inlet is energized)
  if (!hasWater &&
      (currentState == STATE_PURGE || currentState == STATE_HEATING ||
       currentState == STATE_STERILIZING)) {
    Serial.println(
        "EMERGENCY: WATER LOSS DETECTED DURING RUN! Immediate cutoff.");
    digitalWrite(RELAY_HEATER, RELAY_OFF);
    digitalWrite(RELAY_STEAM_INLET, RELAY_OFF);
    currentState = STATE_EMERGENCY_SHUTDOWN;
  }

  // =========================================================================
  // NORMAL STATE MACHINE
  // =========================================================================
  static CycleState lastState = (CycleState)-1;
  if (currentState != lastState) {
    Serial.print("--- ENTERING STATE: ");
    switch (currentState) {
    case STATE_IDLE:
      Serial.println("IDLE");
      break;
    case STATE_CHECK_WATER:
      Serial.println("CHECK_WATER");
      break;
    case STATE_LOCK_DOOR:
      Serial.println("LOCK_DOOR");
      break;
    case STATE_PURGE:
      Serial.println("PURGE");
      break;
    case STATE_HEATING:
      Serial.println("HEATING");
      break;
    case STATE_STERILIZING:
      Serial.println("STERILIZING");
      break;
    case STATE_EXHAUST:
      Serial.println("EXHAUST");
      break;
    case STATE_COMPLETE:
      Serial.println("COMPLETE");
      break;
    case STATE_EMERGENCY_SHUTDOWN:
      Serial.println("EMERGENCY_SHUTDOWN");
      break;
    }
    lastState = currentState;
  }

  switch (currentState) {

  case STATE_CHECK_WATER:
    if (hasWater) {
      digitalWrite(RELAY_DOOR_LOCK, RELAY_ON);
      stateStartTime = millis();
      currentState = STATE_LOCK_DOOR;
    } else {
      static unsigned long lastWaterAlert = 0;
      if (millis() - lastWaterAlert > 2000) {
        Serial.println("ALERT: Insufficient water. Awaiting refill...");
        lastWaterAlert = millis();
      }
    }
    break;

  case STATE_LOCK_DOOR:
    // Non-blocking wait for 2 seconds to ensure door is securely locked
    if (millis() - stateStartTime >= 2000) {
      stateStartTime = millis();
      currentState = STATE_PURGE;
    }
    break;

  case STATE_PURGE:
    digitalWrite(RELAY_EXHAUST, RELAY_ON);
    digitalWrite(RELAY_HEATER, RELAY_ON);

    // Purge for 120s or until steam reaches 90°C
    if (millis() - stateStartTime > PURGE_TIME || tempC >= 90.0) {
      digitalWrite(RELAY_EXHAUST, RELAY_OFF);
      currentState = STATE_HEATING;
    }
    break;

  case STATE_HEATING:
    digitalWrite(RELAY_HEATER, RELAY_ON);
    digitalWrite(RELAY_STEAM_INLET, RELAY_ON);

    // Transition to sterilization once both thresholds are reached
    if (tempC >= TARGET_TEMP_C && pressurePsi >= TARGET_PSI) {
      stateStartTime = millis();
      currentState = STATE_STERILIZING;
    }
    break;

  case STATE_STERILIZING:
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
    if (digitalRead(btnStartPin) == LOW) {
      Serial.println("Start button pressed, reading sensor data...");

      long sum = 0;
      for (int i = 0; i < 20; i++) {
        sum += bmp.readPressure();
        delay(100); // 20 * 100ms = 2 seconds
      }
      baselinePressurePa = sum / 20.0;

      Serial.print("Atmospheric baseline set: ");
      Serial.print(baselinePressurePa / 6894.76, 2);
      Serial.println(" psia (0.00 psig)");

      Serial.println("Initiating cycle...");
      currentState = STATE_CHECK_WATER;
    }
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

    {
      static unsigned long lastEmergencyAlert = 0;
      if (millis() - lastEmergencyAlert > 2000) {
        Serial.println("SYSTEM IN EMERGENCY SHUTDOWN. Manual reset required.");
        lastEmergencyAlert = millis();
      }
    }
    if (digitalRead(btnResetPin) == LOW) {
      Serial.println("Reset button pressed. Returning to IDLE.");
      currentState = STATE_IDLE;
    }
    break;
  }

  // --- Telemetry Display ---
  static unsigned long lastTelemetryTime = 0;
  if (millis() - lastTelemetryTime >= 1000) {
    lastTelemetryTime = millis();
    Serial.print("T: ");
    Serial.print(tempC, 1);
    Serial.print(" °C | P: ");
    Serial.print(pressurePsi, 2);
    Serial.print(" PSI | Water: ");
    Serial.println(hasWater ? "OK" : "EMPTY");
  }
}
