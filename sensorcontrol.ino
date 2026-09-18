// autoclave project

#include <Wire.h>
#include <PID_v1.h> // Arduino PID Library (Brett Beauregard)

#include "Config.h"
#include "Outputs.h"
#include "Sensors.h"
#include "HeaterController.h"
#include "PurgeController.h"
#include "Safety.h"
#include "Telemetry.h"

// =============================================================================
// STATE
// =============================================================================
// This .ino owns ONLY the abstract state machine + top-level orchestration.
// Each stage's actual behavior lives in its own module (see includes above)
// and is driven by simply passing it the latest sensor readings.

CycleState currentState = STATE_IDLE;
unsigned long stateStartTime = 0;

// =============================================================================
// SETUP
// =============================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  Outputs::begin();

  pinMode(btnStartPin, INPUT_PULLUP);
  pinMode(btnResetPin, INPUT_PULLUP);

  Sensors::begin();
  HeaterController::begin();

  Serial.println("==============================================");
  Serial.println(" Class B Autoclave Controller — Heater-Only  ");
  Serial.println(" Steam inlet valve DISABLED (physically removed)");
  Serial.print(" Sterilizing target: ");
  Serial.print(TARGET_TEMP_C, 1);
  Serial.print("C / ");
  Serial.print(TARGET_PSI, 1);
  Serial.println(" PSI");
  Serial.print(" PID setpoint: ");
  Serial.print(STERILIZE_SETPOINT_C, 1);
  Serial.println("C (target + 1C margin)");
  Serial.print(" PID gains: Kp=");
  Serial.print(PID_KP, 1);
  Serial.print(" Ki=");
  Serial.print(PID_KI, 2);
  Serial.print(" Kd=");
  Serial.println(PID_KD, 1);
  Serial.print(" SSR time-proportional: ");
  Serial.print(SSR_CYCLE_MS / 1000);
  Serial.print("s cycle, min pulse ");
  Serial.print(SSR_MIN_PULSE_MS);
  Serial.println("ms");
  Serial.print(" Floor safety: < ");
  Serial.print(TARGET_TEMP_C, 1);
  Serial.println("C or < target PSI → EMERGENCY");
  Serial.println("==============================================");
}

// =============================================================================
// LOOP
// =============================================================================

void loop() {

  // --- Continuous Metric Acquisition ---
  Sensors::update();
  float tempC = Sensors::tempC();
  float filteredTempC = Sensors::filteredTempC();
  float pressurePsi = Sensors::pressurePsi();
  bool hasWater = Sensors::waterPresent();

  // --- Global Safety Checks (every loop cycle, regardless of state) ---
  if (Safety::check(tempC, pressurePsi, hasWater, currentState)) {
    currentState = STATE_EMERGENCY_SHUTDOWN;
  }

  // --- State Transition Logging ---
  Telemetry::logTransition(currentState);

  // ===========================================================================
  // STATE MACHINE  (abstract logic only — stages delegate to their modules)
  // ===========================================================================
  switch (currentState) {

  // --------------------------------------------------------------------------
  case STATE_IDLE:
    Outputs::allOff();
    if (digitalRead(btnStartPin) == LOW) {
      Serial.println(
          "Start pressed — pot reads gauge PSI directly, no baseline needed.");
      Serial.print("Current sim pressure: ");
      Serial.print(pressurePsi, 2);
      Serial.println(" PSI gauge");
      Serial.println("Initiating sterilization cycle...");
      currentState = STATE_CHECK_WATER;
    }
    break;

  // --------------------------------------------------------------------------
  case STATE_CHECK_WATER:
    if (hasWater) {
      Outputs::doorLock(true);
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

  // --------------------------------------------------------------------------
  case STATE_LOCK_DOOR:
    // Non-blocking 2-second wait for door solenoid to fully engage
    if (millis() - stateStartTime >= 2000) {
      Serial.println(
          "Door locked. Starting Class B fractionated vacuum purge.");
      Serial.print("Purge params: target ");
      Serial.print(TARGET_TEMP_C, 1);
      Serial.print("C / ");
      Serial.print(TARGET_PSI, 1);
      Serial.println(" PSI");
      Serial.print("Vacuum pull: until ");
      Serial.print(VACUUM_TARGET_PSI, 1);
      Serial.print(" PSI (sensor)  |  Steam inject: until ");
      Serial.print(TARGET_TEMP_C, 1);
      Serial.println("C (sensor)  |  3 pulses  [sim pressure via pot]");

      PurgeController::reset();
      stateStartTime = millis();
      currentState = STATE_PURGE;
    }
    break;

  // --------------------------------------------------------------------------
  // CLASS B FRACTIONATED VACUUM PURGE — delegated to PurgeController
  // --------------------------------------------------------------------------
  case STATE_PURGE: {
    bool sterilizingConditionsReached =
        PurgeController::update(tempC, pressurePsi);

    if (sterilizingConditionsReached) {
      // Safe entry into sterilizing: arm PID fresh
      HeaterController::arm(filteredTempC);
      stateStartTime = millis();
      currentState = STATE_STERILIZING;
    }
    break;
  }

  // --------------------------------------------------------------------------
  // STERILIZING — PID + time-proportional SSR delegated to HeaterController
  //
  //  Floor safety (checked first, every cycle):
  //    tempC < TARGET_TEMP_C  OR  pressurePsi < TARGET_PSI  → EMERGENCY
  //    SHUTDOWN
  // --------------------------------------------------------------------------
  case STATE_STERILIZING: {

    // 1. Sterilizing floor — legal minimum check
    if (tempC < TARGET_TEMP_C || pressurePsi < TARGET_PSI) {
      Serial.print("EMERGENCY: Sterilizing floor breached! T=");
      Serial.print(tempC, 1);
      Serial.print("C / P=");
      Serial.print(pressurePsi, 2);
      Serial.println(
          " PSI — below legal minimum. Stopping & awaiting manual reset.");
      HeaterController::disarm();
      currentState = STATE_EMERGENCY_SHUTDOWN;
      break;
    }

    // 2. PID + time-proportional SSR heater control
    HeaterController::update(filteredTempC);

    // 3. Hold-time complete → exhaust
    if (millis() - stateStartTime >= HOLD_TIME) {
      Serial.println("Sterilizing hold time complete. Opening exhaust.");
      HeaterController::disarm();
      currentState = STATE_EXHAUST;
    }
    break;
  }

  // --------------------------------------------------------------------------
  case STATE_EXHAUST:
    Outputs::exhaust(true);
    Outputs::drain(true);

    if (pressurePsi <= 1.0) {
      Outputs::exhaust(false);
      Outputs::drain(false);
      currentState = STATE_COMPLETE;
    }
    break;

  // --------------------------------------------------------------------------
  case STATE_COMPLETE:
    Serial.println(
        "[SUCCESS] Cycle complete. Chamber depressurised. Unlocking door.");
    Outputs::doorLock(false);
    currentState = STATE_IDLE;
    break;

  // --------------------------------------------------------------------------
  case STATE_EMERGENCY_SHUTDOWN:
    // Kill all heat and pump; vent chamber safely
    Outputs::heater(false);
    Outputs::airValve(false); // Vacuum pump OFF during venting
    Outputs::exhaust(true);
    Outputs::drain(true);

    // Keep door locked until chamber pressure is safe
    Outputs::doorLock(pressurePsi > 1.0);

    {
      static unsigned long lastEmergencyAlert = 0;
      if (millis() - lastEmergencyAlert > 2000) {
        Serial.println("!! EMERGENCY SHUTDOWN — Manual reset required. !!");
        lastEmergencyAlert = millis();
      }
    }

    if (digitalRead(btnResetPin) == LOW) {
      Serial.println("Reset pressed. Returning to IDLE.");
      // Clear stage sub-state so a fresh cycle starts cleanly
      PurgeController::reset();
      HeaterController::disarm();
      currentState = STATE_IDLE;
    }
    break;
  }

  // --- Telemetry (1 Hz, rate-limited internally) ---
  Telemetry::update(currentState, tempC, filteredTempC, pressurePsi, hasWater,
                     stateStartTime);
}
