#include "Telemetry.h"
#include "HeaterController.h"
#include "PurgeController.h"

namespace Telemetry {

void logTransition(CycleState currentState) {
  static CycleState lastState = (CycleState)-1;
  if (currentState == lastState)
    return;
  lastState = currentState;

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
}

void update(CycleState currentState, float tempC, float filteredTempC,
            float pressurePsi, bool hasWater, unsigned long stateStartTime) {
  static unsigned long lastTelemetryTime = 0;
  if (millis() - lastTelemetryTime < 1000)
    return;
  lastTelemetryTime = millis();

  Serial.print("T: ");
  Serial.print(tempC, 1);
  Serial.print("C | P: ");
  Serial.print(pressurePsi, 2);
  Serial.print(" PSI | Water: ");
  Serial.print(hasWater ? "OK" : "EMPTY");
  Serial.print(" | ");

  unsigned long elapsed = millis() - stateStartTime;

  switch (currentState) {
  case STATE_IDLE:
    Serial.println("Waiting for START button");
    break;

  case STATE_CHECK_WATER:
    Serial.println("Checking water level...");
    break;

  case STATE_LOCK_DOOR: {
    unsigned long rem = (elapsed < 2000) ? (2000 - elapsed) / 1000 : 0;
    Serial.print("Locking door... ");
    Serial.print(rem);
    Serial.println("s left");
    break;
  }

  case STATE_PURGE: {
    if (PurgeController::phase() < VACUUM_PULSE_COUNT * 2) {
      bool isVacuumPhase = (PurgeController::phase() % 2 == 0);
      Serial.print("Purge Pulse ");
      Serial.print(PurgeController::currentPulse());
      Serial.print("/");
      Serial.print(VACUUM_PULSE_COUNT);
      Serial.print(" [");
      Serial.print(PurgeController::phaseName());
      if (isVacuumPhase) {
        Serial.print("  P:");
        Serial.print(pressurePsi, 2);
        Serial.print(" / target ");
        Serial.print(VACUUM_TARGET_PSI, 1);
        Serial.print(" PSI");
      } else {
        Serial.print("  T:");
        Serial.print(tempC, 1);
        Serial.print(" / target ");
        Serial.print(TARGET_TEMP_C, 1);
        Serial.print("C");
      }
      Serial.println("]");
    } else {
      Serial.print("Purge: awaiting target  T>=");
      Serial.print(TARGET_TEMP_C, 1);
      Serial.print("C  P>=");
      Serial.print(TARGET_PSI, 1);
      Serial.println(" PSI");
    }
    break;
  }

  case STATE_STERILIZING: {
    unsigned long holdRemaining =
        (elapsed < HOLD_TIME) ? (HOLD_TIME - elapsed) / 1000 : 0;
    Serial.print("Sterilizing [Heater:");
    Serial.print(HeaterController::isOn() ? "ON " : "OFF");
    Serial.print("  PID:");
    Serial.print(HeaterController::dutyPercent(), 0);
    Serial.print("%  set:");
    Serial.print(STERILIZE_SETPOINT_C, 1);
    Serial.print("C  T:");
    Serial.print(filteredTempC, 1);
    Serial.print("C  floor:");
    Serial.print(TARGET_TEMP_C, 0);
    Serial.print("C/");
    Serial.print(TARGET_PSI, 0);
    Serial.print("PSI]  hold:");
    Serial.print(holdRemaining);
    Serial.println("s left");
    break;
  }

  case STATE_EXHAUST:
    Serial.println("Exhausting... Waiting for <= 1.0 PSI");
    break;

  case STATE_COMPLETE:
    Serial.println("Cycle complete! Unlocking door...");
    break;

  case STATE_EMERGENCY_SHUTDOWN:
    Serial.println(
        "EMERGENCY! Awaiting RESET button (door locked until <= 1.0 PSI)");
    break;
  }
}

} // namespace Telemetry
