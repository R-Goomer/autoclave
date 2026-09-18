#include "Safety.h"
#include "Outputs.h"

namespace Safety {

bool check(float tempC, float pressurePsi, bool hasWater,
           CycleState currentState) {
  bool tripped = false;

  // 1. OVER-PRESSURE INTERLOCK
  if (pressurePsi >= MAX_SAFE_PSI) {
    Serial.print(
        "EMERGENCY: OVER-PRESSURE! Opening exhaust & killing heater: ");
    Serial.println(pressurePsi, 2);
    Outputs::heater(false);
    Outputs::exhaust(true);
    tripped = true;
  }

  // 2. OVER-TEMPERATURE INTERLOCK
  if (tempC >= MAX_SAFE_TEMP_C) {
    Serial.println("EMERGENCY: OVER-TEMPERATURE! Shutting down heater.");
    Outputs::heater(false);
    tripped = true;
  }

  // 3. DRY-RUN PROTECTION (heater can be active in PURGE and STERILIZING)
  if (!hasWater &&
      (currentState == STATE_PURGE || currentState == STATE_STERILIZING)) {
    Serial.println("EMERGENCY: WATER LOSS DURING RUN! Immediate cutoff.");
    Outputs::heater(false);
    tripped = true;
  }

  return tripped;
}

} // namespace Safety
