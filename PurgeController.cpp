#include "PurgeController.h"
#include "Config.h"
#include "Outputs.h"

namespace PurgeController {

namespace {
// purgePhase  0,2,4  = vacuum pull  (even)
// purgePhase  1,3,5  = steam inject (odd)
// purgePhase  6      = await target temp + PSI after 3rd cycle
int purgePhase = 0;
} // namespace

void reset() { purgePhase = 0; }

bool update(float tempC, float pressurePsi) {
  if (purgePhase < VACUUM_PULSE_COUNT * 2) {
    bool isVacuumPhase = (purgePhase % 2 == 0);

    if (isVacuumPhase) {
      // Vacuum pump + air/vacuum valve ON; heater OFF
      Outputs::airValve(true);
      Outputs::heater(false);

      // Sensor-based: advance once target vacuum depth is reached
      if (pressurePsi <= VACUUM_TARGET_PSI) {
        purgePhase++;
        Serial.print("Purge: vacuum reached ");
        Serial.print(pressurePsi, 2);
        Serial.print(" PSI → entering phase ");
        Serial.print(purgePhase);
        Serial.print(" [");
        Serial.print(phaseName());
        Serial.println("]");
      }
    } else {
      // Pump OFF; heater ON — boil water to inject steam
      Outputs::airValve(false);
      Outputs::heater(true);

      // Sensor-based: advance once sterilizing temp is reached
      if (tempC >= TARGET_TEMP_C) {
        purgePhase++;
        Serial.print("Purge: temp reached ");
        Serial.print(tempC, 1);
        Serial.print("C → entering phase ");
        Serial.print(purgePhase);
        Serial.print(" [");
        Serial.print(phaseName());
        Serial.println("]");
      }
    }
    return false;
  }

  // --- Phase 6: Await sterilizing conditions after 3rd cycle ---
  Outputs::airValve(false);
  Outputs::heater(true);

  if (tempC >= TARGET_TEMP_C && pressurePsi >= TARGET_PSI) {
    Serial.print("Purge complete: ");
    Serial.print(tempC, 1);
    Serial.print("C / ");
    Serial.print(pressurePsi, 2);
    Serial.println(" PSI — sterilizing conditions reached after 3rd cycle.");

    // Safe hand-off to sterilizing: heater OFF here; HeaterController::arm()
    // (called by the caller) takes over from a clean state.
    Outputs::heater(false);
    return true;
  }
  return false;
}

int phase() { return purgePhase; }

const char *phaseName() {
  if (purgePhase >= VACUUM_PULSE_COUNT * 2)
    return "AWAIT TARGET";
  return (purgePhase % 2 == 0) ? "VACUUM PULL" : "STEAM INJECT";
}

int currentPulse() { return (purgePhase / 2) + 1; }

} // namespace PurgeController
