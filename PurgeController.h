#pragma once
#include <Arduino.h>

// =============================================================================
// PURGE CONTROLLER — Class B fractionated vacuum purge (STATE_PURGE)
// =============================================================================
// Encapsulates the 3-pulse alternating vacuum-pull / steam-inject sequence.
// Drives Outputs::airValve()/heater() itself — the main loop just calls
// update() with the latest sensor readings every iteration while in
// STATE_PURGE.

namespace PurgeController {

// Resets the purge sub-state. Call once on entry to STATE_PURGE.
void reset();

// Runs one purge step. Returns true once sterilizing conditions (target
// temp + PSI) are reached after the 3rd cycle — the caller should then arm
// HeaterController and transition to STATE_STERILIZING.
bool update(float tempC, float pressurePsi);

// For telemetry:
int phase();
const char *phaseName();
int currentPulse(); // 1-based pulse number (1..VACUUM_PULSE_COUNT)

} // namespace PurgeController
