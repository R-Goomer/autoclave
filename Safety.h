#pragma once
#include <Arduino.h>
#include "Config.h"

// =============================================================================
// SAFETY — global interlocks (run every loop cycle, regardless of state)
// =============================================================================

namespace Safety {

// Checks over-pressure, over-temperature and dry-run interlocks. Drives the
// relevant relays to a safe state via Outputs when a check trips, and
// returns true if the caller should transition to STATE_EMERGENCY_SHUTDOWN.
bool check(float tempC, float pressurePsi, bool hasWater,
           CycleState currentState);

} // namespace Safety
