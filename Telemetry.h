#pragma once
#include <Arduino.h>
#include "Config.h"

// =============================================================================
// TELEMETRY — serial status output
// =============================================================================

namespace Telemetry {

// Edge-triggered: prints "--- ENTERING STATE: X" whenever currentState
// changes. Call every loop() iteration, before the state machine runs.
void logTransition(CycleState currentState);

// Rate-limited to 1 Hz internally. Call every loop() iteration.
// tempC/filteredTempC/pressurePsi/hasWater are the latest sensor readings;
// stateStartTime is the millis() timestamp of the current state's entry.
void update(CycleState currentState, float tempC, float filteredTempC,
            float pressurePsi, bool hasWater, unsigned long stateStartTime);

} // namespace Telemetry
