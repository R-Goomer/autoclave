#pragma once
#include <Arduino.h>

// =============================================================================
// SENSORS — acquisition + filtering
// =============================================================================
// Call begin() once from setup(), then update() once per loop() iteration.
// After update(), use the getters below to read the latest values.

namespace Sensors {

void begin();
void update(); // reads raw sensors & advances the temperature moving average

float tempC();         // raw LM35 temperature (°C)
float filteredTempC();  // moving-average filtered temperature (°C) — used by PID
float pressurePsi();    // simulated chamber pressure (PSI gauge)
bool waterPresent();     // true when water level is above threshold

} // namespace Sensors
