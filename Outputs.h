#pragma once
#include <Arduino.h>

// =============================================================================
// OUTPUTS — relay control primitives
// =============================================================================
// Thin wrapper around the relay GPIOs. No state/logic lives here — stage
// controllers (PurgeController, HeaterController, ...) and the main state
// machine call these directly to drive the physical outputs.

namespace Outputs {

// Configures all relay pins as OUTPUT and drives them to a safe (OFF) state.
void begin();

// Forces every relay OFF (used on entry to IDLE / after an emergency).
void allOff();

void heater(bool on);
void drain(bool on);
void exhaust(bool on);
void airValve(bool on); // vacuum pump + air/vacuum valve (shared relay)
void doorLock(bool on);

} // namespace Outputs
