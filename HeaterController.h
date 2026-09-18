#pragma once
#include <Arduino.h>

// =============================================================================
// HEATER CONTROLLER — PID + time-proportional SSR (sterilizing stage)
// =============================================================================
// Owns the PID controller and the SSR duty-cycle driver used to hold the
// chamber at STERILIZE_SETPOINT_C. Drives the heater relay itself via
// Outputs::heater() — the caller only needs to arm/update/disarm.

namespace HeaterController {

void begin();

// Arms the PID fresh (anti-windup — fresh integral term) and starts a new
// SSR cycle. Call once on entry to STATE_STERILIZING.
void arm(float currentFilteredTempC);

// Disarms the PID (MANUAL mode) and forces the heater relay off. Call on
// exit from STATE_STERILIZING (floor breach or hold-time complete).
void disarm();

// Runs one PID + SSR step and drives Outputs::heater() accordingly.
// Call every loop() iteration while armed (i.e. while in STATE_STERILIZING).
void update(float filteredTempC);

bool isOn();          // current SSR relay state (for telemetry)
double dutyPercent();  // last PID output, 0-100% (for telemetry)

} // namespace HeaterController
