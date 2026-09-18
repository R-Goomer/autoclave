#pragma once
#include <Arduino.h>

// =============================================================================
// PIN ASSIGNMENTS
// =============================================================================

// --- Input Pins ---
const int btnStartPin = 14;
const int btnResetPin = 13;

// --- Sensor Pins ---
const int lm35Pin = 5;
const int waterLevelPin = 4;
const int waterThreshold = 1850; // Above 1850 = Water detected
// Potentiometer on GPIO 6 — simulates chamber pressure (-1 to 30 PSI gauge)
// Wiring: pot wiper → GPIO 6, ends to 3.3 V and GND
const int simPressurePin = 6;

// --- Relay Pins (Top to Bottom) ---
// NOTE: RELAY_STEAM_INLET physically removed. GPIO 6 is now simPressurePin.
const int RELAY_STEAM_INLET = -1; // REMOVED — do not drive
const int RELAY_DRAIN = 42;
const int RELAY_EXHAUST = 41;
const int RELAY_AIR_VALVE = 40; // Controls vacuum pump + air/vacuum valve (same relay)
const int RELAY_HEATER = 39;
const int RELAY_DOOR_LOCK = 38;

// --- Relay Logic Configuration ---
const int RELAY_ON = HIGH;
const int RELAY_OFF = LOW;

// =============================================================================
// PROCESS PARAMETERS
// =============================================================================

// --- Sterilizing Targets ---
const float TARGET_TEMP_C = 121.0; // Legal minimum sterilizing temp (°C)
const float TARGET_PSI = 15.0;     // Legal minimum sterilizing pressure (PSI gauge)
const unsigned long HOLD_TIME =
    15UL * 60UL * 1000UL; // 15-minute sterilizing hold

// --- Sterilizing Heater Control (PID + time-proportional SSR) ---
// PID holds the chamber at STERILIZE_SETPOINT_C (target + 1 °C margin).
// The PID output (0–100%) is converted to a binary SSR duty cycle over a
// fixed SSR_CYCLE_MS period (SSR minimum cycle time = 2 s).
const float STERILIZE_SETPOINT_C = TARGET_TEMP_C + 1.0f; // 122 °C

// PID tuning gains (heater + LM35 + chamber thermal mass)
const double PID_KP = 40.0;
const double PID_KI = 0.35;
const double PID_KD = 0.0; // Derivative OFF — LM35 too noisy; rely on I

// PID output is a duty cycle in percent (0–100)
const double PID_OUT_MIN = 0.0;
const double PID_OUT_MAX = 100.0;

// Time-proportional SSR parameters
const unsigned long SSR_CYCLE_MS = 2000UL; // SSR minimum cycle time (2 s)
const unsigned long SSR_MIN_PULSE_MS =
    200UL; // Minimum ON/OFF pulse to avoid relay chatter

// PID sample interval (1 s — plenty for a heater thermal mass)
const unsigned long PID_SAMPLE_MS = 1000UL;

// Temperature filter — moving average window (smooths LM35 noise for PID)
const int TEMP_FILTER_WINDOW = 5;

// --- Class B Fractionated Vacuum Purge Parameters ---
// Three alternating cycles of:  vacuum pull  →  heater-on steam inject
// BMP180 cannot read deep vacuum (~10 kPa abs), so pulse depth is enforced by
// time.
const int VACUUM_PULSE_COUNT = 3;
// Sensor-based purge: vacuum pull ends when gauge pressure reaches this value
// (or deeper). Pot simulates -2 to +30 PSI gauge; -0.8 PSI ≈ light vacuum.
const float VACUUM_TARGET_PSI = -0.8;

// --- Global Safety Limits ---
const float MAX_SAFE_PSI = 30.0;     // Immediate blowoff threshold
const float MAX_SAFE_TEMP_C = 150.0; // Emergency heater cutoff threshold

// =============================================================================
// STATE MACHINE
// =============================================================================

enum CycleState {
  STATE_IDLE,
  STATE_CHECK_WATER,
  STATE_LOCK_DOOR,
  STATE_PURGE, // Class B 3-pulse fractionated vacuum + await sterilizing
               // conditions
  STATE_STERILIZING,
  STATE_EXHAUST,
  STATE_COMPLETE,
  STATE_EMERGENCY_SHUTDOWN
};
