#include "HeaterController.h"
#include "Config.h"
#include "Outputs.h"
#include <PID_v1.h>

namespace HeaterController {

namespace {
double pidInput = 0.0;
double pidOutput = 0.0;
double pidSetpoint = STERILIZE_SETPOINT_C;
PID sterilizePID(&pidInput, &pidOutput, &pidSetpoint, PID_KP, PID_KI, PID_KD,
                  DIRECT);

unsigned long ssrCycleStart = 0; // start of current SSR cycle (ms)
bool ssrOn = false;              // whether heater is ON within this cycle

// Converts a PID duty cycle (0–100 %) into a binary SSR ON/OFF pattern over
// a fixed SSR_CYCLE_MS period. Enforces a minimum pulse to avoid relay
// chatter. Call every loop() iteration; returns the desired heater state.
bool driveSSR(double dutyPercent_) {
  unsigned long now = millis();

  if (dutyPercent_ < 0.0)
    dutyPercent_ = 0.0;
  if (dutyPercent_ > 100.0)
    dutyPercent_ = 100.0;

  // Start a fresh cycle
  if (now - ssrCycleStart >= SSR_CYCLE_MS) {
    ssrCycleStart = now;
    ssrOn = (dutyPercent_ > 0.0);
  }

  // Compute ON duration for this cycle (with minimum-pulse floor)
  unsigned long onMs =
      (unsigned long)((dutyPercent_ / 100.0) * (double)SSR_CYCLE_MS);
  if (onMs > 0 && onMs < SSR_MIN_PULSE_MS)
    onMs = SSR_MIN_PULSE_MS;

  unsigned long elapsed = now - ssrCycleStart;

  if (ssrOn) {
    // Turn OFF once the ON window elapses (or at cycle end)
    if (elapsed >= onMs)
      ssrOn = false;
  } else {
    // Stay OFF for the remainder of the cycle
    if (elapsed >= SSR_CYCLE_MS)
      ssrOn = true; // next cycle begins
  }

  return ssrOn;
}
} // namespace

void begin() {
  sterilizePID.SetOutputLimits(PID_OUT_MIN, PID_OUT_MAX);
  sterilizePID.SetSampleTime(PID_SAMPLE_MS);
  sterilizePID.SetMode(MANUAL); // Armed (AUTOMATIC) only on sterilizing entry
  sterilizePID.SetTunings(PID_KP, PID_KI, PID_KD);
}

void arm(float currentFilteredTempC) {
  ssrOn = false;
  ssrCycleStart = millis();
  pidInput = currentFilteredTempC;
  sterilizePID.SetMode(AUTOMATIC); // Arm PID (anti-windup: fresh I term)
}

void disarm() {
  sterilizePID.SetMode(MANUAL);
  ssrOn = false;
  Outputs::heater(false);
}

void update(float filteredTempC) {
  pidInput = filteredTempC;
  sterilizePID.Compute(); // updates pidOutput (0–100 %) at sample rate
  bool heaterOn = driveSSR(pidOutput);
  Outputs::heater(heaterOn);
}

bool isOn() { return ssrOn; }
double dutyPercent() { return pidOutput; }

} // namespace HeaterController
