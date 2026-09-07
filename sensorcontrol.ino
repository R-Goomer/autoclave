//autoclave project

#include <Wire.h>
#include <PID_v1.h> // Arduino PID Library (Brett Beauregard)

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
const int RELAY_AIR_VALVE =
    40; // Controls vacuum pump + air/vacuum valve (same relay)
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
const float TARGET_PSI = 15.0; // Legal minimum sterilizing pressure (PSI gauge)
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
const unsigned long VACUUM_PULL_MS =
    30000; // 30 s — vacuum pump run per pulse (~-0.8 bar by time)
const unsigned long STEAM_INJECT_MS =
    20000; // 20 s — heater-on steam injection between pulses

// --- Global Safety Limits ---
const float MAX_SAFE_PSI = 20.0;     // Immediate blowoff threshold
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

CycleState currentState = STATE_IDLE;
unsigned long stateStartTime = 0;
// baselinePressurePa removed — pot gives gauge PSI directly

// --- Purge Sub-State ---
// purgePhase  0,2,4  = vacuum pull  (even)
// purgePhase  1,3,5  = steam inject (odd)
// purgePhase  6      = await target temp + PSI after 3rd cycle
int purgePhase = 0;
unsigned long purgePhaseStart = 0;

// --- Sterilizing Heater State (PID + time-proportional SSR) ---
double pidInput = 0.0;  // filtered temperature (°C)
double pidOutput = 0.0; // PID output duty cycle (0–100 %)
double pidSetpoint = STERILIZE_SETPOINT_C;
PID sterilizePID(&pidInput, &pidOutput, &pidSetpoint, PID_KP, PID_KI, PID_KD,
                 DIRECT);

// Time-proportional SSR state
unsigned long ssrCycleStart = 0; // start of current SSR cycle (ms)
bool ssrOn = false;              // whether heater is ON within this cycle
bool pidArmed = false;           // PID active only during STERILIZING

// Temperature moving-average filter
float tempFilterBuf[TEMP_FILTER_WINDOW] = {0};
int tempFilterIdx = 0;
float filteredTempC = 0.0;

// BMP085/BMP180 removed — pressure now read from simulation potentiometer

// =============================================================================
// HELPERS
// =============================================================================

void allRelaysOff() {
  // RELAY_STEAM_INLET is -1 (removed) — do not write
  digitalWrite(RELAY_DRAIN, RELAY_OFF);
  digitalWrite(RELAY_EXHAUST, RELAY_OFF);
  digitalWrite(RELAY_AIR_VALVE, RELAY_OFF);
  digitalWrite(RELAY_HEATER, RELAY_OFF);
  digitalWrite(RELAY_DOOR_LOCK, RELAY_OFF);
}

// Returns a human-readable label for the current purge phase
const char *purgePhaseName(int phase) {
  if (phase >= VACUUM_PULSE_COUNT * 2)
    return "AWAIT TARGET";
  return (phase % 2 == 0) ? "VACUUM PULL" : "STEAM INJECT";
}

// Returns the 1-based pulse number for telemetry (1–3)
int purgeCurrentPulse(int phase) { return (phase / 2) + 1; }

// =============================================================================
// TEMPERATURE FILTER  (moving average — smooths LM35 noise for the PID)
// =============================================================================
float filterTemp(float rawC) {
  tempFilterBuf[tempFilterIdx] = rawC;
  tempFilterIdx = (tempFilterIdx + 1) % TEMP_FILTER_WINDOW;

  float sum = 0.0f;
  for (int i = 0; i < TEMP_FILTER_WINDOW; i++)
    sum += tempFilterBuf[i];
  return sum / (float)TEMP_FILTER_WINDOW;
}

// =============================================================================
// TIME-PROPORTIONAL SSR DRIVER
// =============================================================================
// Converts a PID duty cycle (0–100 %) into a binary SSR ON/OFF pattern over a
// fixed SSR_CYCLE_MS period. Enforces a minimum pulse to avoid relay chatter.
// Call every loop() iteration; returns the desired heater relay state.
bool driveSSR(double dutyPercent) {
  unsigned long now = millis();

  // Clamp duty to [0,100]
  if (dutyPercent < 0.0)
    dutyPercent = 0.0;
  if (dutyPercent > 100.0)
    dutyPercent = 100.0;

  // Start a fresh cycle
  if (now - ssrCycleStart >= SSR_CYCLE_MS) {
    ssrCycleStart = now;
    ssrOn = (dutyPercent > 0.0);
  }

  // Compute ON duration for this cycle (with minimum-pulse floor)
  unsigned long onMs = (unsigned long)((dutyPercent / 100.0) *
                                       (double)SSR_CYCLE_MS);
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

// =============================================================================
// SETUP
// =============================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  int relayPins[] = {RELAY_DRAIN, RELAY_EXHAUST, RELAY_AIR_VALVE, RELAY_HEATER,
                     RELAY_DOOR_LOCK};

  for (int pin : relayPins) {
    digitalWrite(pin, RELAY_OFF); // Safe state before setting direction
    pinMode(pin, OUTPUT);
  }

  pinMode(btnStartPin, INPUT_PULLUP);
  pinMode(btnResetPin, INPUT_PULLUP);

  analogSetPinAttenuation(lm35Pin, ADC_11db);
  analogSetPinAttenuation(waterLevelPin, ADC_11db);
  analogSetPinAttenuation(simPressurePin, ADC_11db); // Pressure simulation pot

  // --- Configure PID controller ---
  sterilizePID.SetOutputLimits(PID_OUT_MIN, PID_OUT_MAX);
  sterilizePID.SetSampleTime(PID_SAMPLE_MS);
  sterilizePID.SetMode(MANUAL); // Armed (AUTOMATIC) only on sterilizing entry
  sterilizePID.SetTunings(PID_KP, PID_KI, PID_KD);

  Serial.println("==============================================");
  Serial.println(" Class B Autoclave Controller — Heater-Only  ");
  Serial.println(" Steam inlet valve DISABLED (physically removed)");
  Serial.print(" Sterilizing target: ");
  Serial.print(TARGET_TEMP_C, 1);
  Serial.print("C / ");
  Serial.print(TARGET_PSI, 1);
  Serial.println(" PSI");
  Serial.print(" PID setpoint: ");
  Serial.print(STERILIZE_SETPOINT_C, 1);
  Serial.println("C (target + 1C margin)");
  Serial.print(" PID gains: Kp=");
  Serial.print(PID_KP, 1);
  Serial.print(" Ki=");
  Serial.print(PID_KI, 2);
  Serial.print(" Kd=");
  Serial.println(PID_KD, 1);
  Serial.print(" SSR time-proportional: ");
  Serial.print(SSR_CYCLE_MS / 1000);
  Serial.print("s cycle, min pulse ");
  Serial.print(SSR_MIN_PULSE_MS);
  Serial.println("ms");
  Serial.print(" Floor safety: < ");
  Serial.print(TARGET_TEMP_C, 1);
  Serial.println("C or < target PSI → EMERGENCY");
  Serial.println("==============================================");
}

// =============================================================================
// LOOP
// =============================================================================

void loop() {

  // --- Continuous Metric Acquisition ---
  float tempC = analogReadMilliVolts(lm35Pin) / 10.0;
  filteredTempC = filterTemp(tempC); // smoothed for PID / telemetry

  // Pressure simulation: pot wiper on GPIO 6
  // Full CCW (0 mV)               = -2 PSI       ← deep vacuum side
  // Full CW  (3300 mV)            = +30 PSI       ← sterilizing side
  float pressurePsi = -2.0 + (analogReadMilliVolts(simPressurePin) - 656.0) * (27.0 / (4095.0 - 656.0));
  pressurePsi = constrain(pressurePsi, -2.0, 25.0);

  bool hasWater = (analogRead(waterLevelPin) > waterThreshold);

  // ===========================================================================
  // GLOBAL SAFETY CHECKS  (run every loop cycle, regardless of state)
  // ===========================================================================

  // 1. OVER-PRESSURE INTERLOCK
  if (pressurePsi >= MAX_SAFE_PSI) {
    Serial.print(
        "EMERGENCY: OVER-PRESSURE! Opening exhaust & killing heater: ");
    Serial.println(pressurePsi, 2);
    digitalWrite(RELAY_HEATER, RELAY_OFF);
    digitalWrite(RELAY_EXHAUST, RELAY_ON);
    currentState = STATE_EMERGENCY_SHUTDOWN;
  }

  // 2. OVER-TEMPERATURE INTERLOCK
  if (tempC >= MAX_SAFE_TEMP_C) {
    Serial.println("EMERGENCY: OVER-TEMPERATURE! Shutting down heater.");
    digitalWrite(RELAY_HEATER, RELAY_OFF);
    currentState = STATE_EMERGENCY_SHUTDOWN;
  }

  // 3. DRY-RUN PROTECTION (heater can be active in PURGE and STERILIZING)
  if (!hasWater &&
      (currentState == STATE_PURGE || currentState == STATE_STERILIZING)) {
    Serial.println("EMERGENCY: WATER LOSS DURING RUN! Immediate cutoff.");
    digitalWrite(RELAY_HEATER, RELAY_OFF);
    currentState = STATE_EMERGENCY_SHUTDOWN;
  }

  // ===========================================================================
  // STATE TRANSITION LOGGING
  // ===========================================================================
  static CycleState lastState = (CycleState)-1;
  if (currentState != lastState) {
    Serial.print("--- ENTERING STATE: ");
    switch (currentState) {
    case STATE_IDLE:
      Serial.println("IDLE");
      break;
    case STATE_CHECK_WATER:
      Serial.println("CHECK_WATER");
      break;
    case STATE_LOCK_DOOR:
      Serial.println("LOCK_DOOR");
      break;
    case STATE_PURGE:
      Serial.println("PURGE");
      break;
    case STATE_STERILIZING:
      Serial.println("STERILIZING");
      break;
    case STATE_EXHAUST:
      Serial.println("EXHAUST");
      break;
    case STATE_COMPLETE:
      Serial.println("COMPLETE");
      break;
    case STATE_EMERGENCY_SHUTDOWN:
      Serial.println("EMERGENCY_SHUTDOWN");
      break;
    }
    lastState = currentState;
  }

  // ===========================================================================
  // STATE MACHINE
  // ===========================================================================
  switch (currentState) {

  // --------------------------------------------------------------------------
  case STATE_IDLE:
    allRelaysOff();
    if (digitalRead(btnStartPin) == LOW) {
      Serial.println(
          "Start pressed — pot reads gauge PSI directly, no baseline needed.");
      Serial.print("Current sim pressure: ");
      Serial.print(pressurePsi, 2);
      Serial.println(" PSI gauge");
      Serial.println("Initiating sterilization cycle...");
      currentState = STATE_CHECK_WATER;
    }
    break;

  // --------------------------------------------------------------------------
  case STATE_CHECK_WATER:
    if (hasWater) {
      digitalWrite(RELAY_DOOR_LOCK, RELAY_ON);
      stateStartTime = millis();
      currentState = STATE_LOCK_DOOR;
    } else {
      static unsigned long lastWaterAlert = 0;
      if (millis() - lastWaterAlert > 2000) {
        Serial.println("ALERT: Insufficient water. Awaiting refill...");
        lastWaterAlert = millis();
      }
    }
    break;

  // --------------------------------------------------------------------------
  case STATE_LOCK_DOOR:
    // Non-blocking 2-second wait for door solenoid to fully engage
    if (millis() - stateStartTime >= 2000) {
      Serial.println(
          "Door locked. Starting Class B fractionated vacuum purge.");
      Serial.print("Purge params: target ");
      Serial.print(TARGET_TEMP_C, 1);
      Serial.print("C / ");
      Serial.print(TARGET_PSI, 1);
      Serial.println(" PSI");
      Serial.print("Vacuum pull: ");
      Serial.print(VACUUM_PULL_MS / 1000);
      Serial.print("s (timed)  |  Steam inject: ");
      Serial.print(STEAM_INJECT_MS / 1000);
      Serial.println("s (timed)  |  3 pulses  [sim pressure via pot]");

      // Initialise purge sub-state
      purgePhase = 0;
      purgePhaseStart = millis();
      stateStartTime = millis();
      currentState = STATE_PURGE;
    }
    break;

  // --------------------------------------------------------------------------
  // CLASS B FRACTIONATED VACUUM PURGE
  //
  //  Phases 0→5  (3 vacuum-pull / steam-inject pairs):
  //    Even (0,2,4): RELAY_AIR_VALVE ON  (vacuum pump + valve)  —
  //    VACUUM_PULL_MS Odd  (1,3,5): RELAY_AIR_VALVE OFF, RELAY_HEATER ON —
  //    STEAM_INJECT_MS
  //
  //  Phase 6:  All 3 pulses done.  Heater ON, pump OFF.
  //            Await TARGET_TEMP_C AND TARGET_PSI simultaneously.
  //            → advances to STATE_STERILIZING.
  // --------------------------------------------------------------------------
  case STATE_PURGE: {
    unsigned long phaseElapsed = millis() - purgePhaseStart;

    if (purgePhase < VACUUM_PULSE_COUNT * 2) {
      bool isVacuumPhase = (purgePhase % 2 == 0);
      unsigned long phaseDuration =
          isVacuumPhase ? VACUUM_PULL_MS : STEAM_INJECT_MS;

      if (isVacuumPhase) {
        // Vacuum pump + air/vacuum valve ON; heater OFF
        digitalWrite(RELAY_AIR_VALVE, RELAY_ON);
        digitalWrite(RELAY_HEATER, RELAY_OFF);
      } else {
        // Pump OFF; heater ON — boil water to inject steam
        digitalWrite(RELAY_AIR_VALVE, RELAY_OFF);
        digitalWrite(RELAY_HEATER, RELAY_ON);
      }

      if (phaseElapsed >= phaseDuration) {
        purgePhase++;
        purgePhaseStart = millis();

        Serial.print("Purge: phase ");
        Serial.print(purgePhase - 1);
        Serial.print(" complete → entering phase ");
        Serial.print(purgePhase);
        Serial.print(" [");
        Serial.print(purgePhaseName(purgePhase));
        Serial.println("]");
      }

    } else {
      // --- Phase 6: Await sterilizing conditions after 3rd cycle ---
      digitalWrite(RELAY_AIR_VALVE, RELAY_OFF);
      digitalWrite(RELAY_HEATER, RELAY_ON);

      if (tempC >= TARGET_TEMP_C && pressurePsi >= TARGET_PSI) {
        Serial.print("Purge complete: ");
        Serial.print(tempC, 1);
        Serial.print("C / ");
        Serial.print(pressurePsi, 2);
        Serial.println(
            " PSI — sterilizing conditions reached after 3rd cycle.");

        // Safe entry into sterilizing: heater OFF, arm PID fresh
        digitalWrite(RELAY_HEATER, RELAY_OFF);
        ssrOn = false;
        ssrCycleStart = millis();
        pidInput = filteredTempC;
        sterilizePID.SetMode(AUTOMATIC); // Arm PID (anti-windup: fresh I term)
        pidArmed = true;
        stateStartTime = millis();
        currentState = STATE_STERILIZING;
      }
    }
    break;
  }

  // --------------------------------------------------------------------------
  // STERILIZING
  //
  //  PID + time-proportional SSR control:
  //    - PID holds filtered temp at STERILIZE_SETPOINT_C (122 °C)
  //    - PID output (0–100 %) → SSR duty cycle over SSR_CYCLE_MS (2 s)
  //    - Anti-windup: PID armed fresh on entry; output clamped 0–100 %
  //
  //  Floor safety (checked first, every cycle):
  //    tempC < TARGET_TEMP_C  OR  pressurePsi < TARGET_PSI  → EMERGENCY
  //    SHUTDOWN
  // --------------------------------------------------------------------------
  case STATE_STERILIZING: {

    // 1. Sterilizing floor — legal minimum check
    if (tempC < TARGET_TEMP_C || pressurePsi < TARGET_PSI) {
      Serial.print("EMERGENCY: Sterilizing floor breached! T=");
      Serial.print(tempC, 1);
      Serial.print("C / P=");
      Serial.print(pressurePsi, 2);
      Serial.println(
          " PSI — below legal minimum. Stopping & awaiting manual reset.");
      digitalWrite(RELAY_HEATER, RELAY_OFF);
      ssrOn = false;
      pidArmed = false;
      sterilizePID.SetMode(MANUAL);
      currentState = STATE_EMERGENCY_SHUTDOWN;
      break;
    }

    // 2. PID + time-proportional SSR heater control
    if (pidArmed) {
      pidInput = filteredTempC;
      sterilizePID.Compute(); // updates pidOutput (0–100 %) at sample rate
      bool heaterOn = driveSSR(pidOutput);
      digitalWrite(RELAY_HEATER, heaterOn ? RELAY_ON : RELAY_OFF);
    } else {
      digitalWrite(RELAY_HEATER, RELAY_OFF);
    }

    // 3. Hold-time complete → exhaust
    if (millis() - stateStartTime >= HOLD_TIME) {
      Serial.println("Sterilizing hold time complete. Opening exhaust.");
      digitalWrite(RELAY_HEATER, RELAY_OFF);
      ssrOn = false;
      pidArmed = false;
      sterilizePID.SetMode(MANUAL);
      currentState = STATE_EXHAUST;
    }
    break;
  }

  // --------------------------------------------------------------------------
  case STATE_EXHAUST:
    digitalWrite(RELAY_EXHAUST, RELAY_ON);
    digitalWrite(RELAY_DRAIN, RELAY_ON);

    if (pressurePsi <= 1.0) {
      digitalWrite(RELAY_EXHAUST, RELAY_OFF);
      digitalWrite(RELAY_DRAIN, RELAY_OFF);
      currentState = STATE_COMPLETE;
    }
    break;

  // --------------------------------------------------------------------------
  case STATE_COMPLETE:
    Serial.println(
        "[SUCCESS] Cycle complete. Chamber depressurised. Unlocking door.");
    digitalWrite(RELAY_DOOR_LOCK, RELAY_OFF);
    currentState = STATE_IDLE;
    break;

  // --------------------------------------------------------------------------
  case STATE_EMERGENCY_SHUTDOWN:
    // Kill all heat and pump; vent chamber safely
    digitalWrite(RELAY_HEATER, RELAY_OFF);
    digitalWrite(RELAY_AIR_VALVE, RELAY_OFF); // Vacuum pump OFF during venting
    digitalWrite(RELAY_EXHAUST, RELAY_ON);
    digitalWrite(RELAY_DRAIN, RELAY_ON);

    // Keep door locked until chamber pressure is safe
    digitalWrite(RELAY_DOOR_LOCK, (pressurePsi > 1.0) ? RELAY_ON : RELAY_OFF);

    {
      static unsigned long lastEmergencyAlert = 0;
      if (millis() - lastEmergencyAlert > 2000) {
        Serial.println("!! EMERGENCY SHUTDOWN — Manual reset required. !!");
        lastEmergencyAlert = millis();
      }
    }

    if (digitalRead(btnResetPin) == LOW) {
      Serial.println("Reset pressed. Returning to IDLE.");
      // Clear purge sub-state so a fresh cycle starts cleanly
      purgePhase = 0;
      ssrOn = false;
      pidArmed = false;
      sterilizePID.SetMode(MANUAL);
      currentState = STATE_IDLE;
    }
    break;
  }

  // ===========================================================================
  // TELEMETRY  (1 Hz)
  // ===========================================================================
  static unsigned long lastTelemetryTime = 0;
  if (millis() - lastTelemetryTime >= 1000) {
    lastTelemetryTime = millis();

    Serial.print("T: ");
    Serial.print(tempC, 1);
    Serial.print("C | P: ");
    Serial.print(pressurePsi, 2);
    Serial.print(" PSI | Water: ");
    Serial.print(hasWater ? "OK" : "EMPTY");
    Serial.print(" | ");

    unsigned long elapsed = millis() - stateStartTime;

    switch (currentState) {
    case STATE_IDLE:
      Serial.println("Waiting for START button");
      break;

    case STATE_CHECK_WATER:
      Serial.println("Checking water level...");
      break;

    case STATE_LOCK_DOOR: {
      unsigned long rem = (elapsed < 2000) ? (2000 - elapsed) / 1000 : 0;
      Serial.print("Locking door... ");
      Serial.print(rem);
      Serial.println("s left");
      break;
    }

    case STATE_PURGE: {
      if (purgePhase < VACUUM_PULSE_COUNT * 2) {
        bool isVacuumPhase = (purgePhase % 2 == 0);
        unsigned long phaseDuration =
            isVacuumPhase ? VACUUM_PULL_MS : STEAM_INJECT_MS;
        unsigned long phaseElapsed = millis() - purgePhaseStart;
        unsigned long rem = (phaseElapsed < phaseDuration)
                                ? (phaseDuration - phaseElapsed) / 1000
                                : 0;
        Serial.print("Purge Pulse ");
        Serial.print(purgeCurrentPulse(purgePhase));
        Serial.print("/");
        Serial.print(VACUUM_PULSE_COUNT);
        Serial.print(" [");
        Serial.print(purgePhaseName(purgePhase));
        Serial.print("] ");
        Serial.print(rem);
        Serial.println("s left");
      } else {
        Serial.print("Purge: awaiting target  T>=");
        Serial.print(TARGET_TEMP_C, 1);
        Serial.print("C  P>=");
        Serial.print(TARGET_PSI, 1);
        Serial.println(" PSI");
      }
      break;
    }

    case STATE_STERILIZING: {
      unsigned long holdRemaining =
          (elapsed < HOLD_TIME) ? (HOLD_TIME - elapsed) / 1000 : 0;
      Serial.print("Sterilizing [Heater:");
      Serial.print(ssrOn ? "ON " : "OFF");
      Serial.print("  PID:");
      Serial.print(pidOutput, 0);
      Serial.print("%  set:");
      Serial.print(STERILIZE_SETPOINT_C, 1);
      Serial.print("C  T:");
      Serial.print(filteredTempC, 1);
      Serial.print("C  floor:");
      Serial.print(TARGET_TEMP_C, 0);
      Serial.print("C/");
      Serial.print(TARGET_PSI, 0);
      Serial.print("PSI]  hold:");
      Serial.print(holdRemaining);
      Serial.println("s left");
      break;
    }

    case STATE_EXHAUST:
      Serial.println("Exhausting... Waiting for <= 1.0 PSI");
      break;

    case STATE_COMPLETE:
      Serial.println("Cycle complete! Unlocking door...");
      break;

    case STATE_EMERGENCY_SHUTDOWN:
      Serial.println(
          "EMERGENCY! Awaiting RESET button (door locked until <= 1.0 PSI)");
      break;
    }
  }
}
