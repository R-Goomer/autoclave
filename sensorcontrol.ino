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

CycleState currentState = STATE_IDLE;
unsigned long stateStartTime = 0;
// baselinePressurePa removed — pot gives gauge PSI directly

// --- Purge Sub-State ---
// purgePhase  0,2,4  = vacuum pull  (even)
// purgePhase  1,3,5  = steam inject (odd)
// purgePhase  6      = await target temp + PSI after 3rd cycle
int purgePhase = 0;

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

// Returns a human-readable label for the current cycle state
const char *stateName(CycleState s) {
  switch (s) {
  case STATE_IDLE:
    return "IDLE";
  case STATE_CHECK_WATER:
    return "CHECK_WATER";
  case STATE_LOCK_DOOR:
    return "LOCK_DOOR";
  case STATE_PURGE:
    return "PURGE";
  case STATE_STERILIZING:
    return "STERILIZING";
  case STATE_EXHAUST:
    return "EXHAUST";
  case STATE_COMPLETE:
    return "COMPLETE";
  case STATE_EMERGENCY_SHUTDOWN:
    return "EMERGENCY_SHUTDOWN";
  }
  return "UNKNOWN";
}

// =============================================================================
// TELEMETRY  (single JSON structure, filled in as the cycle progresses)
// =============================================================================
// One struct is reused for the whole cycle. Fields that don't apply to the
// current stage keep their key but are emitted as `null` in the final JSON.
struct Telemetry {
  // --- Always populated ---
  const char *state;
  float tempC;
  float pressurePsi;
  bool hasWater;

  // --- Purge ---
  bool hasPurge;
  int purgePhase;
  const char *purgePhaseName;
  int purgePulse;
  float vacuumTargetPsi;
  float steamTargetTempC;

  // --- Sterilizing ---
  bool hasSterilizing;
  float pidOutput;
  bool ssrOn;
  float pidSetpoint;
  float filteredTempC;
  unsigned long holdRemaining;

  // --- Door lock ---
  bool doorLocked;

  // --- Emergency ---
  bool hasEmergency;
  const char *emergencyReason;
};

Telemetry telemetry;

// Emits the single JSON object for the whole cycle. Unset fields → null.
void printTelemetryJson() {
  Serial.print("{\"state\":\"");
  Serial.print(telemetry.state);
  Serial.print("\",\"tempC\":");
  Serial.print(telemetry.tempC, 1);
  Serial.print(",\"pressurePsi\":");
  Serial.print(telemetry.pressurePsi, 2);
  Serial.print(",\"hasWater\":");
  Serial.print(telemetry.hasWater ? "true" : "false");

  // Purge
  Serial.print(",\"purge\":");
  if (telemetry.hasPurge) {
    Serial.print("{\"phase\":");
    Serial.print(telemetry.purgePhase);
    Serial.print(",\"phaseName\":\"");
    Serial.print(telemetry.purgePhaseName);
    Serial.print("\",\"pulse\":");
    Serial.print(telemetry.purgePulse);
    Serial.print(",\"vacuumTargetPsi\":");
    Serial.print(telemetry.vacuumTargetPsi, 1);
    Serial.print(",\"steamTargetTempC\":");
    Serial.print(telemetry.steamTargetTempC, 1);
    Serial.print("}");
  } else {
    Serial.print("null");
  }

  // Sterilizing
  Serial.print(",\"sterilizing\":");
  if (telemetry.hasSterilizing) {
    Serial.print("{\"pidOutput\":");
    Serial.print(telemetry.pidOutput, 0);
    Serial.print(",\"ssrOn\":");
    Serial.print(telemetry.ssrOn ? "true" : "false");
    Serial.print(",\"pidSetpoint\":");
    Serial.print(telemetry.pidSetpoint, 1);
    Serial.print(",\"filteredTempC\":");
    Serial.print(telemetry.filteredTempC, 1);
    Serial.print(",\"holdRemaining\":");
    Serial.print(telemetry.holdRemaining);
    Serial.print("}");
  } else {
    Serial.print("null");
  }

  // Door lock
  Serial.print(",\"doorLocked\":");
  Serial.print(telemetry.doorLocked ? "true" : "false");

  // Emergency
  Serial.print(",\"emergency\":");
  if (telemetry.hasEmergency) {
    Serial.print("{\"reason\":\"");
    Serial.print(telemetry.emergencyReason);
    Serial.print("\"}");
  } else {
    Serial.print("null");
  }

  Serial.println("}");
}

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
    telemetry.hasEmergency = true;
    telemetry.emergencyReason = "OVER-PRESSURE";
    digitalWrite(RELAY_HEATER, RELAY_OFF);
    digitalWrite(RELAY_EXHAUST, RELAY_ON);
    currentState = STATE_EMERGENCY_SHUTDOWN;
  }

  // 2. OVER-TEMPERATURE INTERLOCK
  if (tempC >= MAX_SAFE_TEMP_C) {
    telemetry.hasEmergency = true;
    telemetry.emergencyReason = "OVER-TEMPERATURE";
    digitalWrite(RELAY_HEATER, RELAY_OFF);
    currentState = STATE_EMERGENCY_SHUTDOWN;
  }

  // 3. DRY-RUN PROTECTION (heater can be active in PURGE and STERILIZING)
  if (!hasWater &&
      (currentState == STATE_PURGE || currentState == STATE_STERILIZING)) {
    telemetry.hasEmergency = true;
    telemetry.emergencyReason = "WATER LOSS";
    digitalWrite(RELAY_HEATER, RELAY_OFF);
    currentState = STATE_EMERGENCY_SHUTDOWN;
  }

  // ===========================================================================
  // STATE TRANSITION LOGGING
  // ===========================================================================
  static CycleState lastState = (CycleState)-1;
  if (currentState != lastState) {
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
      // Reset telemetry for a fresh cycle
      telemetry = Telemetry{};
      telemetry.state = stateName(currentState);
      currentState = STATE_CHECK_WATER;
    }
    break;

  // --------------------------------------------------------------------------
  case STATE_CHECK_WATER:
    if (hasWater) {
      digitalWrite(RELAY_DOOR_LOCK, RELAY_ON);
      stateStartTime = millis();
      currentState = STATE_LOCK_DOOR;
    }
    break;

  // --------------------------------------------------------------------------
  case STATE_LOCK_DOOR:
    // Non-blocking 2-second wait for door solenoid to fully engage
    if (millis() - stateStartTime >= 2000) {
      // Initialise purge sub-state
      purgePhase = 0;
      stateStartTime = millis();
      currentState = STATE_PURGE;
    }
    break;

  // --------------------------------------------------------------------------
  // CLASS B FRACTIONATED VACUUM PURGE  (sensor-based)
  //
  //  Phases 0→5  (3 vacuum-pull / steam-inject pairs):
  //    Even (0,2,4): RELAY_AIR_VALVE ON  (vacuum pump + valve)  —
  //    runs until pressurePsi <= VACUUM_TARGET_PSI (-0.8 PSI)
  //    Odd  (1,3,5): RELAY_AIR_VALVE OFF, RELAY_HEATER ON —
  //    runs until tempC >= TARGET_TEMP_C
  //
  //  Phase 6:  All 3 pulses done.  Heater ON, pump OFF.
  //            Await TARGET_TEMP_C AND TARGET_PSI simultaneously.
  //            → advances to STATE_STERILIZING.
  // --------------------------------------------------------------------------
  case STATE_PURGE: {

    if (purgePhase < VACUUM_PULSE_COUNT * 2) {
      bool isVacuumPhase = (purgePhase % 2 == 0);

      if (isVacuumPhase) {
        // Vacuum pump + air/vacuum valve ON; heater OFF
        digitalWrite(RELAY_AIR_VALVE, RELAY_ON);
        digitalWrite(RELAY_HEATER, RELAY_OFF);

        // Sensor-based: advance once target vacuum depth is reached
        if (pressurePsi <= VACUUM_TARGET_PSI) {
          purgePhase++;
        }
      } else {
        // Pump OFF; heater ON — boil water to inject steam
        digitalWrite(RELAY_AIR_VALVE, RELAY_OFF);
        digitalWrite(RELAY_HEATER, RELAY_ON);

        // Sensor-based: advance once sterilizing temp is reached
        if (tempC >= TARGET_TEMP_C) {
          purgePhase++;
        }
      }

    } else {
      // --- Phase 6: Await sterilizing conditions after 3rd cycle ---
      digitalWrite(RELAY_AIR_VALVE, RELAY_OFF);
      digitalWrite(RELAY_HEATER, RELAY_ON);

      if (tempC >= TARGET_TEMP_C && pressurePsi >= TARGET_PSI) {
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
      telemetry.hasEmergency = true;
      telemetry.emergencyReason = "STERILIZING FLOOR BREACH";
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
    // Final telemetry snapshot, then emit the single JSON for the cycle
    telemetry.state = stateName(currentState);
    telemetry.tempC = tempC;
    telemetry.pressurePsi = pressurePsi;
    telemetry.hasWater = hasWater;
    printTelemetryJson();
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

    if (digitalRead(btnResetPin) == LOW) {
      // Emit the single JSON for the aborted cycle
      telemetry.state = stateName(currentState);
      telemetry.tempC = tempC;
      telemetry.pressurePsi = pressurePsi;
      telemetry.hasWater = hasWater;
      printTelemetryJson();
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
  // TELEMETRY  (every 2 s) — fill the single JSON structure and print it live
  // ===========================================================================
  static unsigned long lastTelemetryTime = 0;
  if (millis() - lastTelemetryTime >= 2000) {
    lastTelemetryTime = millis();

    // Always-populated fields
    telemetry.state = stateName(currentState);
    telemetry.tempC = tempC;
    telemetry.pressurePsi = pressurePsi;
    telemetry.hasWater = hasWater;

    unsigned long elapsed = millis() - stateStartTime;

    switch (currentState) {
    case STATE_IDLE:
    case STATE_COMPLETE:
      // Door unlocked
      telemetry.doorLocked = false;
      break;

    case STATE_CHECK_WATER:
    case STATE_LOCK_DOOR:
    case STATE_PURGE:
    case STATE_STERILIZING:
    case STATE_EXHAUST:
      // Door locked for the entire active cycle
      telemetry.doorLocked = true;
      break;

    case STATE_EMERGENCY_SHUTDOWN:
      // Door stays locked until pressure is safe
      telemetry.doorLocked = (pressurePsi > 1.0);
      break;
    }

    // Stage-specific fields
    switch (currentState) {
    case STATE_IDLE:
    case STATE_CHECK_WATER:
      // No stage-specific fields
      break;

    case STATE_LOCK_DOOR:
      // No stage-specific fields
      break;

    case STATE_PURGE: {
      telemetry.hasPurge = true;
      telemetry.purgePhase = purgePhase;
      telemetry.purgePhaseName = purgePhaseName(purgePhase);
      telemetry.purgePulse = purgeCurrentPulse(purgePhase);
      telemetry.vacuumTargetPsi = VACUUM_TARGET_PSI;
      telemetry.steamTargetTempC = TARGET_TEMP_C;
      break;
    }

    case STATE_STERILIZING: {
      telemetry.hasSterilizing = true;
      telemetry.pidOutput = pidOutput;
      telemetry.ssrOn = ssrOn;
      telemetry.pidSetpoint = STERILIZE_SETPOINT_C;
      telemetry.filteredTempC = filteredTempC;
      telemetry.holdRemaining =
          (elapsed < HOLD_TIME) ? (HOLD_TIME - elapsed) / 1000 : 0;
      break;
    }

    case STATE_EXHAUST:
    case STATE_COMPLETE:
    case STATE_EMERGENCY_SHUTDOWN:
      // No stage-specific fields
      break;
    }

    // Live telemetry print every 2 s so the operator isn't running blind
    printTelemetryJson();
  }
}
