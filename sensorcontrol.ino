#include <Adafruit_BMP085.h>
#include <Wire.h>

// --- Input Pins ---
const int btnStartPin = 14;
const int btnResetPin = 13;

// --- Sensor Pins ---
const int lm35Pin = 5;
const int waterLevelPin = 4;
const int waterThreshold = 1850; // Above 1850 = Water detected

// --- Relay Pins (Top to Bottom) ---
// NOTE: RELAY_STEAM_INLET is PHYSICALLY REMOVED from the design.
//       The pin is driven LOW at startup and never energised again.
//       Heater alone generates all steam and pressure.
const int RELAY_STEAM_INLET = 6; // UNUSED — kept for pin mapping only
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

// --- Sterilizing Heater Hysteresis Band ---
// Heater turns ON  when temp falls TO or BELOW STERILIZE_HEATER_SOFT_LIMIT_C
// (target + 2 °C → 123 °C) Heater turns OFF when temp rises TO or ABOVE
// STERILIZE_HEATER_HARD_OFF_C   (target + 4 °C → 125 °C) FLOOR:  if temp drops
// BELOW TARGET_TEMP_C  OR  PSI drops BELOW TARGET_PSI  → EMERGENCY SHUTDOWN
const float STERILIZE_HEATER_SOFT_LIMIT_C =
    TARGET_TEMP_C + 2.0f; // 123 °C — heater turns ON  at or below here
const float STERILIZE_HEATER_HARD_OFF_C =
    TARGET_TEMP_C + 4.0f; // 125 °C — heater turns OFF at or above here

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
float baselinePressurePa =
    101325.0; // Ambient baseline set on every START press

// --- Purge Sub-State ---
// purgePhase  0,2,4  = vacuum pull  (even)
// purgePhase  1,3,5  = steam inject (odd)
// purgePhase  6      = await target temp + PSI after 3rd cycle
int purgePhase = 0;
unsigned long purgePhaseStart = 0;

// --- Sterilizing Heater State (hysteresis latch) ---
bool sterilizeHeaterOn = false;

Adafruit_BMP085 bmp;

// =============================================================================
// HELPERS
// =============================================================================

void allRelaysOff() {
  digitalWrite(RELAY_STEAM_INLET,
               RELAY_OFF); // Always OFF — steam inlet removed
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
// SETUP
// =============================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  int relayPins[] = {RELAY_STEAM_INLET, RELAY_DRAIN,  RELAY_EXHAUST,
                     RELAY_AIR_VALVE,   RELAY_HEATER, RELAY_DOOR_LOCK};

  for (int pin : relayPins) {
    digitalWrite(pin, RELAY_OFF); // Safe state before setting direction
    pinMode(pin, OUTPUT);
  }

  pinMode(btnStartPin, INPUT_PULLUP);
  pinMode(btnResetPin, INPUT_PULLUP);

  analogSetPinAttenuation(lm35Pin, ADC_11db);
  analogSetPinAttenuation(waterLevelPin, ADC_11db);

  Wire.begin(8, 9);
  if (!bmp.begin()) {
    Serial.println("CRITICAL: BMP180 sensor initialization failed!");
    currentState = STATE_EMERGENCY_SHUTDOWN;
  }

  Serial.println("==============================================");
  Serial.println(" Class B Autoclave Controller — Heater-Only  ");
  Serial.println(" Steam inlet valve DISABLED (physically removed)");
  Serial.print(" Sterilizing target: ");
  Serial.print(TARGET_TEMP_C, 1);
  Serial.print("C / ");
  Serial.print(TARGET_PSI, 1);
  Serial.println(" PSI");
  Serial.print(" Heater band: ");
  Serial.print(STERILIZE_HEATER_SOFT_LIMIT_C, 1);
  Serial.print("C ON → ");
  Serial.print(STERILIZE_HEATER_HARD_OFF_C, 1);
  Serial.println("C OFF");
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
  float currentPressurePa = bmp.readPressure();
  float pressurePsi = (currentPressurePa - baselinePressurePa) / 6894.76;
  if (pressurePsi < 0.0)
    pressurePsi = 0.0; // Clamp minor ambient drift
  bool hasWater = (analogRead(waterLevelPin) > waterThreshold);

  // ===========================================================================
  // GLOBAL SAFETY CHECKS  (run every loop cycle, regardless of state)
  // ===========================================================================

  // 1. OVER-PRESSURE INTERLOCK
  if (pressurePsi >= MAX_SAFE_PSI) {
    Serial.println(
        "EMERGENCY: OVER-PRESSURE! Opening exhaust & killing heater.");
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
      Serial.println("Start pressed — sampling atmospheric baseline...");

      // 2-second averaged baseline (20 × 100 ms)
      long sum = 0;
      for (int i = 0; i < 20; i++) {
        sum += bmp.readPressure();
        delay(100);
      }
      baselinePressurePa = sum / 20.0;

      Serial.print("Atmospheric baseline set: ");
      Serial.print(baselinePressurePa / 6894.76, 2);
      Serial.println(" psia  (0.00 psig)");
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
      Serial.print("s  |  Steam inject: ");
      Serial.print(STEAM_INJECT_MS / 1000);
      Serial.println("s  |  3 pulses");

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

        // Safe entry into sterilizing: heater OFF, reset hysteresis latch
        digitalWrite(RELAY_HEATER, RELAY_OFF);
        sterilizeHeaterOn = false;
        stateStartTime = millis();
        currentState = STATE_STERILIZING;
      }
    }
    break;
  }

  // --------------------------------------------------------------------------
  // STERILIZING
  //
  //  Hysteresis band:
  //    Heater ON   when tempC <=  STERILIZE_HEATER_SOFT_LIMIT_C  (123 °C)
  //    Heater OFF  when tempC >=  STERILIZE_HEATER_HARD_OFF_C    (125 °C)
  //    Between 123–125 °C: maintain current state (hysteresis — no hunting)
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
      sterilizeHeaterOn = false;
      currentState = STATE_EMERGENCY_SHUTDOWN;
      break;
    }

    // 2. Hysteresis-band heater control
    if (sterilizeHeaterOn) {
      // Currently heating — turn OFF only when hard-off limit is reached
      if (tempC >= STERILIZE_HEATER_HARD_OFF_C) {
        sterilizeHeaterOn = false;
        Serial.print("Sterilize: heater OFF — reached ");
        Serial.print(STERILIZE_HEATER_HARD_OFF_C, 1);
        Serial.println("C hard limit");
      }
    } else {
      // Currently coasting — turn ON when temp drops to soft limit or below
      if (tempC <= STERILIZE_HEATER_SOFT_LIMIT_C) {
        sterilizeHeaterOn = true;
        Serial.print("Sterilize: heater ON — temp at/below ");
        Serial.print(STERILIZE_HEATER_SOFT_LIMIT_C, 1);
        Serial.println("C soft limit");
      }
    }
    digitalWrite(RELAY_HEATER, sterilizeHeaterOn ? RELAY_ON : RELAY_OFF);

    // 3. Hold-time complete → exhaust
    if (millis() - stateStartTime >= HOLD_TIME) {
      Serial.println("Sterilizing hold time complete. Opening exhaust.");
      digitalWrite(RELAY_HEATER, RELAY_OFF);
      sterilizeHeaterOn = false;
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
      sterilizeHeaterOn = false;
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
      Serial.print(sterilizeHeaterOn ? "ON " : "OFF");
      Serial.print("  band:");
      Serial.print(STERILIZE_HEATER_SOFT_LIMIT_C, 0);
      Serial.print("–");
      Serial.print(STERILIZE_HEATER_HARD_OFF_C, 0);
      Serial.print("C");
      Serial.print("  floor:");
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
