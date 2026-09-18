#include "Outputs.h"
#include "Config.h"

namespace Outputs {

void begin() {
  int relayPins[] = {RELAY_DRAIN, RELAY_EXHAUST, RELAY_AIR_VALVE, RELAY_HEATER,
                      RELAY_DOOR_LOCK};

  for (int pin : relayPins) {
    digitalWrite(pin, RELAY_OFF); // Safe state before setting direction
    pinMode(pin, OUTPUT);
  }
}

void allOff() {
  // RELAY_STEAM_INLET is -1 (removed) — do not write
  digitalWrite(RELAY_DRAIN, RELAY_OFF);
  digitalWrite(RELAY_EXHAUST, RELAY_OFF);
  digitalWrite(RELAY_AIR_VALVE, RELAY_OFF);
  digitalWrite(RELAY_HEATER, RELAY_OFF);
  digitalWrite(RELAY_DOOR_LOCK, RELAY_OFF);
}

void heater(bool on) { digitalWrite(RELAY_HEATER, on ? RELAY_ON : RELAY_OFF); }
void drain(bool on) { digitalWrite(RELAY_DRAIN, on ? RELAY_ON : RELAY_OFF); }
void exhaust(bool on) { digitalWrite(RELAY_EXHAUST, on ? RELAY_ON : RELAY_OFF); }
void airValve(bool on) { digitalWrite(RELAY_AIR_VALVE, on ? RELAY_ON : RELAY_OFF); }
void doorLock(bool on) { digitalWrite(RELAY_DOOR_LOCK, on ? RELAY_ON : RELAY_OFF); }

} // namespace Outputs
