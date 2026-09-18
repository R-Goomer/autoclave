#include "Sensors.h"
#include "Config.h"

namespace Sensors {

namespace {
float tempFilterBuf[TEMP_FILTER_WINDOW] = {0};
int tempFilterIdx = 0;

float g_tempC = 0.0f;
float g_filteredTempC = 0.0f;
float g_pressurePsi = 0.0f;
bool g_hasWater = false;

// Moving average — smooths LM35 noise for the PID
float filterTemp(float rawC) {
  tempFilterBuf[tempFilterIdx] = rawC;
  tempFilterIdx = (tempFilterIdx + 1) % TEMP_FILTER_WINDOW;

  float sum = 0.0f;
  for (int i = 0; i < TEMP_FILTER_WINDOW; i++)
    sum += tempFilterBuf[i];
  return sum / (float)TEMP_FILTER_WINDOW;
}
} // namespace

void begin() {
  analogSetPinAttenuation(lm35Pin, ADC_11db);
  analogSetPinAttenuation(waterLevelPin, ADC_11db);
  analogSetPinAttenuation(simPressurePin, ADC_11db); // Pressure simulation pot
}

void update() {
  g_tempC = analogReadMilliVolts(lm35Pin) / 10.0;
  g_filteredTempC = filterTemp(g_tempC);

  // Pressure simulation: pot wiper on GPIO 6
  // Full CCW (0 mV)               = -2 PSI       ← deep vacuum side
  // Full CW  (3300 mV)            = +30 PSI       ← sterilizing side
  float p = -2.0 + (analogReadMilliVolts(simPressurePin) - 656.0) *
                        (27.0 / (4095.0 - 656.0));
  g_pressurePsi = constrain(p, -2.0, 25.0);

  g_hasWater = (analogRead(waterLevelPin) > waterThreshold);
}

float tempC() { return g_tempC; }
float filteredTempC() { return g_filteredTempC; }
float pressurePsi() { return g_pressurePsi; }
bool waterPresent() { return g_hasWater; }

} // namespace Sensors
