#include "PowerManager.h"
#include <Arduino.h>

void PowerManager::enable3V3Sensors() {
  if (_sensors3V3) return;
  Serial.println("[PowerManager] enable3V3Sensors (stub - bench has no gating FET yet)");
  _sensors3V3 = true;
}

void PowerManager::disable3V3Sensors() {
  if (!_sensors3V3) return;
  Serial.println("[PowerManager] disable3V3Sensors (stub - bench has no gating FET yet)");
  _sensors3V3 = false;
}
