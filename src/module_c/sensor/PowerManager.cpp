#include "PowerManager.h"
#include <Arduino.h>

// Solo apunta e imprime: no toca ningun pin. Ver la nota en PowerManager.h.
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
