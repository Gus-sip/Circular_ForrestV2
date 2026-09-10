#include "Bme690Sensor.h"

// The first forced measurement after begin() ALWAYS fails on this part.
//
// Measured on node C2: read 1 -> Timeout, read 2 -> OK (gas 41451), read 3 -> OK,
// on every single boot. The library's getData() triggers a forced measurement,
// waits the calculated duration, then treats any non-OK status from
// bme69x_get_data() as failure - and Bosch warnings such as NO_NEW_DATA are
// POSITIVE status codes, so a first-measurement warning is indistinguishable from
// a hard error there. The gas heater also needs one run before its reading is
// stable, which is why the first measurement is the one that warns.
//
// This cost one reading in a hundred while the node sampled continuously, so it
// went unnoticed. Under the sleep cycle it is fatal: every wake is a fresh begin()
// followed by exactly ONE read, so the only read taken is the one that always
// fails, and the BME690 would never contribute a single value to a packet.
//
// Hence a discarded priming measurement here, and one retry in read().

bool Bme690Sensor::begin() {
  bool up = _sensor.begin(_addr);
  if (!up) {
    uint8_t altAddr = (_addr == 0x76) ? 0x77 : 0x76;
    if (_sensor.begin(altAddr)) {
      _addr = altAddr;
      up = true;
    }
  }
  if (!up) return false;

  // Priming measurement, deliberately discarded. Costs ~200ms, and on the sleep
  // path it lands inside the BMV080's 5s startup wait anyway, so it is free.
  float t, h, pr, g;
  _sensor.getData(t, h, pr, g);
  _primed = true;
  return true;
}

Reading Bme690Sensor::read() {
  Reading r;
  float temperature, humidity, pressure, gas;

  // One retry. The priming read in begin() handles the cold case; this covers a
  // measurement that warns mid-run, which would otherwise discard a whole wake's
  // environmental data. Two failures in a row is a real fault worth reporting.
  for (int attempt = 0; attempt < 2; attempt++) {
    if (_sensor.getData(temperature, humidity, pressure, gas)) {
      r.status = ReadingStatus::Ok;
      r.values[0] = temperature;
      r.values[1] = humidity;
      r.values[2] = pressure;
      r.values[3] = gas;
      r.count = 4;
      return r;
    }
    delay(20);
  }

  r.status = ReadingStatus::Timeout;
  return r;
}
