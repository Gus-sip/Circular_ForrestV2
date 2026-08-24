#include "Bme690Sensor.h"

bool Bme690Sensor::begin() {
  if (_sensor.begin(_addr)) return true;

  uint8_t altAddr = (_addr == 0x76) ? 0x77 : 0x76;
  if (_sensor.begin(altAddr)) {
    _addr = altAddr;
    return true;
  }
  return false;
}

Reading Bme690Sensor::read() {
  Reading r;
  float temperature, humidity, pressure, gas;
  if (_sensor.getData(temperature, humidity, pressure, gas)) {
    r.status = ReadingStatus::Ok;
    r.values[0] = temperature;
    r.values[1] = humidity;
    r.values[2] = pressure;
    r.values[3] = gas;
    r.count = 4;
  } else {
    r.status = ReadingStatus::Timeout;
  }
  return r;
}
