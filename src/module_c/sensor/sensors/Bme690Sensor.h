#pragma once

#include "ISensor.h"
#include <7Semi_BME690.h>

// I2C, shared bus (SDA/SCL from pins.h). SDO floats on this board revision, so the
// address isn't fixed in hardware - primaryAddr is tried first, then the other of
// {0x76, 0x77} as a fallback, since either is a legitimate strap state.
//
// values[]: [0]=temperature C, [1]=humidity %, [2]=pressure hPa, [3]=gas resistance ohm.
class Bme690Sensor : public ISensor {
public:
  explicit Bme690Sensor(uint8_t primaryAddr) : _addr(primaryAddr) {}

  bool begin() override;
  Reading read() override;
  void sleep() override {}
  const char *name() const override { return "BME690"; }

private:
  uint8_t _addr;
  BME69X_7Semi _sensor;
};
