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

  // Which address begin() settled on. It tries the configured one then the
  // alternate, so the caller cannot otherwise know which is live - and that
  // matters when a sensor reports OK at init but returns nothing afterwards.
  uint8_t address() const { return _addr; }
  void sleep() override {}
  const char *name() const override { return "BME690"; }

  // Raw value of the chip-ID register (0xD0) read directly when begin() fails,
  // bypassing the Bosch driver. begin() collapses five different failures into one
  // `false`; this is what distinguishes them. Expect 0x61 on a healthy part.
  // Only meaningful when chipIdValid() is true.
  uint8_t lastChipId() const { return _chipId; }
  bool chipIdValid() const { return _chipIdValid; }
  // Which access pattern answered: 0 = repeated start (what the Bosch driver
  // uses), 1 = stop-separated, -1 = neither.
  int chipIdMode() const { return _chipIdMode; }

private:
  void primeAndMark();

  uint8_t _addr;
  bool _primed = false;  // a discarded first measurement has been taken - see the .cpp
  uint8_t _chipId = 0;
  bool _chipIdValid = false;
  int _chipIdMode = -1;
  BME69X_7Semi _sensor;
};
