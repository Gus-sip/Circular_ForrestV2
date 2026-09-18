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

  // Full read: temperature, humidity, pressure AND gas resistance. Runs the gas
  // heater at 320C, which takes ~10.8s per notes/power_budget.md and must not be
  // truncated. For the periodic burst, not for frequent sampling.
  Reading read() override;

  // Fast read: temperature, humidity and pressure only, with the GAS HEATER OFF.
  //
  // This is what makes a 10s sentinel cadence affordable. The heater is the
  // expensive part of a BME690 measurement by an order of magnitude - without it
  // a forced measurement is a couple of hundred milliseconds instead of 10.8
  // seconds, and the current drops from ~3.1mA to negligible.
  //
  // values[]: [0]=temperature, [1]=humidity, [2]=pressure, [3]=0 (no gas reading).
  // count is 3, so a caller cannot mistake the absent gas value for a real one.
  Reading readFast();

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
  // Switches the gas heater on or off, remembering the current state so the
  // Bosch config write is only issued when it actually changes - it is an I2C
  // transaction, and the sentinel path runs every 10 seconds.
  bool setHeater(bool on);

  uint8_t _addr;
  bool _primed = false;  // a discarded first measurement has been taken - see the .cpp
  uint8_t _chipId = 0;
  bool _chipIdValid = false;
  int _chipIdMode = -1;
  bool _heaterOn = true;  // begin() leaves the library's default profile enabled
  BME69X_7Semi _sensor;
};
