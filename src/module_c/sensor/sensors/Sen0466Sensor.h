#pragma once

#include "ISensor.h"
#include <Wire.h>
#include <DFRobot_MultiGasSensor.h>

// I2C, shared bus. Address is board-selectable; confirmed 0x74 on this unit.
//
// values[]: [0]=CO ppm, [1]=onboard temperature C.
//
// Note: DFRobot_GAS_I2C's read calls don't surface a distinct bus-failure signal of
// their own, so a read() here always reports Ok if begin() succeeded - a genuine bus
// dropout after begin() would currently just read back as an unchanging/zero value,
// not a NoAck. Documented rather than papered over with a fabricated check.
class Sen0466Sensor : public ISensor {
public:
  explicit Sen0466Sensor(uint8_t addr) : _sensor(&Wire, addr) {}

  bool begin() override;
  Reading read() override;
  void sleep() override {}
  const char *name() const override { return "SEN0466"; }

private:
  DFRobot_GAS_I2C _sensor;
};
