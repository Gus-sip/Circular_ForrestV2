#pragma once

#include "ISensor.h"
#include <Arduino.h>

// UART, request/response protocol with a checksum (sum of all 8 response bytes,
// including the checksum byte itself, is 0 mod 256). EN held high for a warm-up
// period before the first read is attempted. Confirmed running fine at 3.3V on this
// board (an earlier unrelated project's CM1106 unit needed 5V - doesn't apply here).
//
// COMSEL (UART vs I2C mode select) is unstrapped on this board revision, but this
// part has consistently returned checksum-valid UART frames, so it's evidently
// defaulting into UART mode reliably on this specific unit.
//
// values[]: [0]=CO2 ppm.
class Cm1106Sensor : public ISensor {
public:
  // Some breakouts don't expose an EN pin at all; pass kNoEnPin (and warmupMs=0) for those.
  static constexpr uint8_t kNoEnPin = 0xFF;

  Cm1106Sensor(HardwareSerial &serial, uint8_t rxPin, uint8_t txPin, uint8_t enPin, uint32_t warmupMs,
               uint32_t baud = 9600)
      : _serial(serial), _rxPin(rxPin), _txPin(txPin), _enPin(enPin), _warmupMs(warmupMs), _baud(baud) {}

  bool begin() override;
  Reading read() override;
  void sleep() override;
  const char *name() const override { return "CM1106"; }

private:
  HardwareSerial &_serial;
  uint8_t _rxPin, _txPin, _enPin;
  uint32_t _warmupMs;
  uint32_t _baud;
};
