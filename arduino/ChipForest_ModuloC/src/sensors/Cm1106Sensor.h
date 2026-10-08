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

  // Raw bytes of the most recent response, whatever its verdict. A parsed ppm that
  // never changes can mean two very different things - a sensor returning a frozen
  // but genuinely fresh frame, or a read path quietly recycling an old value - and
  // only the wire bytes tell them apart. Length is 0 if nothing arrived.
  const uint8_t *lastRaw() const { return _lastRaw; }
  uint8_t lastRawLen() const { return _lastRawLen; }

  // Consecutive valid frames carrying the SAME CO2 value bytes (resp[3..4]).
  //
  // Deliberately NOT a whole-frame comparison: observed 2026-09-04, this unit
  // keeps a counter in resp[6] that increments every read (checksum tracking it),
  // so successive frames are never byte-identical even while the measurement
  // itself is pegged. Whole-frame equality would therefore never fire. The device
  // being alive and the measurement being live are separate facts - this tracks
  // the second one.
  uint16_t frozenValueRun() const { return _frozenRun; }

  // The CO2 value bytes behind the most recent valid frame, for logging.
  uint16_t lastValueRaw() const { return _lastValue; }

  // Power-cycle via EN. The datasheet offers no soft reset for this state, and the
  // unit has previously only recovered from a full power cycle.
  void powerCycle(uint32_t offMs = 500);

private:
  HardwareSerial &_serial;
  uint8_t _rxPin, _txPin, _enPin;
  uint32_t _warmupMs;
  uint32_t _baud;
  uint8_t _lastRaw[8] = {0};
  uint8_t _lastRawLen = 0;
  uint8_t _prevRaw[8] = {0};
  uint8_t _prevRawLen = 0;
  uint16_t _frozenRun = 0;
  uint16_t _lastValue = 0;
  bool _haveValue = false;
};
