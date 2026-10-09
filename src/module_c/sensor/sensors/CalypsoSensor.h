#pragma once

#include "ISensor.h"
#include <Arduino.h>

// UART, streaming NMEA 0183 (confirmed TTL-UART/NMEA variant, not RS485). The sensor
// streams unprompted - read() drains whatever's buffered and returns the last
// complete, checksum-valid $..MWV sentence seen this call; NotReady if nothing new
// arrived since the last call. Poll mode (sending a command instead of streaming) is
// supported by the hardware per the datasheet but isn't implemented here - not needed
// until the duty-cycled version, and streaming is fine for a continuously-powered bench.
//
// values[]: [0]=wind angle (deg, relative), [1]=wind speed (as reported - typically m/s
// on this unit's default NMEA field), [2]=status (1=valid/A, 0=invalid).
//
// Sentence buffer is a fixed-size char array, not String - this sensor streams
// continuously, and a String rebuilt every line on a device meant to run for days
// would fragment the heap over time.
class CalypsoSensor : public ISensor {
public:
  CalypsoSensor(HardwareSerial &serial, uint8_t rxPin, uint8_t txPin, uint32_t baud = 38400)
      : _serial(serial), _rxPin(rxPin), _txPin(txPin), _baud(baud) {}

  bool begin() override;
  Reading read() override;
  void sleep() override {}
  const char *name() const override { return "Calypso"; }

  // Drops any buffered bytes and resets the partial-line parser. Call right
  // after a light-sleep wake: the UART ISR was halted through the sleep, so
  // whatever survived in the RX FIFO is a stale, likely-truncated fragment -
  // parsing it just wastes the first read() and can desync the next one.
  void flushInput();

  // Bytes drained by the last read() call - lets the caller tell "sensor not
  // wired / silent" (0 across every cycle) from "data arriving but no valid
  // sentence yet" (>0).
  uint16_t lastReadBytes() const { return _lastReadBytes; }

  // La ultima trama MWV que paso el checksum, TAL CUAL llego.
  //
  // Hace falta porque el sensor puede emitir tramas perfectamente validas cuyo
  // contenido sea cero: eso no se distingue de un fallo de parseo mirando solo el
  // valor. Con la trama cruda delante se ve de inmediato si el cero lo dijo el
  // sensor o lo invento este driver leyendo el campo equivocado.
  const char *lastFrame() const { return _lastFrame; }

private:
  static constexpr uint8_t kLineMax = 120;

  HardwareSerial &_serial;
  uint8_t _rxPin, _txPin;
  uint32_t _baud;
  char _line[kLineMax + 1] = {0};
  uint8_t _lineLen = 0;
  uint16_t _lastReadBytes = 0;
  char _lastFrame[kLineMax + 1] = {0};

  static bool checksumOk(const char *sentence, uint8_t len, int starIdx);
};
