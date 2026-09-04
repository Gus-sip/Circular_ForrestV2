#include "Cm1106Sensor.h"

static const uint8_t READ_CO2_CMD[] = {0x11, 0x01, 0x01, 0xED};

bool Cm1106Sensor::begin() {
  if (_enPin != kNoEnPin) {
    pinMode(_enPin, OUTPUT);
    digitalWrite(_enPin, HIGH);
  }
  _serial.begin(_baud, SERIAL_8N1, _rxPin, _txPin);
  if (_warmupMs > 0) delay(_warmupMs);
  return true;  // no handshake beyond EN + baud - presence is confirmed by the first valid read
}

Reading Cm1106Sensor::read() {
  Reading r;

  while (_serial.available()) _serial.read();
  _serial.write(READ_CO2_CMD, sizeof(READ_CO2_CMD));

  uint8_t resp[8];
  uint8_t len = 0;
  uint32_t start = millis();
  while (millis() - start < 1000 && len < sizeof(resp)) {
    if (_serial.available()) resp[len++] = _serial.read();
  }

  // Record the wire bytes before any verdict is reached, so a caller can tell a
  // frozen-but-fresh frame from a recycled value regardless of how this read ends.
  memcpy(_prevRaw, _lastRaw, sizeof(_prevRaw));
  _prevRawLen = _lastRawLen;
  memcpy(_lastRaw, resp, sizeof(_lastRaw));
  _lastRawLen = len;

  if (len < 8) {
    r.status = ReadingStatus::Timeout;
    return r;
  }
  if (resp[0] != 0x16 || resp[1] != 0x05 || resp[2] != 0x01) {
    r.status = ReadingStatus::InvalidFrame;
    return r;
  }

  uint16_t sum = 0;
  for (uint8_t i = 0; i < 7; i++) sum += resp[i];
  uint8_t checksum = (uint8_t)(256 - (sum % 256));
  if (checksum != resp[7]) {
    r.status = ReadingStatus::InvalidFrame;
    return r;
  }

  r.status = ReadingStatus::Ok;
  const uint16_t value = (uint16_t)resp[3] * 256 + resp[4];

  // Track staleness of the MEASUREMENT, not of the frame - see frozenValueRun().
  if (_haveValue && value == _lastValue) {
    if (_frozenRun < 0xFFFF) _frozenRun++;
  } else {
    _frozenRun = 0;
  }
  _lastValue = value;
  _haveValue = true;

  r.values[0] = value;
  r.count = 1;
  return r;
}

void Cm1106Sensor::sleep() {
  if (_enPin != kNoEnPin) digitalWrite(_enPin, LOW);
}

void Cm1106Sensor::powerCycle(uint32_t offMs) {
  if (_enPin == kNoEnPin) return;
  digitalWrite(_enPin, LOW);
  delay(offMs);
  digitalWrite(_enPin, HIGH);
  if (_warmupMs > 0) delay(_warmupMs);
  // Frames from before the cycle say nothing about the sensor after it.
  _frozenRun = 0;
  _lastRawLen = 0;
  _prevRawLen = 0;
  _haveValue = false;
  while (_serial.available()) _serial.read();
}
