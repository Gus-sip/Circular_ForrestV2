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
  r.values[0] = resp[3] * 256 + resp[4];
  r.count = 1;
  return r;
}

void Cm1106Sensor::sleep() {
  if (_enPin != kNoEnPin) digitalWrite(_enPin, LOW);
}
