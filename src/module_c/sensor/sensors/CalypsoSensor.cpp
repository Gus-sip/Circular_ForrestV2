#include "CalypsoSensor.h"
#include <stdlib.h>
#include <string.h>

bool CalypsoSensor::begin() {
  _serial.begin(_baud, SERIAL_8N1, _rxPin, _txPin);
  return true;  // no handshake beyond opening the UART - presence confirmed by the first valid sentence
}

bool CalypsoSensor::checksumOk(const char *sentence, uint8_t len, int starIdx) {
  if (starIdx < 0 || len < starIdx + 3) return false;

  uint8_t calc = 0;
  for (int i = 1; i < starIdx; i++) calc ^= sentence[i];

  char hex[3] = {sentence[starIdx + 1], sentence[starIdx + 2], '\0'};
  uint8_t received = (uint8_t)strtol(hex, nullptr, 16);
  return calc == received;
}

Reading CalypsoSensor::read() {
  Reading r;
  r.status = ReadingStatus::NotReady;

  while (_serial.available()) {
    char c = _serial.read();

    if (c == '\n') {
      if (_lineLen > 0 && _line[0] == '$' && _lineLen > 5 && memcmp(_line + 3, "MWV", 3) == 0) {
        int starIdx = -1;
        for (uint8_t i = 0; i < _lineLen; i++) {
          if (_line[i] == '*') {
            starIdx = i;
            break;
          }
        }

        if (checksumOk(_line, _lineLen, starIdx)) {
          char *fields[6] = {};
          uint8_t fieldIdx = 0;
          fields[fieldIdx++] = _line;
          for (uint8_t i = 0; i < _lineLen && fieldIdx < 6; i++) {
            if (_line[i] == ',' || _line[i] == '*') {
              _line[i] = '\0';
              fields[fieldIdx++] = _line + i + 1;
            }
          }

          if (fieldIdx >= 6) {
            r.status = ReadingStatus::Ok;
            r.values[0] = atof(fields[1]);
            r.values[1] = atof(fields[3]);
            r.values[2] = (fields[5][0] == 'A') ? 1.0f : 0.0f;
            r.count = 3;
          } else {
            r.status = ReadingStatus::InvalidFrame;
          }
        }
      }
      _lineLen = 0;
    } else if (c != '\r') {
      if (_lineLen < kLineMax) {
        _line[_lineLen++] = c;
        _line[_lineLen] = '\0';
      } else {
        _lineLen = 0;  // guard against garbage
      }
    }
  }
  return r;
}
