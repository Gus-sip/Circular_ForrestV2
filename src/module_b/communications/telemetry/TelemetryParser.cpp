#include "TelemetryParser.h"
#include <stdlib.h>
#include <string.h>

bool TelemetryParser::parse(const char *payload, uint8_t len, SensorSnapshot &out) {
  if (len == 0 || len > 200) return false;

  char buf[241];
  memcpy(buf, payload, len);
  buf[len] = '\0';

  char *fields[kFieldCount] = {};
  uint8_t fieldIdx = 0;
  fields[fieldIdx++] = buf;

  for (uint8_t i = 0; i < len && fieldIdx < kFieldCount; i++) {
    if (buf[i] == ',') {
      buf[i] = '\0';
      fields[fieldIdx++] = buf + i + 1;
    }
  }

  if (fieldIdx != kFieldCount) return false;  // wrong shape - reject rather than guess

  out.temp = atof(fields[0]);
  out.hum = atof(fields[1]);
  out.pres = atof(fields[2]);
  out.gas = atof(fields[3]);
  out.pm1 = atof(fields[4]);
  out.pm25 = atof(fields[5]);
  out.pm10 = atof(fields[6]);
  out.co2 = atof(fields[7]);
  out.co = atof(fields[8]);
  out.coTemp = atof(fields[9]);
  out.windAngle = atof(fields[10]);
  out.windSpeed = atof(fields[11]);
  out.windValid = (fields[12][0] == '1');

  return true;
}
