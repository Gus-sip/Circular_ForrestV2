#include "TelemetryParser.h"
#include <stdlib.h>
#include <string.h>

// Splits a single comma-separated record into exactly kFieldCount fields.
//
// Returns false if there are too few OR TOO MANY - the "too many" case is the one
// that matters. The previous version stopped scanning once it had 13 fields and
// then checked `fieldIdx != kFieldCount`, which by construction could never fail.
// Anything with MORE than 13 fields was silently accepted using only its first 13,
// so a batch payload "B,2,<record>;<record>;" parsed as a single reading with the
// header shifted into the data:
//
//   "B"   -> temp = 0.0      (atof of a letter)
//   "2"   -> rh   = 2.0      (the batch count)
//   42.4  -> pres            (actually the temperature)
//   11.3  -> gas             (actually the humidity)
//
// which is exactly the corruption seen in ThingsBoard - every field displaced by
// two, with 0.0 and 2.0 appearing from nowhere. The second reading of each batch
// was discarded entirely.
static bool splitRecord(char *buf, uint8_t len, char **fields, uint8_t want) {
  uint8_t fieldIdx = 0;
  fields[fieldIdx++] = buf;

  for (uint8_t i = 0; i < len; i++) {
    if (buf[i] == ',') {
      if (fieldIdx >= want) return false;  // MORE fields than expected - not this shape
      buf[i] = '\0';
      fields[fieldIdx++] = buf + i + 1;
    }
  }
  return fieldIdx == want;
}

static void fillSnapshot(char **fields, SensorSnapshot &out) {
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
}

bool TelemetryParser::parse(const char *payload, uint8_t len, SensorSnapshot &out) {
  if (len == 0 || len > 200) return false;

  // A batch is a different shape and must not be squeezed into this one - see
  // parseBatch(). Rejecting it here is what stops the field-shift corruption.
  if (len >= 2 && payload[0] == 'B' && payload[1] == ',') return false;

  char buf[241];
  memcpy(buf, payload, len);
  buf[len] = '\0';

  char *fields[kFieldCount] = {};
  if (!splitRecord(buf, len, fields, kFieldCount)) return false;

  fillSnapshot(fields, out);
  return true;
}

uint8_t TelemetryParser::parseBatch(const char *payload, uint8_t len, SensorSnapshot *out,
                                    uint8_t maxOut, uint16_t *ageTicksOut) {
  if (len < 4 || len > 240) return 0;
  if (!(payload[0] == 'B' && payload[1] == ',')) return 0;

  char buf[241];
  memcpy(buf, payload, len);
  buf[len] = '\0';

  // Skip the "B,<count>," header. The count is deliberately NOT trusted as the
  // number of records - the records themselves are counted while parsing, so a
  // truncated or malformed packet yields the records that are actually present
  // rather than a promised number that isn't.
  char *p = strchr(buf, ',');
  if (!p) return 0;
  p = strchr(p + 1, ',');
  if (!p) return 0;
  p++;

  uint8_t n = 0;
  while (*p && n < maxOut) {
    char *end = strchr(p, ';');
    if (end) *end = '\0';

    uint8_t recLen = (uint8_t)strlen(p);
    if (recLen > 0) {
      // A batch record carries ONE extra trailing field beyond a plain reading:
      // its age in ticks at the moment of transmit.
      char *fields[kBatchFieldCount] = {};
      if (splitRecord(p, recLen, fields, kBatchFieldCount)) {
        fillSnapshot(fields, out[n]);
        if (ageTicksOut) ageTicksOut[n] = (uint16_t)atoi(fields[13]);
        n++;
      }
    }

    if (!end) break;
    p = end + 1;
  }

  return n;
}
