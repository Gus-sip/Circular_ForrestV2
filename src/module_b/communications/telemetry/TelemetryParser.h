#pragma once

#include "SensorSnapshot.h"

// Parses the sensor node's telemetry payload into a SensorSnapshot. Knows
// nothing about LoRa or the web server - just text in, struct out, so it can
// be unit-tested or reused independently of both.
//
// PROVISIONAL SCHEMA: the sensor node repo doesn't have a real telemetry
// encoder yet (only a range-test demo that sends a bare counter), so there is
// no ground-truth format to parse against yet. This is a placeholder chosen to
// be simple on both ends - fixed-order CSV, no header, exactly 13 fields:
//
//   temp,hum,pres,gas,pm1,pm25,pm10,co2,co,coTemp,windAngle,windSpeed,windValid
//
// All fields except windValid (literal "0" or "1") are plain decimal numbers.
// If the node side's encoding changes, update kFieldCount and the field
// assignments in the .cpp to match - both ends must agree on the exact order.
class TelemetryParser {
public:
  // Returns false (and leaves out untouched) if the payload doesn't split into
  // exactly kFieldCount comma-separated fields - rejected outright rather than
  // partially populating a snapshot from a malformed or out-of-sync payload.
  static bool parse(const char *payload, uint8_t len, SensorSnapshot &out);

private:
  static constexpr uint8_t kFieldCount = 13;
};
