#pragma once

#include <stdint.h>

// A failed sensor and a sensor that measured zero must be distinguishable at the
// call site - hence an explicit status rather than a sentinel float.
enum class ReadingStatus : uint8_t {
  NotInitialized,  // begin() never succeeded
  NotReady,        // sensor is fine but has nothing new this cycle
  Timeout,         // request sent, no (or incomplete) response in time
  NoAck,           // bus-level failure - device didn't respond at all
  InvalidFrame,    // response received but failed checksum/format validation
  Ok
};

// values[]/count meaning is per-sensor; documented in each driver's header.
struct Reading {
  ReadingStatus status = ReadingStatus::NotInitialized;
  float values[4] = {0, 0, 0, 0};
  uint8_t count = 0;

  bool ok() const { return status == ReadingStatus::Ok; }
};
