#pragma once

#include "Reading.h"

// Common interface so the Sampler doesn't care what bus or protocol is on the other
// end of any given sensor.
class ISensor {
public:
  virtual ~ISensor() = default;

  // One-shot init. False = not detected / failed to configure.
  virtual bool begin() = 0;

  // Never blocks longer than the sensor's own protocol requires.
  virtual Reading read() = 0;

  // Quiesce the part. The PowerManager still owns the rail - this just tells the
  // sensor to stop doing work, it doesn't cut power.
  virtual void sleep() = 0;

  virtual const char *name() const = 0;
};
