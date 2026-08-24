#pragma once

#include "sensors/ISensor.h"

#define SAMPLER_MAX_SENSORS 8

// Walks every registered sensor and prints a readable table over USB serial. One
// unresponsive device never stalls the others - each sensor's read() bounds its own
// worst-case time, and a NotInitialized sensor is simply skipped every cycle.
class Sampler {
public:
  void addSensor(ISensor *sensor);
  void beginAll();
  void printTable();

private:
  ISensor *_sensors[SAMPLER_MAX_SENSORS] = {};
  bool _ready[SAMPLER_MAX_SENSORS] = {};
  uint8_t _count = 0;
};
