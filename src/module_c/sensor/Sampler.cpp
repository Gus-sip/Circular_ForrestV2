#include "Sampler.h"
#include <Arduino.h>

void Sampler::addSensor(ISensor *sensor) {
  if (_count >= SAMPLER_MAX_SENSORS) return;
  _sensors[_count++] = sensor;
}

void Sampler::beginAll() {
  for (uint8_t i = 0; i < _count; i++) {
    _ready[i] = _sensors[i]->begin();
    Serial.printf("%-8s %s\n", _sensors[i]->name(), _ready[i] ? "OK" : "NOT FOUND");
  }
}

void Sampler::printTable() {
  for (uint8_t i = 0; i < _count; i++) {
    if (!_ready[i]) continue;

    Reading r = _sensors[i]->read();
    Serial.printf("%-8s ", _sensors[i]->name());

    switch (r.status) {
      case ReadingStatus::Ok:
        for (uint8_t v = 0; v < r.count; v++) Serial.printf("%.2f ", r.values[v]);
        Serial.println();
        break;
      case ReadingStatus::NotReady:
        Serial.println("(not ready)");
        break;
      case ReadingStatus::Timeout:
        Serial.println("(timeout)");
        break;
      case ReadingStatus::NoAck:
        Serial.println("(no ack)");
        break;
      case ReadingStatus::InvalidFrame:
        Serial.println("(invalid frame)");
        break;
      default:
        Serial.println("(not initialized)");
        break;
    }
  }
}
