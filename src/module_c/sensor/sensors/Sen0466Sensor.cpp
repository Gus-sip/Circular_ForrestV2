#include "Sen0466Sensor.h"

bool Sen0466Sensor::begin() {
  if (!_sensor.begin()) return false;
  _sensor.changeAcquireMode(_sensor.PASSIVITY);
  delay(1000);
  _sensor.setTempCompensation(_sensor.ON);
  return true;
}

Reading Sen0466Sensor::read() {
  Reading r;
  r.status = ReadingStatus::Ok;
  r.values[0] = _sensor.readGasConcentrationPPM();
  r.values[1] = _sensor.readTempC();
  r.count = 2;
  return r;
}
