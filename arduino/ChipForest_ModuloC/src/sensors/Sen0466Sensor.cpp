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

// HONEST NOTE: this does very little, and that is the truth rather than an
// oversight. The DFRobot library exposes no sleep or standby command for this
// part - PASSIVITY is the lowest-power mode it offers, and begin() already selects
// it, so all this can do is re-assert it in case something changed the mode.
//
// The real saving on the way into hibernation is the 3V3 rail being switched off,
// which removes the sensor's supply entirely. This call exists so the shutdown is
// orderly and so "tell the SEN0466 to sleep" is not silently a no-op, which is
// what it was before.
void Sen0466Sensor::sleep() {
  _sensor.changeAcquireMode(_sensor.PASSIVITY);
}
