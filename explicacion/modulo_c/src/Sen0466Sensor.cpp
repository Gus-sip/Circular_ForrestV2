#include "Sen0466Sensor.h"

// Arranca el sensor en modo PASIVO (solo mide cuando se le pregunta, que es el
// modo de menor consumo que ofrece) y activa la compensacion por temperatura.
bool Sen0466Sensor::begin() {
  if (!_sensor.begin()) return false;
  _sensor.changeAcquireMode(_sensor.PASSIVITY);
  delay(1000);
  _sensor.setTempCompensation(_sensor.ON);
  return true;
}

// Pide la concentracion de CO y la temperatura interna.
Reading Sen0466Sensor::read() {
  Reading r;
  r.status = ReadingStatus::Ok;
  r.values[0] = _sensor.readGasConcentrationPPM();
  r.values[1] = _sensor.readTempC();
  r.count = 2;
  return r;
}

// NOTA HONESTA: esto hace muy poco, y es la verdad, no un descuido. La libreria
// de DFRobot no ofrece ningun comando de reposo para esta pieza: PASIVO es el modo
// de menor consumo que tiene, y begin() ya lo selecciona, asi que lo unico que
// puede hacer esto es volver a fijarlo por si algo lo cambio.
//
// El ahorro de verdad es quitarle la alimentacion (su puerta en GPIO16). Esta
// funcion existe para que el apagado sea ordenado y para que "dormir el SEN0466"
// no sea, sin decirlo, una funcion vacia - que es lo que era antes.
void Sen0466Sensor::sleep() {
  _sensor.changeAcquireMode(_sensor.PASSIVITY);
}
