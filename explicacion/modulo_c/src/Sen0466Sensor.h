#pragma once

#include "ISensor.h"
#include <Wire.h>
#include <DFRobot_MultiGasSensor.h>

// =============================================================================
// SEN0466 (DFRobot) - SENSOR DE MONOXIDO DE CARBONO (CO)
// =============================================================================
//
// Bus I2C compartido. La direccion se elige en la placa; en esta unidad es 0x74.
// Alimentacion: puerta propia en GPIO16 (PIN_SEN0466_EN, LOW = encendido).
//
// values[]: [0] = CO en ppm, [1] = temperatura interna del sensor en C.
//
// LIMITACION CONOCIDA: las funciones de lectura de la libreria DFRobot no
// informan por si mismas de un fallo del bus. Asi que read() devuelve siempre Ok
// si begin() tuvo exito: si el sensor se cae del bus despues, se veria como un
// valor que no cambia o cero, no como NoAck. Se documenta en vez de taparlo con
// una comprobacion inventada.
class Sen0466Sensor : public ISensor {
public:
  explicit Sen0466Sensor(uint8_t addr) : _sensor(&Wire, addr) {}

  bool begin() override;
  Reading read() override;
  // Ver el .cpp: hace muy poco, y lo dice alli en vez de aparentar.
  void sleep() override;
  const char *name() const override { return "SEN0466"; }

private:
  DFRobot_GAS_I2C _sensor;
};
