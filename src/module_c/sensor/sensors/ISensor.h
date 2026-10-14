#pragma once

#include "Reading.h"

// =============================================================================
// INTERFAZ COMUN DE TODOS LOS SENSORES
// =============================================================================
//
// Todos los drivers implementan estas cuatro funciones, asi que quien los usa no
// necesita saber que bus o que protocolo hay al otro lado (I2C, UART...).
class ISensor {
public:
  virtual ~ISensor() = default;

  // Arranque, una sola vez. false = no detectado o no se pudo configurar.
  virtual bool begin() = 0;

  // Nunca bloquea mas de lo que exija el propio protocolo del sensor.
  virtual Reading read() = 0;

  // Dejar el sensor en reposo. La alimentacion la sigue controlando el firmware
  // principal (los rails): esto solo le dice al sensor que deje de trabajar, no
  // le quita la corriente.
  virtual void sleep() = 0;

  virtual const char *name() const = 0;
};
