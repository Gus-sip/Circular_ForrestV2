#pragma once

#include "ISensor.h"
#include <7Semi_BME690.h>

// =============================================================================
// BME690 (Bosch) - TEMPERATURA, HUMEDAD, PRESION Y GAS (VOC)
// =============================================================================
//
// Bus I2C compartido (SDA/SCL de pins.h). En esta revision de placa el pin SDO
// queda al aire, asi que la direccion no esta fijada en hardware: se prueba
// primero primaryAddr y despues la otra de {0x76, 0x77}, porque las dos son
// estados legitimos.
//
// values[]: [0] = temperatura C, [1] = humedad %, [2] = presion hPa,
//           [3] = resistencia de gas en ohmios.
class Bme690Sensor : public ISensor {
public:
  explicit Bme690Sensor(uint8_t primaryAddr) : _addr(primaryAddr) {}

  bool begin() override;

  // Lectura COMPLETA: temperatura, humedad, presion Y resistencia de gas. Enciende
  // el calefactor de gas a 320 C. Para el trabajo de gas (VOC) del calendario, no
  // para lecturas frecuentes.
  // (Una hoja de calculo antigua daba ~10,8 s para esta lectura; medido en el banco
  // el 2026-10-06, la llamada vuelve en ~37 ms porque el calefactor trabaja dentro
  // del sensor y no bloquea al ESP32.)
  Reading read() override;

  // Lectura RAPIDA: solo temperatura, humedad y presion, con el calefactor de gas
  // APAGADO. Es la del trabajo de T/H/P, el mas frecuente del calendario. El
  // calefactor es, con diferencia, la parte cara de una medida del BME690: sin el,
  // el consumo baja de ~3,1 mA a casi nada.
  //
  // values[]: [0] = temperatura, [1] = humedad, [2] = presion, [3] = 0 (sin gas).
  // count vale 3, para que nadie confunda el gas ausente con uno real.
  Reading readFast();

  // En que direccion se quedo begin(). Prueba la configurada y luego la otra, asi
  // que quien llama no puede saber de otro modo cual esta viva - y eso importa
  // cuando un sensor arranca bien pero despues no devuelve nada.
  uint8_t address() const { return _addr; }
  // Antes era una funcion vacia, y "dormir el sensor" no hacia nada. El calefactor
  // ES el consumo del BME690 (el resto es despreciable), asi que dormirlo es
  // apagar el calefactor y dejarlo apagado.
  void sleep() override;
  const char *name() const override { return "BME690"; }

  // Valor crudo del registro de identificacion del chip (0xD0), leido directamente
  // cuando begin() falla, sin pasar por el driver de Bosch. begin() resume cinco
  // fallos distintos en un solo `false`; esto es lo que los distingue. En una pieza
  // sana vale 0x61. Solo tiene sentido si chipIdValid() es true.
  uint8_t lastChipId() const { return _chipId; }
  bool chipIdValid() const { return _chipIdValid; }
  // Que forma de acceso contesto: 0 = inicio repetido (la que usa el driver de
  // Bosch), 1 = separada por STOP, -1 = ninguna.
  int chipIdMode() const { return _chipIdMode; }

private:
  void primeAndMark();
  // Enciende o apaga el calefactor de gas, recordando el estado actual para que la
  // escritura de configuracion (una transaccion I2C) solo se haga cuando cambia de
  // verdad.
  bool setHeater(bool on);

  uint8_t _addr;
  bool _primed = false;  // historico: ver primeAndMark() en el .cpp
  uint8_t _chipId = 0;
  bool _chipIdValid = false;
  int _chipIdMode = -1;
  bool _heaterOn = true;  // begin() deja activo el perfil por defecto de la libreria
  BME69X_7Semi _sensor;
};
