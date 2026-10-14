#pragma once

#include "ISensor.h"
#include <Arduino.h>

// =============================================================================
// CM1106 (Cubic) - SENSOR DE CO2
// =============================================================================
//
// UART, protocolo pregunta-respuesta con checksum: la suma de los 8 bytes de la
// respuesta, incluido el propio byte de checksum, es 0 modulo 256. El pin EN se
// pone a nivel alto y se espera un tiempo de calentamiento antes del primer intento
// de lectura. Cuelga del rail de 5 V conmutado.
//
// MIDE SOLO AL ENCENDERSE: despues repite ese valor el resto del tiempo. Para
// obtener un valor NUEVO hay que hacerle un ciclo de alimentacion con EN
// (powerCycle()). No es un apano de un fallo: es como funciona la pieza.
//
// COMSEL (seleccion UART/I2C) no esta fijado en esta revision de placa, pero la
// pieza ha devuelto siempre tramas UART con checksum valido, asi que en esta
// unidad arranca en modo UART de forma fiable.
//
// values[]: [0] = CO2 en ppm.
class Cm1106Sensor : public ISensor {
public:
  // Algunas placas no sacan el pin EN; para esas, pasar kNoEnPin (y warmupMs = 0).
  static constexpr uint8_t kNoEnPin = 0xFF;

  Cm1106Sensor(HardwareSerial &serial, uint8_t rxPin, uint8_t txPin, uint8_t enPin, uint32_t warmupMs,
               uint32_t baud = 9600)
      : _serial(serial), _rxPin(rxPin), _txPin(txPin), _enPin(enPin), _warmupMs(warmupMs), _baud(baud) {}

  bool begin() override;
  Reading read() override;
  void sleep() override;
  const char *name() const override { return "CM1106"; }

  // Bytes crudos de la ultima respuesta, sea cual sea su veredicto. Un valor en
  // ppm que nunca cambia puede significar dos cosas muy distintas - un sensor que
  // devuelve una trama nueva pero congelada, o un camino de lectura que reutiliza
  // en silencio un valor viejo - y solo los bytes del cable las distinguen.
  // Longitud 0 si no llego nada.
  const uint8_t *lastRaw() const { return _lastRaw; }
  uint8_t lastRawLen() const { return _lastRawLen; }

  // Tramas validas seguidas con el MISMO valor de CO2 (bytes resp[3..4]).
  //
  // A proposito NO compara la trama entera: se vio el 2026-09-04 que esta unidad
  // lleva un contador en resp[6] que sube en cada lectura (y el checksum con el),
  // asi que dos tramas nunca son identicas aunque la medida este clavada. Comparar
  // tramas enteras no detectaria nunca nada. Que el aparato este vivo y que la
  // medida este viva son dos cosas distintas: esto vigila la segunda.
  uint16_t frozenValueRun() const { return _frozenRun; }

  // Los bytes del valor de CO2 de la ultima trama valida, para el registro.
  uint16_t lastValueRaw() const { return _lastValue; }

  // Ciclo de alimentacion con EN: apagar offMs, encender y esperar el
  // calentamiento. La hoja de datos no ofrece reinicio por software para este
  // caso, y la unidad solo se ha recuperado siempre con un corte completo.
  void powerCycle(uint32_t offMs = 500);

private:
  HardwareSerial &_serial;
  uint8_t _rxPin, _txPin, _enPin;
  uint32_t _warmupMs;
  uint32_t _baud;
  uint8_t _lastRaw[8] = {0};
  uint8_t _lastRawLen = 0;
  uint8_t _prevRaw[8] = {0};
  uint8_t _prevRawLen = 0;
  uint16_t _frozenRun = 0;
  uint16_t _lastValue = 0;
  bool _haveValue = false;
};
