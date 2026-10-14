#pragma once

#include "ISensor.h"
#include <Arduino.h>

// =============================================================================
// CALYPSO ULP PRO - ANEMOMETRO ULTRASONICO (VIENTO)
// =============================================================================
//
// UART, frases NMEA 0183 (variante TTL-UART/NMEA confirmada, no RS485). Necesita
// 5 V y unos 20 s desde que recibe corriente hasta que empieza a emitir.
//
// El sensor emite SOLO, sin que se le pregunte. read() vacia lo que haya en el
// buffer y devuelve la ultima frase $..MWV completa y con checksum valido vista en
// esa llamada; NotReady si no llego nada nuevo desde la anterior. El hardware
// admite tambien modo pregunta-respuesta segun la hoja de datos, pero no esta
// implementado: no hace falta.
//
// values[]: [0] = angulo del viento (grados, relativo), [1] = velocidad (tal como
// la da el sensor; normalmente m/s en el campo NMEA por defecto de esta unidad),
// [2] = estado (1 = valido/A, 0 = no valido).
//
// La frase se guarda en un array de char de tamano fijo, no en un String: el
// sensor emite sin parar, y un String reconstruido en cada linea en un aparato que
// debe funcionar dias enteros acabaria fragmentando la memoria.
class CalypsoSensor : public ISensor {
public:
  CalypsoSensor(HardwareSerial &serial, uint8_t rxPin, uint8_t txPin, uint32_t baud = 38400)
      : _serial(serial), _rxPin(rxPin), _txPin(txPin), _baud(baud) {}

  bool begin() override;
  Reading read() override;
  void sleep() override {}
  const char *name() const override { return "Calypso"; }

  // Descarta los bytes del buffer y reinicia la linea a medio construir. Se llama
  // justo antes de escuchar: lo que hubiera en el buffer es un fragmento viejo y
  // seguramente cortado, y procesarlo solo desperdicia la primera lectura y puede
  // descolocar la siguiente.
  void flushInput();

  // Bytes leidos en la ultima llamada a read(). Distingue "sensor sin conectar o
  // mudo" (0 en todos los ciclos) de "llegan datos pero aun ninguna frase valida"
  // (>0).
  uint16_t lastReadBytes() const { return _lastReadBytes; }

  // La ultima trama MWV que paso el checksum, TAL CUAL llego.
  //
  // Hace falta porque el sensor puede emitir tramas perfectamente validas cuyo
  // contenido sea cero: eso no se distingue de un fallo de parseo mirando solo el
  // valor. Con la trama cruda delante se ve de inmediato si el cero lo dijo el
  // sensor o lo invento este driver leyendo el campo equivocado.
  const char *lastFrame() const { return _lastFrame; }

private:
  static constexpr uint8_t kLineMax = 120;

  HardwareSerial &_serial;
  uint8_t _rxPin, _txPin;
  uint32_t _baud;
  char _line[kLineMax + 1] = {0};
  uint8_t _lineLen = 0;
  uint16_t _lastReadBytes = 0;
  char _lastFrame[kLineMax + 1] = {0};

  static bool checksumOk(const char *sentence, uint8_t len, int starIdx);
};
