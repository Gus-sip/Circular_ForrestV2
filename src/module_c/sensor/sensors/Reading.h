#pragma once

#include <stdint.h>

// =============================================================================
// RESULTADO DE UNA LECTURA DE SENSOR
// =============================================================================
//
// Un sensor que FALLA y un sensor que MIDE CERO tienen que poder distinguirse en
// quien los llama. Por eso cada lectura lleva un estado explicito, en vez de un
// valor "especial" escondido en el numero.
enum class ReadingStatus : uint8_t {
  NotInitialized,  // begin() nunca tuvo exito
  NotReady,        // el sensor esta bien pero no tiene nada nuevo en este ciclo
  Timeout,         // se pidio el dato y no llego respuesta (o llego incompleta) a tiempo
  NoAck,           // fallo del bus: el dispositivo no contesto en absoluto
  InvalidFrame,    // llego respuesta pero no paso la comprobacion de checksum/formato
  Ok
};

// Que significa cada posicion de values[] y cuantas hay (count) depende del
// sensor; esta documentado en la cabecera de cada driver.
struct Reading {
  ReadingStatus status = ReadingStatus::NotInitialized;
  float values[4] = {0, 0, 0, 0};
  uint8_t count = 0;

  bool ok() const { return status == ReadingStatus::Ok; }
};
