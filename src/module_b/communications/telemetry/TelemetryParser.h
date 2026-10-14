#pragma once

#include "SensorSnapshot.h"

// =============================================================================
// INTERPRETE DE LA TELEMETRIA DE LOS NODOS (Modulo C)
// =============================================================================
//
// Convierte el texto que manda el nodo en una SensorSnapshot. No sabe nada de LoRa
// ni del servidor web: entra texto y sale una estructura, asi que se puede probar o
// reutilizar sin ninguno de los dos.
//
// FORMATO: CSV de orden fijo, sin cabecera, exactamente 13 campos:
//
//   temp,hum,pres,gas,pm1,pm25,pm10,co2,co,coTemp,windAngle,windSpeed,windValid
//
// Todos son numeros decimales salvo windValid (literalmente "0" o "1"). Un campo que
// el nodo no midio llega como -999. Si cambia la codificacion en el nodo, hay que
// cambiar kFieldCount y la asignacion de campos del .cpp: los dos extremos tienen
// que coincidir en el orden exacto (ver transmitStore() en el firmware de Modulo C).
class TelemetryParser {
public:
  // Una lectura suelta: exactamente 13 campos separados por comas, ni uno mas. Si
  // no, devuelve false y deja `out` sin tocar: se rechaza entera en vez de rellenar a
  // medias una lectura a partir de un texto mal formado o desincronizado.
  // RECHAZA un lote ("B,<n>,...") - ver parseBatch.
  static bool parse(const char *payload, uint8_t len, SensorSnapshot &out);

  // Un lote: "B,<cuantas>,<registro>;<registro>;..." donde cada registro son los
  // mismos 13 campos mas, al final, su antiguedad en el momento del envio (hoy en
  // SEGUNDOS reales; antes eran ticks de 10 s, de ahi el nombre del parametro).
  //
  // Devuelve cuantos registros se leyeron de verdad, escribiendo hasta maxOut
  // lecturas y (opcionalmente) sus antiguedades. El numero de la cabecera no se cree:
  // un paquete cortado devuelve los registros que realmente estan.
  static uint8_t parseBatch(const char *payload, uint8_t len, SensorSnapshot *out,
                            uint8_t maxOut, uint16_t *ageTicksOut = nullptr);

private:
  static constexpr uint8_t kFieldCount = 13;
  static constexpr uint8_t kBatchFieldCount = 14;  // 13 + antiguedad
};
