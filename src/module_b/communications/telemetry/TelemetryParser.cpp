#include "TelemetryParser.h"
#include <stdlib.h>
#include <string.h>

// Parte un registro separado por comas en EXACTAMENTE kFieldCount campos.
//
// Devuelve false si hay de menos O DE MAS - y el caso "de mas" es el que importa.
// La version anterior dejaba de mirar al tener 13 campos y luego comprobaba
// `fieldIdx != kFieldCount`, que por construccion no podia fallar nunca. Cualquier
// cosa con MAS de 13 campos se aceptaba en silencio usando solo los 13 primeros, asi
// que un lote "B,2,<registro>;<registro>;" se interpretaba como una sola lectura con
// la cabecera metida en los datos:
//
//   "B"   -> temp = 0.0      (atof de una letra)
//   "2"   -> rh   = 2.0      (el numero de registros del lote)
//   42.4  -> pres            (en realidad la temperatura)
//   11.3  -> gas             (en realidad la humedad)
//
// que es exactamente la corrupcion que se vio en ThingsBoard: todos los campos
// desplazados dos posiciones, con 0.0 y 2.0 apareciendo de la nada. La segunda
// lectura de cada lote se perdia por completo.
static bool splitRecord(char *buf, uint8_t len, char **fields, uint8_t want) {
  uint8_t fieldIdx = 0;
  fields[fieldIdx++] = buf;

  for (uint8_t i = 0; i < len; i++) {
    if (buf[i] == ',') {
      if (fieldIdx >= want) return false;  // MAS campos de los esperados: no es esta forma
      buf[i] = '\0';
      fields[fieldIdx++] = buf + i + 1;
    }
  }
  return fieldIdx == want;
}

// Copia los 13 campos de texto, en su orden fijo, a la estructura de lectura.
static void fillSnapshot(char **fields, SensorSnapshot &out) {
  out.temp = atof(fields[0]);
  out.hum = atof(fields[1]);
  out.pres = atof(fields[2]);
  out.gas = atof(fields[3]);
  out.pm1 = atof(fields[4]);
  out.pm25 = atof(fields[5]);
  out.pm10 = atof(fields[6]);
  out.co2 = atof(fields[7]);
  out.co = atof(fields[8]);
  out.coTemp = atof(fields[9]);
  out.windAngle = atof(fields[10]);
  out.windSpeed = atof(fields[11]);
  out.windValid = (fields[12][0] == '1');
}

// Una lectura suelta: exactamente 13 campos.
bool TelemetryParser::parse(const char *payload, uint8_t len, SensorSnapshot &out) {
  if (len == 0 || len > 200) return false;

  // Un lote tiene otra forma y no se debe meter a la fuerza en esta - ver
  // parseBatch(). Rechazarlo aqui es lo que evita la corrupcion por desplazamiento.
  if (len >= 2 && payload[0] == 'B' && payload[1] == ',') return false;

  char buf[241];
  memcpy(buf, payload, len);
  buf[len] = '\0';

  char *fields[kFieldCount] = {};
  if (!splitRecord(buf, len, fields, kFieldCount)) return false;

  fillSnapshot(fields, out);
  return true;
}

// Un lote: "B,<cuantas>,<registro>;<registro>;...". Devuelve cuantos registros se
// pudieron leer, y la antiguedad de cada uno en ageTicksOut.
uint8_t TelemetryParser::parseBatch(const char *payload, uint8_t len, SensorSnapshot *out,
                                    uint8_t maxOut, uint16_t *ageTicksOut) {
  if (len < 4 || len > 240) return 0;
  if (!(payload[0] == 'B' && payload[1] == ',')) return 0;

  char buf[241];
  memcpy(buf, payload, len);
  buf[len] = '\0';

  // Saltar la cabecera "B,<cuantas>,". El numero NO se toma como cuantos registros
  // hay, a proposito: los registros se cuentan mientras se leen, asi que un paquete
  // cortado o mal formado devuelve los registros que de verdad estan, no un numero
  // prometido que no se cumple.
  char *p = strchr(buf, ',');
  if (!p) return 0;
  p = strchr(p + 1, ',');
  if (!p) return 0;
  p++;

  uint8_t n = 0;
  while (*p && n < maxOut) {
    char *end = strchr(p, ';');
    if (end) *end = '\0';

    uint8_t recLen = (uint8_t)strlen(p);
    if (recLen > 0) {
      // Un registro de lote lleva UN campo mas al final que una lectura suelta: su
      // antiguedad en el momento del envio. Hoy son SEGUNDOS reales medidos por el
      // nodo (antes eran ticks; el nombre ageTicksOut viene de entonces).
      char *fields[kBatchFieldCount] = {};
      if (splitRecord(p, recLen, fields, kBatchFieldCount)) {
        fillSnapshot(fields, out[n]);
        if (ageTicksOut) ageTicksOut[n] = (uint16_t)atoi(fields[13]);
        n++;
      }
    }

    if (!end) break;
    p = end + 1;
  }

  return n;
}
