#pragma once

#include <stdint.h>

// =============================================================================
// UNA LECTURA DE UN NODO, TAL COMO LA GUARDA MODULO B
// =============================================================================
//
// Las ultimas lecturas conocidas mas la calidad del enlace y su frescura. Esta
// estructura es lo unico que conoce la parte web: no sabe que existen LoRa ni el
// interprete, solo lee la lectura que se le da.
struct SensorSnapshot {
  bool hasData = false;     // false hasta que llega el primer paquete valido
  uint32_t lastHeardMs = 0;  // millis() del momento en que se TOMO la lectura (ya corregido por su antiguedad)
  uint16_t senderAddr = 0;  // direccion LoRa AT+ADDRESS del nodo que la envio
  int16_t rssi = 0;         // intensidad de la senal recibida (dBm)
  int8_t snr = 0;           // relacion senal/ruido (dB)

  float temp = 0;    // BME690, grados C
  float hum = 0;      // BME690, %
  float pres = 0;      // BME690, hPa
  float gas = 0;        // BME690, ohmios

  float pm1 = 0;    // BMV080, ug/m3
  float pm25 = 0;
  float pm10 = 0;

  float co2 = 0;   // CM1106, ppm

  float co = 0;      // SEN0466, ppm
  float coTemp = 0;   // SEN0466, temperatura interna, grados C

  float windAngle = 0;  // Calypso, grados
  float windSpeed = 0;   // Calypso
  bool windValid = false;

  // Salud del nodo. Llega en el paquete STAT, aparte de la telemetria. Se rellena
  // con el ultimo STAT recibido de ese nodo, para que el estado de carga viaje a
  // ThingsBoard junto con los datos de los sensores en vez de quedarse en Modulo B.
  //
  // chargePct vale -1 hasta que se recibe un STAT de ese nodo: un 0% real y "aun no
  // se sabe" son cosas distintas, y un nodo que dijera 0% simplemente por no haber
  // hablado todavia alarmaria sin motivo.
  int16_t chargePct = -1;
  int16_t capMv = -1;

  // Estado de incendio segun el STAT del nodo: 0 normal, 1 pre-alarma, 2 alarma.
  // -1 significa que el nodo nunca lo dijo (un nodo antiguo cuyo STAT solo tiene ocho
  // campos). "Desconocido" y "normal" tienen que seguir siendo distintos: un panel que
  // muestra NORMAL con seguridad para un nodo que no informa es peor que uno que no
  // muestra nada.
  int8_t alarmState = -1;
};
