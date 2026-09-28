#pragma once

#include <stdint.h>

// Latest-known sensor readings plus link quality and freshness. This struct is
// the only thing the web layer knows about - it has no idea LoRa or a parser
// exist, it just reads whatever snapshot it's handed.
struct SensorSnapshot {
  bool hasData = false;     // false until the very first valid packet arrives
  uint32_t lastHeardMs = 0;  // millis() timestamp of the last valid packet
  uint16_t senderAddr = 0;  // LoRa AT+ADDRESS of the node that sent this reading
  int16_t rssi = 0;
  int8_t snr = 0;

  float temp = 0;    // BME690, deg C
  float hum = 0;      // BME690, %
  float pres = 0;      // BME690, hPa
  float gas = 0;        // BME690, ohm

  float pm1 = 0;    // BMV080, ug/m3
  float pm25 = 0;
  float pm10 = 0;

  float co2 = 0;   // CM1106, ppm

  float co = 0;      // SEN0466, ppm
  float coTemp = 0;   // SEN0466 onboard temp, deg C

  float windAngle = 0;  // Calypso, deg
  float windSpeed = 0;   // Calypso
  bool windValid = false;

  // Node health, carried in the node's separate STAT packet rather than in the
  // telemetry payload. Filled in from the most recent STAT seen from that node, so
  // the charge state travels to ThingsBoard alongside the sensor data instead of
  // stopping at Module B.
  //
  // chargePct is -1 until a STAT has been heard from that node - a real 0% and
  // "not yet known" are different things, and a node reporting 0% when it simply
  // has not said anything yet would be alarming for no reason.
  int16_t chargePct = -1;
  int16_t capMv = -1;

  // Fire state as reported by the node in its STAT packet: 0 normal, 1 pre-alarm,
  // 2 alarm. -1 means the node never said - an older node whose STAT has only
  // eight fields. "Unknown" and "normal" must stay distinct: a dashboard showing
  // a confident NORMAL for a node that is not reporting at all is worse than one
  // showing nothing.
  int8_t alarmState = -1;
};
