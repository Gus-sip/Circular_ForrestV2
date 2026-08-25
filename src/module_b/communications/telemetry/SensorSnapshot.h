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
};
