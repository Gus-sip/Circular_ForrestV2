/*
 * CHIP FOREST - bring-up sampler.
 *
 * All five sensors driven through the common ISensor interface, printed as a table
 * over USB serial once a second. No deep sleep / RTC wake / LoRa / telemetry packing
 * yet - this milestone is "get all five sensors producing trustworthy numbers,"
 * nothing more.
 */

#include <Arduino.h>
#include <Wire.h>
#include "pins.h"
#include "Config.h"
#include "PowerManager.h"
#include "Sampler.h"
#include "sensors/Bme690Sensor.h"
#include "sensors/Sen0466Sensor.h"
#include "sensors/Cm1106Sensor.h"
#include "sensors/CalypsoSensor.h"
#include "sensors/Bmv080Sensor.h"

PowerManager power;
Sampler sampler;

Bme690Sensor bme690(BME690_I2C_ADDR);
Sen0466Sensor sen0466(SEN0466_I2C_ADDR);
Bmv080Sensor bmv080(BMV080_I2C_ADDR);
Cm1106Sensor cm1106(Serial2, PIN_CM1106_RX, PIN_CM1106_TX, PIN_CM1106_EN, CM1106_WARMUP_MS);
CalypsoSensor calypso(Serial1, PIN_CALYPSO_RX, PIN_CALYPSO_TX);

void setup() {
  Serial.begin(115200);
  uint32_t waitStart = millis();
  while (!Serial && millis() - waitStart < 3000) delay(10);
  delay(500);

  Serial.println();
  Serial.println("=== CHIP FOREST bring-up: BME690 + SEN0466 + BMV080 + CM1106 + Calypso ===");

  power.enable3V3Sensors();

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  // BMV080 needs this window for HW init / laser preheat / self-test before
  // bmv080_open will succeed reliably - doing it once here up front rather than
  // inside the driver keeps every sensor's begin() call in this file symmetric.
  Serial.printf("Waiting %dms for BMV080 startup...\n", BMV080_STARTUP_DELAY_MS);
  delay(BMV080_STARTUP_DELAY_MS);

  sampler.addSensor(&bme690);
  sampler.addSensor(&sen0466);
  sampler.addSensor(&bmv080);
  sampler.addSensor(&cm1106);
  sampler.addSensor(&calypso);
  sampler.beginAll();

  Serial.println();
}

void loop() {
  static uint32_t lastSample = 0;
  if (millis() - lastSample >= SAMPLE_INTERVAL_MS) {
    lastSample = millis();
    sampler.printTable();
  }
}
