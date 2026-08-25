/*
 * Combined sensor node - ESP32-S3 Mini
 *
 * Sensors on this board:
 *   - BME690 (7Semi lib)       I2C (Wire)   SDA=8  SCL=9    addr 0x76
 *   - SEN0466 CO (DFRobot lib) I2C (Wire)   SDA=8  SCL=9    addr 0x74  [not physically wired yet]
 *   - BMV080 (SparkFun lib)    I2C (Wire1)  SDA=10 SCL=11   addr 0x57
 *   - CM1106SL-NS CO2 (Cubic)  UART2 RX=GPIO7  TX=GPIO4      no EN pin on this breakout
 *   - Calypso ULP Pro wind     UART1 RX=GPIO5  TX=GPIO6
 *
 * CM1106 needs 5V (VBUS) on its VCC pin; every other sensor runs on 3V3. All grounds common.
 * BMV080 is on its own I2C bus (Wire1) rather than sharing BME690's, to rule out a shared-bus issue.
 * SEN0466 shares BME690's Wire bus - its address (0x74) doesn't collide with BME690 (0x76).
 *
 * Per-sensor protocol handling (CO2 UART framing, wind NMEA parsing, BMV080 SDK
 * callbacks, ...) lives in sensors/ behind the shared ISensor interface, same as
 * chip_forest_lora_tx.cpp - this file just wires up the board-specific pins/buses
 * and prints each reading in this node's own format.
 */
#include <Arduino.h>
#include <Wire.h>
#include "sensors/Bme690Sensor.h"
#include "sensors/Sen0466Sensor.h"
#include "sensors/Bmv080Sensor.h"
#include "sensors/Cm1106Sensor.h"
#include "sensors/CalypsoSensor.h"

// ---------- BME690 + SEN0466: I2C on Wire ----------
#define I2C_SDA 8
#define I2C_SCL 9
#define BME690_I2C_ADDR 0x76
#define CO_I2C_ADDR 0x74

// ---------- BMV080: I2C on Wire1 (dedicated bus) ----------
#define BMV080_SDA 10
#define BMV080_SCL 11
#define BMV080_I2C_ADDR 0x57

// ---------- CM1106SL-NS CO2 sensor: UART2 (no EN pin on this breakout) ----------
#define CO2_RX_PIN 7  // sensor TX -> ESP RX
#define CO2_TX_PIN 4  // sensor RX <- ESP TX

// ---------- Calypso ULP Pro wind sensor: UART1 ----------
#define WIND_RX_PIN 5  // sensor GREEN wire (TX out of sensor)
#define WIND_TX_PIN 6  // sensor YELLOW wire (RX into sensor)

TwoWire BMV080Wire(1);

Bme690Sensor bme690(BME690_I2C_ADDR);
Sen0466Sensor sen0466(CO_I2C_ADDR);
Bmv080Sensor bmv080(BMV080_I2C_ADDR, BMV080Wire);
Cm1106Sensor cm1106(Serial2, CO2_RX_PIN, CO2_TX_PIN, Cm1106Sensor::kNoEnPin, /*warmupMs=*/0);
CalypsoSensor calypso(Serial1, WIND_RX_PIN, WIND_TX_PIN);

bool bmeReady = false;
bool sen0466Ready = false;
bool bmvReady = false;

void setup() {
  Serial.begin(115200);
  uint32_t waitStart = millis();
  while (!Serial && millis() - waitStart < 3000) delay(10);
  delay(1500);  // extra settle time so the monitor doesn't miss the setup banner

  Serial.println();
  Serial.println("=== Combined sensor node (BME690 + SEN0466 + BMV080 + CM1106 + ULP Pro) ===");

  Wire.begin(I2C_SDA, I2C_SCL);
  BMV080Wire.begin(BMV080_SDA, BMV080_SCL);

  Serial.println("Scanning BMV080 I2C bus (Wire1, SDA=10 SCL=11)...");
  {
    byte found = 0;
    for (byte addr = 1; addr < 127; addr++) {
      BMV080Wire.beginTransmission(addr);
      if (BMV080Wire.endTransmission() == 0) {
        Serial.printf("  device found at 0x%02X\n", addr);
        found++;
      }
    }
    if (found == 0) Serial.println("  no I2C device found on Wire1");
  }

  bmeReady = bme690.begin();
  Serial.println(bmeReady ? "BME690: OK" : "BME690: NOT FOUND");

  sen0466Ready = sen0466.begin();
  Serial.println(sen0466Ready ? "SEN0466: OK" : "SEN0466: NOT FOUND");

  bmvReady = bmv080.begin();
  Serial.println(bmvReady ? "BMV080: OK (continuous mode)" : "BMV080: NOT FOUND");

  cm1106.begin();
  calypso.begin();

  Serial.println("Setup complete.\n");
}

void loop() {
  static uint32_t lastPoll = 0;

  // Wind sensor streams continuously - drain every loop iteration so the UART buffer
  // never has a chance to overflow between samples.
  Reading wind = calypso.read();
  if (wind.ok()) {
    Serial.printf("Wind: %.1f deg | %.1f | %s\n", wind.values[0], wind.values[1],
                  wind.values[2] > 0.5f ? "VALID" : "INVALID");
  }

  if (bmvReady) {
    Reading pm = bmv080.read();
    if (pm.ok()) {
      Serial.printf("PM1: %.1f  PM2.5: %.1f  PM10: %.1f%s\n", pm.values[0], pm.values[1], pm.values[2],
                    pm.values[3] > 0.5f ? "  [obstructed]" : "");
    }
  }

  // Polled sensors (BME690 + CO + CO2): every 3s, since the CO2 read blocks for up to 1s.
  if (millis() - lastPoll >= 3000) {
    lastPoll = millis();

    if (bmeReady) {
      Reading env = bme690.read();
      if (env.ok()) {
        Serial.printf("BME690: %.1f C | %.1f %% | %.1f hPa | %.0f ohm\n", env.values[0], env.values[1],
                      env.values[2], env.values[3]);
      } else {
        Serial.println("BME690: read failed");
      }
    }

    if (sen0466Ready) {
      Reading co = sen0466.read();
      Serial.printf("CO: %.1f ppm\n", co.values[0]);
    }

    Reading co2 = cm1106.read();
    if (co2.ok()) {
      Serial.printf("CO2: %.0f ppm\n", co2.values[0]);
    } else {
      Serial.println("CO2: read failed (timeout or bad frame)");
    }
  }
}