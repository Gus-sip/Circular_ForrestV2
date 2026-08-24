#include <Arduino.h>
#include <Wire.h>
#include <DFRobot_MultiGasSensor.h>

// Standalone test for the DFRobot SEN0466 Gravity CO sensor (I2C, factory-calibrated, 0-1000 ppm).
// NOT WIRED YET - pins below assume it shares the BME690's I2C bus (Wire, SDA=8 SCL=9),
// same as main.cpp. Adjust I2C_SDA/I2C_SCL once the actual wiring is decided.
// Default I2C address per DFRobot's SEN0466 page is 0x74 (SEL dip switch = 0).

#define I2C_SDA 8
#define I2C_SCL 9
#define CO_I2C_ADDR 0x74

DFRobot_GAS_I2C co(&Wire, CO_I2C_ADDR);

void setup() {
  Serial.begin(115200);
  uint32_t waitStart = millis();
  while (!Serial && millis() - waitStart < 3000) delay(10);

  Serial.println();
  Serial.println("=== TEST SEN0466 (DFRobot Gravity CO sensor, I2C) ===");

  Wire.begin(I2C_SDA, I2C_SCL);

  Serial.println("Scanning I2C bus...");
  byte found = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  device found at 0x%02X\n", addr);
      found++;
    }
  }
  if (found == 0) Serial.println("  no I2C device found. Check wiring/power.");

  uint32_t beginStart = millis();
  while (!co.begin()) {
    Serial.println("SEN0466 not detected, retrying...");
    delay(1000);
    if (millis() - beginStart > 10000) {
      Serial.println("SEN0466: giving up after 10s. Check wiring/address.");
      while (1) delay(1000);
    }
  }
  Serial.println("SEN0466 found!");

  co.changeAcquireMode(co.PASSIVITY);
  delay(1000);
  co.setTempCompensation(co.ON);

  Serial.print("Gas type reported by sensor: ");
  Serial.println(co.queryGasType());
}

void loop() {
  float ppm = co.readGasConcentrationPPM();
  float tempC = co.readTempC();

  Serial.print("CO: ");
  Serial.print(ppm);
  Serial.print(" ppm  |  Temp: ");
  Serial.print(tempC);
  Serial.println(" C");

  delay(1000);
}
