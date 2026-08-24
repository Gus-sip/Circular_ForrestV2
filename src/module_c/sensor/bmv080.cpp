#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_BMV080_Arduino_Library.h>

#define BMV080_I2C_ADDR 0x57

SparkFunBMV080 bmv080;

void setup() {
  Serial.begin(115200);
  uint32_t serialWaitStart = millis();
  while (!Serial && millis() - serialWaitStart < 3000) delay(10);

  Wire.begin();

  if (!bmv080.begin(BMV080_I2C_ADDR, Wire)) {
    Serial.println("BMV080 not detected. Check wiring and I2C address. Freezing...");
    while (1) delay(10);
  }
  Serial.println("BMV080 found!");

  bmv080.init();

  if (bmv080.setMode(SF_BMV080_MODE_CONTINUOUS)) {
    Serial.println("BMV080 set to continuous mode");
  } else {
    Serial.println("Error setting BMV080 mode");
  }
}

void loop() {
  if (bmv080.readSensor()) {
    Serial.print("PM1: ");
    Serial.print(bmv080.PM1());
    Serial.print("\tPM2.5: ");
    Serial.print(bmv080.PM25());
    Serial.print("\tPM10: ");
    Serial.print(bmv080.PM10());
    if (bmv080.isObstructed()) {
      Serial.print("\tObstructed");
    }
    Serial.println();
  }

  delay(100);
}
