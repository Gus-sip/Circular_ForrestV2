/*
 * RYLR998 isolation test - bare AT liveness check + transparent bridge, no
 * WiFi/AP/web server involved at all. Same wiring as main.cpp (GPIO4/5).
 *
 * Purpose: rule out any interaction with the WiFi/AP stack when main.cpp's
 * "AT" gets no reply at all. If this ALSO gets nothing, the problem is
 * upstream of firmware entirely - wiring, ground, power, or the module's
 * actual configured baud rate - not something to keep retrying code for.
 */

#include <Arduino.h>

#define LORA_RX_PIN 4  // module TXD -> ESP RX
#define LORA_TX_PIN 5  // module RXD <- ESP TX
#define LORA_BAUD 115200

HardwareSerial LoRaSerial(1);

void setup() {
  Serial.begin(115200);
  uint32_t waitStart = millis();
  while (!Serial && millis() - waitStart < 3000) delay(10);
  delay(500);

  Serial.println();
  Serial.println("=== RYLR998 isolation test (no WiFi/web server) ===");

  LoRaSerial.begin(LORA_BAUD, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);

  Serial.println("Sending AT...");
  LoRaSerial.print("AT\r\n");

  uint32_t start = millis();
  bool gotReply = false;
  while (millis() - start < 1000) {
    if (LoRaSerial.available()) {
      Serial.write(LoRaSerial.read());
      gotReply = true;
    }
  }
  Serial.println();
  if (!gotReply) {
    Serial.println("No reply at all - check: common GND between module and board,");
    Serial.println("solid 3.3V on module VDD, TXD/RXD not swapped, and that the module");
    Serial.println("hasn't previously been configured to a non-default baud rate.");
  }

  Serial.println();
  Serial.println("--- Transparent bridge: type AT commands below, Enter to send ---");
}

void loop() {
  while (LoRaSerial.available()) Serial.write(LoRaSerial.read());
  while (Serial.available()) LoRaSerial.write(Serial.read());
}
