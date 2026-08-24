/*
 * RYLR998 LoRa module - AT bring-up, stage 1: liveness check + transparent bridge.
 *
 * Purpose: confirm the module answers AT commands, then act as a plain
 * USB<->UART passthrough so the REYAX AT command set can be driven by hand from
 * the serial monitor. No AT command parsing, no framing, no driver abstraction -
 * that comes in a later stage, once responses have been seen firsthand.
 *
 * Wiring (ESP32-S3-Zero):
 *   Module VDD  -> 3V3
 *   Module GND  -> GND
 *   Module TXD  -> GPIO4 (ESP RX)
 *   Module RXD  -> GPIO5 (ESP TX)
 *   Module NRST -> floating, not wired this stage
 *
 * NOTE - net-name crossover: this project's schematic names the LoRa UART nets
 * from the MCU's point of view (UART1_TX = the ESP's transmit pin), the opposite
 * convention used for the CM1106/Calypso nets on this same board. So the
 * crossover below (GPIO5 ESP-TX -> module RXD, GPIO4 ESP-RX -> module TXD) is
 * correct as written - don't "fix" it to mirror the other sensors' wiring.
 *
 * Module UART is fixed at 115200 8N1 by default. If AT gets no reply at all,
 * wrong baud is the first suspect, not a code bug.
 *
 * TX bursts pull ~120mA. If the module resets mid-send, that's a brownout from
 * insufficient bulk decoupling on the bench supply, not something to fix in
 * firmware - flag it and add capacitance, don't add retry/backoff to paper over it.
 *
 * This is a real hardware UART (UART1 peripheral), not SoftwareSerial - required
 * for a module that expects clean, continuously-available AT framing.
 */

#include <Arduino.h>

#define LORA_RX_PIN 4  // module TXD -> ESP RX
#define LORA_TX_PIN 5  // module RXD <- ESP TX
#define LORA_BAUD 115200

HardwareSerial LoRaSerial(1);  // UART1

void setup() {
  Serial.begin(115200);
  uint32_t waitStart = millis();
  while (!Serial && millis() - waitStart < 3000) delay(10);
  delay(500);

  Serial.println();
  Serial.println("=== RYLR998 AT bring-up: liveness check + transparent bridge ===");

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
    Serial.println("No reply to AT within 1s - check wiring/crossover and that the");
    Serial.println("module is actually at 115200 8N1 before assuming a code bug.");
  }

  Serial.println();
  Serial.println("--- Transparent bridge: type AT commands below, Enter to send ---");
  Serial.println("--- Everything the module sends back is printed as-is           ---");
}

void loop() {
  // Module -> USB
  while (LoRaSerial.available()) {
    Serial.write(LoRaSerial.read());
  }

  // USB -> module: raw passthrough, byte for byte.
  while (Serial.available()) {
    LoRaSerial.write(Serial.read());
  }
}
