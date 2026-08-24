/*
 * RYLR998 parameter probe - diagnostic for the AT+PARAMETER=7,7,1,4 rejection
 * (+ERR=18) seen from chip_forest_lora_tx.cpp. Queries the module for its
 * current AT+PARAMETER value (whatever it already has, since our SET was
 * rejected) rather than guessing a new SF/BW/CR/preamble combination blind -
 * ground truth from the actual device beats trial-and-error against an
 * un-vendored datasheet.
 */

#include <Arduino.h>

#define LORA_RX_PIN 4
#define LORA_TX_PIN 5
#define LORA_BAUD 115200

HardwareSerial LoRaSerial(1);

static void sendAndPrint(const char *cmd) {
  Serial.printf("-> %s\n", cmd);
  LoRaSerial.print(cmd);
  LoRaSerial.print("\r\n");

  uint32_t start = millis();
  while (millis() - start < 1000) {
    if (LoRaSerial.available()) Serial.write(LoRaSerial.read());
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  uint32_t waitStart = millis();
  while (!Serial && millis() - waitStart < 3000) delay(10);
  delay(500);

  Serial.println();
  Serial.println("=== RYLR998 parameter probe ===");
  LoRaSerial.begin(LORA_BAUD, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);

  sendAndPrint("AT");
  sendAndPrint("AT+PARAMETER?");
  sendAndPrint("AT+BAND?");
  sendAndPrint("AT+ADDRESS?");
  sendAndPrint("AT+NETWORKID?");
  sendAndPrint("AT+VER?");

  Serial.println("--- done, dropping into transparent bridge for manual follow-up ---");
}

void loop() {
  while (LoRaSerial.available()) Serial.write(LoRaSerial.read());
  while (Serial.available()) LoRaSerial.write(Serial.read());
}
