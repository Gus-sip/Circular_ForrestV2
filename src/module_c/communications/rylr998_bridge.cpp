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

// PCB power-rail enables (see sensor/pins.h). Each drives a high-side transistor
// closing the circuit into the 5V / 3V3 rail; LOW = rail on. This sketch predates
// the fabbed PCB, so without these the RYLR998 may simply be unpowered and the
// "no reply to AT" below would be a power fault wearing a wiring fault's clothes.
#define PCB_EN_A_PIN 10
#define PCB_EN_B_PIN 11
#define PCB_EN_SETTLE_MS 300

// The RYLR998 has its OWN rail gate, separate from the two sensor rails: a P-FET
// on GPIO13, LOW = on (same pin/convention as Module B's LORA_EN_PIN). Leaving it
// undriven is why the sweep below found silence at every baud.
#define LORA_EN_PIN 13
#define LORA_EN_SETTLE_MS 200

// Baud sweep. The module is documented as 115200 8N1, but a module that has had
// AT+IPR changed sticks at the new rate across power cycles, and the header above
// names wrong baud as the first suspect for total silence - so prove it rather
// than assume it. 115200 first so the normal case costs one pass.
static const uint32_t kBaudSweep[] = {115200, 9600, 57600, 38400, 19200, 4800};

HardwareSerial LoRaSerial(1);  // UART1

void setup() {
  Serial.begin(115200);
  uint32_t waitStart = millis();
  while (!Serial && millis() - waitStart < 3000) delay(10);
  delay(500);

  Serial.println();
  Serial.println("=== RYLR998 AT bring-up: liveness check + transparent bridge ===");

  // Rails first - an unpowered module is silent in exactly the same way a
  // miswired one is, so remove that ambiguity before the sweep runs.
  pinMode(PCB_EN_A_PIN, OUTPUT);
  digitalWrite(PCB_EN_A_PIN, LOW);
  pinMode(PCB_EN_B_PIN, OUTPUT);
  digitalWrite(PCB_EN_B_PIN, HIGH);  // 5V gate is ACTIVE-HIGH - see pins.h
  Serial.printf("PCB rails: GPIO%d=LOW (3V3), GPIO%d=HIGH (5V), settling %dms\n", PCB_EN_A_PIN,
                PCB_EN_B_PIN, PCB_EN_SETTLE_MS);
  delay(PCB_EN_SETTLE_MS);

  // LoRa rail last - it is the one this sketch actually cares about.
  pinMode(LORA_EN_PIN, OUTPUT);
  digitalWrite(LORA_EN_PIN, LOW);
  Serial.printf("LoRa rail: GPIO%d LOW, settling %dms\n", LORA_EN_PIN, LORA_EN_SETTLE_MS);
  delay(LORA_EN_SETTLE_MS);

  // ---- Line-continuity probe, before any UART is opened ----
  // The module's TXD is a UART output idling HIGH. So drive an internal PULLDOWN
  // on the ESP's RX pin and see who wins: if the line still reads HIGH, something
  // out there is actively driving it (module present, wired, powered). If it
  // follows the pulldown to LOW, nothing is on the far end - an open line, which
  // on a freshly assembled board means an unpopulated 0R series resistor.
  //
  // The pull-UP reading is the control: it should read HIGH either way, so a pin
  // that reads LOW even with the pullup is shorted to ground, a different fault.
  {
    Serial.println();
    Serial.println("--- line continuity probe (no UART yet) ---");
    const int probePins[2] = {LORA_RX_PIN, LORA_TX_PIN};
    const char *probeNames[2] = {"GPIO4 (ESP RX <- module TXD)", "GPIO5 (ESP TX -> module RXD)"};
    for (int i = 0; i < 2; i++) {
      pinMode(probePins[i], INPUT_PULLDOWN);
      delay(30);
      int withPulldown = digitalRead(probePins[i]);
      pinMode(probePins[i], INPUT_PULLUP);
      delay(30);
      int withPullup = digitalRead(probePins[i]);
      pinMode(probePins[i], INPUT);

      const char *verdict;
      if (withPulldown == HIGH && withPullup == HIGH) {
        verdict = "DRIVEN HIGH - something is on the far end";
      } else if (withPulldown == LOW && withPullup == HIGH) {
        verdict = "FLOATING - open line, nothing driving it";
      } else if (withPulldown == LOW && withPullup == LOW) {
        verdict = "SHORTED TO GND";
      } else {
        verdict = "inconsistent";
      }
      Serial.printf("  %-30s pulldown=%d pullup=%d -> %s\n", probeNames[i], withPulldown,
                    withPullup, verdict);
    }
    Serial.println("  (GPIO4 is the informative one: the module's TXD idles high, so a");
    Serial.println("   FLOATING verdict there means the RX path is broken, not the module.)");
    Serial.println();
  }

  // ---- Phase A: internal self-loopback, no jumper ----
  // The GPIO matrix lets a UART's RX and TX sit on the SAME pin, so the ESP reads
  // back whatever it just drove. That proves the UART peripheral, the pin mux and
  // the pad itself all work, without touching the wiring. If a pin echoes here,
  // any silence afterwards is the far end's fault, not ours.
  {
    Serial.println("--- phase A: internal self-loopback (RX and TX on one pin) ---");
    const int selfPins[2] = {LORA_TX_PIN, LORA_RX_PIN};
    for (int i = 0; i < 2; i++) {
      LoRaSerial.begin(115200, SERIAL_8N1, selfPins[i], selfPins[i]);
      delay(60);
      while (LoRaSerial.available()) LoRaSerial.read();
      LoRaSerial.print("AT\r\n");
      LoRaSerial.flush();

      char buf[48];
      size_t n = 0;
      uint32_t start = millis();
      while (millis() - start < 300 && n < sizeof(buf) - 1) {
        if (LoRaSerial.available()) buf[n++] = (char)LoRaSerial.read();
      }
      buf[n] = 0;
      Serial.printf("  GPIO%d self-echo: %u bytes %s\n", selfPins[i], (unsigned)n,
                    n >= 4 ? "-> PAD DRIVEN OK" : "-> NO ECHO (pin or UART fault on our side)");
      LoRaSerial.end();
    }
    Serial.println();
  }

  // ---- Phase B: passive listen ----
  // Say nothing and just watch the RX line. Some modules emit a banner or a READY
  // line shortly after power-up; anything at all here proves the module's TXD is
  // alive and reaching us, independently of whether it ever hears our commands.
  {
    Serial.println("--- phase B: passive listen on RX, 3s, no bytes sent ---");
    LoRaSerial.begin(115200, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
    size_t n = 0;
    uint32_t start = millis();
    while (millis() - start < 3000) {
      if (LoRaSerial.available()) {
        uint8_t c = LoRaSerial.read();
        if (n == 0) Serial.print("  bytes: ");
        Serial.printf("%02X ", c);
        n++;
      }
    }
    if (n == 0) Serial.println("  (nothing - module TXD is idle or not reaching us)");
    else Serial.printf("\n  %u unsolicited bytes - module TXD IS alive\n", (unsigned)n);
    LoRaSerial.end();
    Serial.println();
  }

  // ---- Phase C: terminator variants ----
  // REYAX documents CRLF, but a module in an odd state may answer to one or the
  // other. Cheap to rule out rather than assume.
  {
    Serial.println("--- phase C: AT with different terminators @115200 ---");
    const char *cmds[3] = {"AT\r\n", "AT\r", "AT\n"};
    const char *names[3] = {"CRLF", "CR only", "LF only"};
    LoRaSerial.begin(115200, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
    for (int i = 0; i < 3; i++) {
      while (LoRaSerial.available()) LoRaSerial.read();
      LoRaSerial.print(cmds[i]);
      LoRaSerial.flush();
      size_t n = 0;
      uint32_t start = millis();
      while (millis() - start < 700) {
        if (LoRaSerial.available()) { LoRaSerial.read(); n++; }
      }
      Serial.printf("  %-8s -> %u bytes\n", names[i], (unsigned)n);
    }
    LoRaSerial.end();
    Serial.println();
  }

  // Sweep BOTH crossover orientations. The schematic names these nets from the
  // MCU's point of view, so RX=4/TX=5 should be right - but a swapped pair looks
  // identical to a dead module from here, and eliminating it in firmware is free.
  const int rxPins[2] = {LORA_RX_PIN, LORA_TX_PIN};
  const int txPins[2] = {LORA_TX_PIN, LORA_RX_PIN};

  uint32_t liveBaud = 0;
  int liveRx = 0, liveTx = 0;
  for (int orient = 0; orient < 2; orient++) {
    Serial.printf("\n--- orientation %d: RX=GPIO%d TX=GPIO%d ---\n", orient,
                  rxPins[orient], txPins[orient]);
  for (size_t i = 0; i < sizeof(kBaudSweep) / sizeof(kBaudSweep[0]); i++) {
    uint32_t baud = kBaudSweep[i];
    LoRaSerial.begin(baud, SERIAL_8N1, rxPins[orient], txPins[orient]);
    delay(120);
    while (LoRaSerial.available()) LoRaSerial.read();  // drop stale bytes

    LoRaSerial.print("AT\r\n");

    char reply[96];
    size_t n = 0;
    uint32_t start = millis();
    while (millis() - start < 800 && n < sizeof(reply) - 1) {
      if (LoRaSerial.available()) reply[n++] = (char)LoRaSerial.read();
    }
    reply[n] = 0;

    if (n == 0) {
      Serial.printf("  %6lu baud: silent\n", (unsigned long)baud);
    } else {
      Serial.printf("  %6lu baud: %u bytes -> ", (unsigned long)baud, (unsigned)n);
      for (size_t j = 0; j < n; j++) {
        char c = reply[j];
        if (c == 13) Serial.print("<CR>");
        else if (c == 10) Serial.print("<LF>");
        else if (c >= 32 && c < 127) Serial.write(c);
        else Serial.printf("<%02X>", (uint8_t)c);
      }
      Serial.println();
      if (liveBaud == 0 && strstr(reply, "+OK") != nullptr) {
        liveBaud = baud;
        liveRx = rxPins[orient];
        liveTx = txPins[orient];
      }
    }
    LoRaSerial.end();
  }
  }

  if (liveBaud != 0) {
    Serial.printf("\nRYLR998 answered +OK at %lu baud, RX=GPIO%d TX=GPIO%d - bridge opening there.\n",
                  (unsigned long)liveBaud, liveRx, liveTx);
  } else {
    Serial.println();
    Serial.println("No +OK at any swept baud. Next suspects, in order: the module has no");
    Serial.println("power (is it on a gated rail?), TXD/RXD are not on GPIO4/5 on this PCB,");
    Serial.println("or the crossover is reversed. Opening the bridge at 115200 anyway.");
    liveBaud = LORA_BAUD;
  }
  if (liveRx == 0) { liveRx = LORA_RX_PIN; liveTx = LORA_TX_PIN; }
  LoRaSerial.begin(liveBaud, SERIAL_8N1, liveRx, liveTx);

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
