/*
 * RYLR998 bench probe - is the MODULE dead, or was it the PCB?
 *
 * Context: on the fabbed Module C PCB (2026-09-04) the RYLR998 never answered a
 * bare AT. Everything on the MCU side was eliminated there - six baud rates, both
 * crossover orientations, UART0 and UART1, the GPIO13 rail gate driven low and
 * then physically bypassed with VDD bridged straight to 3V3, and voltages plus
 * continuity confirmed with a meter. An internal self-loopback (UART RX and TX
 * muxed onto one pad) echoed cleanly on both GPIO4 and GPIO5, proving the pads
 * and the UART peripheral good. That left exactly one untested thing: the module.
 *
 * So this sketch runs the same interrogation on a breadboard ESP32-S3 with the
 * module out of the PCB entirely. Same board family, same GPIO4/5 wiring - the
 * ONLY variable removed is the PCB. Read the result that way:
 *
 *   answers here  -> module is fine, the fault is in the Module C PCB
 *   silent here   -> module is dead, and the PCB is exonerated
 *
 * Wiring (breadboard):
 *   Module VDD  -> 3V3      (3.3V part - do NOT feed it 5V)
 *   Module GND  -> GND      (shared ground with the ESP is essential)
 *   Module TXD  -> GPIO4    (ESP RX)
 *   Module RXD  -> GPIO5    (ESP TX)
 *   Module NRST -> leave floating, as on the PCB
 *
 * Phase A runs BEFORE any conclusion is drawn about the module, because a verdict
 * from an unverified board is worthless: if these pads don't echo, this ESP or its
 * wiring is at fault and nothing after it means anything.
 */

#include <Arduino.h>

#define LORA_RX_PIN 4  // module TXD -> ESP RX
#define LORA_TX_PIN 5  // module RXD <- ESP TX

// 115200 is the documented default and comes first so the normal case costs one
// pass. The rest are here because a module whose AT+IPR was changed keeps the new
// rate across power cycles, and total silence cannot distinguish that from death.
static const uint32_t kBaudSweep[] = {115200, 9600, 57600, 38400, 19200, 4800};

HardwareSerial LoRaSerial(1);  // UART1 - real hardware UART, not SoftwareSerial

static void printEscaped(const char *buf, size_t n) {
  for (size_t i = 0; i < n; i++) {
    char c = buf[i];
    if (c == '\r') Serial.print("<CR>");
    else if (c == '\n') Serial.print("<LF>");
    else if (c >= 32 && c < 127) Serial.write(c);
    else Serial.printf("<%02X>", (uint8_t)c);
  }
}

// Sends one command at one baud/orientation and reports whatever comes back.
// Returns the byte count so the caller can distinguish "nothing at all" from
// "something, but not +OK" - those point at different faults.
static size_t tryOnce(uint32_t baud, int rxPin, int txPin, const char *cmd, char *reply,
                      size_t replyCap, uint32_t waitMs) {
  LoRaSerial.begin(baud, SERIAL_8N1, rxPin, txPin);
  delay(120);
  while (LoRaSerial.available()) LoRaSerial.read();  // drop stale bytes

  LoRaSerial.print(cmd);
  LoRaSerial.flush();

  size_t n = 0;
  uint32_t start = millis();
  while (millis() - start < waitMs && n < replyCap - 1) {
    if (LoRaSerial.available()) reply[n++] = (char)LoRaSerial.read();
  }
  reply[n] = 0;
  LoRaSerial.end();
  return n;
}

void setup() {
  Serial.begin(115200);
  uint32_t waitStart = millis();
  while (!Serial && millis() - waitStart < 3000) delay(10);
  delay(500);

  Serial.println();
  Serial.println("=== RYLR998 bench probe: module out of the PCB ===");
  Serial.printf("Wiring expected: module TXD -> GPIO%d, module RXD <- GPIO%d, VDD 3V3, GND shared\n",
                LORA_RX_PIN, LORA_TX_PIN);
  Serial.println();

  // ---- Phase A: verify THIS board before judging the module ----
  // RX and TX muxed onto a single pad, so the ESP reads back what it just drove.
  // Exercises the UART peripheral, the pin mux and the pad without any wiring.
  Serial.println("--- phase A: self-loopback (proves this ESP, not the module) ---");
  bool padsOk = true;
  {
    const int selfPins[2] = {LORA_TX_PIN, LORA_RX_PIN};
    for (int i = 0; i < 2; i++) {
      char buf[48];
      size_t n = tryOnce(115200, selfPins[i], selfPins[i], "AT\r\n", buf, sizeof(buf), 300);
      Serial.printf("  GPIO%d self-echo: %u bytes %s\n", selfPins[i], (unsigned)n,
                    n >= 4 ? "-> pad OK" : "-> NO ECHO");
      if (n < 4) padsOk = false;
    }
  }
  if (!padsOk) {
    Serial.println();
    Serial.println("  !! A pad failed to echo itself. This breadboard ESP or its wiring is at");
    Serial.println("     fault - fix that first. Any verdict about the module below would be");
    Serial.println("     meaningless until these echo.");
  }
  Serial.println();

  // ---- Phase B: listen without transmitting ----
  // Independent of whether the module ever hears us: any unsolicited byte proves
  // its TXD is alive and reaching this pin.
  Serial.println("--- phase B: passive listen on RX, 3s, nothing sent ---");
  {
    LoRaSerial.begin(115200, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
    size_t n = 0;
    uint32_t start = millis();
    while (millis() - start < 3000) {
      if (LoRaSerial.available()) {
        if (n == 0) Serial.print("  bytes: ");
        Serial.printf("%02X ", LoRaSerial.read());
        n++;
      }
    }
    if (n == 0) Serial.println("  (nothing - module TXD idle, which is normal for this part)");
    else Serial.printf("\n  %u unsolicited bytes - module TXD is ALIVE\n", (unsigned)n);
    LoRaSerial.end();
  }
  Serial.println();

  // ---- Phase C: the actual interrogation ----
  Serial.println("--- phase C: AT sweep, both orientations x 6 bauds ---");
  const int rxPins[2] = {LORA_RX_PIN, LORA_TX_PIN};
  const int txPins[2] = {LORA_TX_PIN, LORA_RX_PIN};
  uint32_t liveBaud = 0;
  int liveRx = 0, liveTx = 0;
  size_t anyBytes = 0;

  for (int orient = 0; orient < 2; orient++) {
    Serial.printf("  orientation %d: RX=GPIO%d TX=GPIO%d\n", orient, rxPins[orient], txPins[orient]);
    for (size_t i = 0; i < sizeof(kBaudSweep) / sizeof(kBaudSweep[0]); i++) {
      char reply[96];
      size_t n = tryOnce(kBaudSweep[i], rxPins[orient], txPins[orient], "AT\r\n", reply, sizeof(reply), 800);
      anyBytes += n;
      if (n == 0) {
        Serial.printf("    %6lu baud: silent\n", (unsigned long)kBaudSweep[i]);
      } else {
        Serial.printf("    %6lu baud: %u bytes -> ", (unsigned long)kBaudSweep[i], (unsigned)n);
        printEscaped(reply, n);
        Serial.println();
        if (liveBaud == 0 && strstr(reply, "+OK") != nullptr) {
          liveBaud = kBaudSweep[i];
          liveRx = rxPins[orient];
          liveTx = txPins[orient];
        }
      }
    }
  }

  Serial.println();
  Serial.println("=== VERDICT ===");
  if (liveBaud != 0) {
    Serial.printf("MODULE IS ALIVE - +OK at %lu baud, RX=GPIO%d TX=GPIO%d.\n", (unsigned long)liveBaud,
                  liveRx, liveTx);
    Serial.println("The module is fine, so the fault is in the Module C PCB. Opening a bridge");
    Serial.println("below - type AT commands and they go straight to the module.");
  } else if (anyBytes > 0) {
    Serial.println("Module returned BYTES but never +OK. It is alive but not speaking the");
    Serial.println("expected protocol - suspect a baud/framing mismatch or altered firmware,");
    Serial.println("not dead hardware. The raw bytes above are the evidence to work from.");
  } else if (!padsOk) {
    Serial.println("INCONCLUSIVE - a pad failed phase A, so this rig cannot judge the module.");
  } else {
    Serial.println("MODULE IS SILENT on a board whose pads self-echoed, with the PCB entirely");
    Serial.println("out of the picture. Check VDD/GND at the module's own pins first (a missing");
    Serial.println("shared ground looks exactly like this); if those are good, the module is dead");
    Serial.println("and the Module C PCB is exonerated.");
  }
  // ---- Phase D: read the module's live configuration ----
  // Queries only, no writes: whatever this module is set to is ground truth, and
  // Module C's Config.h has to match it (or be told to set it) for the link to
  // work. Guessing at SF/BW/CR/preamble is what produced the +ERR=18 rejection
  // this project already hit once.
  if (liveBaud != 0) {
    Serial.println();
    Serial.println("--- phase D: live configuration (read-only) ---");
    const char *queries[] = {"AT+VER?", "AT+ADDRESS?", "AT+NETWORKID?",
                             "AT+BAND?", "AT+PARAMETER?", "AT+CRFOP?", "AT+IPR?"};
    LoRaSerial.begin(liveBaud, SERIAL_8N1, liveRx, liveTx);
    for (size_t i = 0; i < sizeof(queries) / sizeof(queries[0]); i++) {
      while (LoRaSerial.available()) LoRaSerial.read();
      LoRaSerial.print(queries[i]);
      LoRaSerial.print("\r\n");
      LoRaSerial.flush();

      char reply[96];
      size_t n = 0;
      uint32_t start = millis();
      while (millis() - start < 700 && n < sizeof(reply) - 1) {
        if (LoRaSerial.available()) reply[n++] = (char)LoRaSerial.read();
      }
      reply[n] = 0;
      Serial.printf("  %-14s -> ", queries[i]);
      if (n == 0) Serial.print("(no reply)");
      else printEscaped(reply, n);
      Serial.println();
    }
    LoRaSerial.end();
  }

  Serial.println();
  Serial.println("--- transparent bridge: type AT commands, Enter to send ---");

  if (liveBaud == 0) { liveBaud = 115200; liveRx = LORA_RX_PIN; liveTx = LORA_TX_PIN; }
  LoRaSerial.begin(liveBaud, SERIAL_8N1, liveRx, liveTx);
}

void loop() {
  while (LoRaSerial.available()) Serial.write(LoRaSerial.read());
  while (Serial.available()) LoRaSerial.write(Serial.read());
}
