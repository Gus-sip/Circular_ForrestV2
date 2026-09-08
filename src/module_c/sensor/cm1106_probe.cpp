/*
 * CM1106SL-NS diagnostic probe - why is the CO2 value frozen?
 *
 * Symptom (Module C PCB, 2026-09-04): the sensor answers every request with a
 * well-formed, checksum-valid frame whose counter byte advances, while the CO2
 * field itself stays pegged for an entire boot and only changes across power
 * cycles:
 *
 *     16 05 01 05 16 00 11 B8   CO2 = 0x0516 = 1302
 *     16 05 01 05 16 00 14 B5   [6] advancing, checksum tracking it
 *     16 05 01 05 16 00 1C AD
 *
 * Two candidate explanations, and this sketch separates them:
 *
 *   (a) Traffic/noise on the UART stopping a fresh value getting through. Argued
 *       against by the evidence - contention corrupts frames, and every frame
 *       here is clean - but phase 1 tests it directly rather than by assertion.
 *   (b) The part is SINGLE-SHOT. notes/power_budget.md lists this sensor as
 *       "BASE_ESCALADA - single-shot", which would mean 0x11 0x01 0x01 merely
 *       reads back the LAST result. Frozen within a boot and fresh after a power
 *       cycle is exactly what that produces.
 *
 * Only one command has ever been sent to this part in this project
 * (0x11 0x01 0x01, "read CO2"), so if a measurement trigger exists we have never
 * issued it. Phase 3 discovers the command set from the device instead of
 * guessing at an un-vendored datasheet - the same approach rylr998_param_probe
 * takes, and for the same reason.
 *
 * Frame format: 0x11 LEN CMD [data...] CS, where the sum of every byte including
 * CS is 0 mod 256. Responses come back as 0x16 LEN CMD [data...] CS.
 */

#include <Arduino.h>
#include "pins.h"
#include "Config.h"

// PCB rail gates - the CM1106 sits on the 5V rail, so nothing here works until
// these are asserted. LOW = rail on.
#define PCB_EN_A_PIN 10
#define PCB_EN_B_PIN 11
#define PCB_EN_SETTLE_MS 300

#define co2Serial Serial2

static uint8_t checksum(const uint8_t *frame, size_t n) {
  uint16_t sum = 0;
  for (size_t i = 0; i < n; i++) sum += frame[i];
  return (uint8_t)(256 - (sum % 256));
}

static void dumpFrame(const uint8_t *buf, size_t n) {
  for (size_t i = 0; i < n; i++) Serial.printf("%02X ", buf[i]);
}

// Sends one command and collects whatever comes back. Returns byte count.
static size_t ask(uint8_t cmd, uint8_t *resp, size_t respCap, uint32_t waitMs = 500) {
  uint8_t frame[4] = {0x11, 0x01, cmd, 0};
  frame[3] = checksum(frame, 3);

  while (co2Serial.available()) co2Serial.read();
  co2Serial.write(frame, sizeof(frame));
  co2Serial.flush();

  size_t n = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < waitMs && n < respCap) {
    if (co2Serial.available()) resp[n++] = co2Serial.read();
  }
  return n;
}

static bool frameValid(const uint8_t *r, size_t n) {
  if (n < 4) return false;
  uint16_t sum = 0;
  for (size_t i = 0; i < n; i++) sum += r[i];
  return (sum % 256) == 0;
}

static uint16_t co2Of(const uint8_t *r, size_t n) {
  if (n < 8) return 0;
  return (uint16_t)r[3] * 256 + r[4];
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  delay(500);

  Serial.println();
  Serial.println("=== CM1106SL-NS probe: frozen CO2 value ===");

  pinMode(PCB_EN_A_PIN, OUTPUT);
  pinMode(PCB_EN_B_PIN, OUTPUT);
  pinMode(PIN_CM1106_EN, OUTPUT);

  uint8_t resp[32];

  // ---- Gate sweep: which GPIO10/11 combination powers this sensor? ----
  // pins.h assumes both gates are active-LOW, but that has never been verified per
  // gate, and which one carries 5V is unknown. Since every working sensor is on 3V3
  // and the only silent connected one is on 5V, an active-HIGH gate driven LOW would
  // explain the silence exactly. Rather than reason about it, try all four.
  //
  // Each combination gets a full settle + EN + warm-up, because a rail coming up is
  // not instant and this sensor needs its warm-up before it answers at all.
  Serial.println("\n--- gate sweep: all four GPIO10/11 combinations ---");
  int liveA = -1, liveB = -1;
  {
    const int levels[4][2] = {{LOW, LOW}, {LOW, HIGH}, {HIGH, LOW}, {HIGH, HIGH}};
    for (int i = 0; i < 4; i++) {
      int a = levels[i][0], b = levels[i][1];

      // Fully de-power between attempts, so a sensor already awake from the previous
      // combination cannot be mistaken for one this combination powered.
      digitalWrite(PIN_CM1106_EN, LOW);
      digitalWrite(PCB_EN_A_PIN, HIGH);
      digitalWrite(PCB_EN_B_PIN, HIGH);
      delay(600);

      digitalWrite(PCB_EN_A_PIN, a);
      digitalWrite(PCB_EN_B_PIN, b);
      delay(PCB_EN_SETTLE_MS);
      digitalWrite(PIN_CM1106_EN, HIGH);

      co2Serial.end();
      co2Serial.begin(CM1106_BAUD, SERIAL_8N1, PIN_CM1106_RX, PIN_CM1106_TX);
      delay(CM1106_WARMUP_MS);

      Serial.printf("  GPIO%d=%s GPIO%d=%s : ", PCB_EN_A_PIN, a == HIGH ? "HIGH" : "LOW ",
                    PCB_EN_B_PIN, b == HIGH ? "HIGH" : "LOW ");

      size_t n = 0;
      for (int attempt = 0; attempt < 3 && n == 0; attempt++) {
        n = ask(0x01, resp, sizeof(resp), 600);
      }
      if (n == 0) {
        Serial.println("silent");
      } else {
        dumpFrame(resp, n);
        Serial.printf(" | CO2=%u %s\n", co2Of(resp, n),
                      frameValid(resp, n) ? "checksum OK" : "CHECKSUM BAD");
        if (liveA < 0) { liveA = a; liveB = b; }
      }
    }
  }

  if (liveA >= 0) {
    Serial.printf("\n  *** CM1106 ANSWERS with GPIO%d=%s GPIO%d=%s ***\n", PCB_EN_A_PIN,
                  liveA == HIGH ? "HIGH" : "LOW", PCB_EN_B_PIN, liveB == HIGH ? "HIGH" : "LOW");
    if (liveA == HIGH || liveB == HIGH) {
      Serial.println("  *** A gate is ACTIVE-HIGH - pins.h is wrong and needs correcting ***");
    }
  } else {
    Serial.println("\n  No combination woke it. Not a gate polarity problem.");
    liveA = LOW;
    liveB = LOW;
  }

  // Settle into whichever state worked (or the documented default) for the rest.
  digitalWrite(PCB_EN_A_PIN, liveA);
  digitalWrite(PCB_EN_B_PIN, liveB);
  delay(PCB_EN_SETTLE_MS);
  digitalWrite(PIN_CM1106_EN, HIGH);
  co2Serial.end();
  co2Serial.begin(CM1106_BAUD, SERIAL_8N1, PIN_CM1106_RX, PIN_CM1106_TX);
  Serial.printf("\nContinuing with GPIO%d=%s GPIO%d=%s, CM1106 on UART2 RX=GPIO%d TX=GPIO%d @ %d baud\n",
                PCB_EN_A_PIN, liveA == HIGH ? "HIGH" : "LOW", PCB_EN_B_PIN,
                liveB == HIGH ? "HIGH" : "LOW", PIN_CM1106_RX, PIN_CM1106_TX, CM1106_BAUD);
  delay(CM1106_WARMUP_MS);

  // ---- Phase 0: is the ESP side even working, and is anything out there? ----
  // Added after the sensor went fully silent (2026-09-07) where it had previously
  // answered with a frozen value. Before drawing any conclusion about the sensor,
  // establish that our own pads work and that something is driving the RX line.
  Serial.println("\n--- phase 0: line state and pad self-check ---");
  {
    // The sensor's TXD is a UART output idling HIGH. Drive an internal pulldown on
    // our RX pin and see who wins: still HIGH means something is actively driving
    // it (sensor powered and present); following the pulldown to LOW means nothing
    // is on the far end - unpowered, absent, or an open line.
    co2Serial.end();
    pinMode(PIN_CM1106_RX, INPUT_PULLDOWN);
    delay(30);
    int rxPd = digitalRead(PIN_CM1106_RX);
    pinMode(PIN_CM1106_RX, INPUT_PULLUP);
    delay(30);
    int rxPu = digitalRead(PIN_CM1106_RX);
    pinMode(PIN_CM1106_RX, INPUT);
    Serial.printf("  GPIO%d (RX <- sensor TXD): pulldown=%d pullup=%d -> %s\n", PIN_CM1106_RX,
                  rxPd, rxPu,
                  (rxPd == HIGH) ? "DRIVEN HIGH - sensor TXD appears alive"
                                 : "FLOATING - nothing driving it (unpowered/absent/open)");

    // Self-loopback: RX and TX muxed onto one pad, so the ESP reads back what it
    // drove. Proves the UART peripheral and the pads independently of the sensor.
    const int pads[2] = {PIN_CM1106_TX, PIN_CM1106_RX};
    for (int i = 0; i < 2; i++) {
      co2Serial.begin(CM1106_BAUD, SERIAL_8N1, pads[i], pads[i]);
      delay(60);
      while (co2Serial.available()) co2Serial.read();
      const uint8_t probe[4] = {0x11, 0x01, 0x01, 0xED};
      co2Serial.write(probe, sizeof(probe));
      co2Serial.flush();
      size_t n = 0;
      uint32_t t = millis();
      while (millis() - t < 300 && n < 16) {
        if (co2Serial.available()) { co2Serial.read(); n++; }
      }
      Serial.printf("  GPIO%d self-echo: %u bytes %s\n", pads[i], (unsigned)n,
                    n >= 4 ? "-> pad OK" : "-> NO ECHO (our UART/pad is at fault)");
      co2Serial.end();
    }
    co2Serial.begin(CM1106_BAUD, SERIAL_8N1, PIN_CM1106_RX, PIN_CM1106_TX);
    delay(50);
  }

  // ---- Phase 1: is anything else talking on this line? ----
  // Directly tests the "traffic on the line" theory. This part is request/response
  // and should say nothing unprompted, so ANY byte here is significant: it would
  // mean something else drives the line, or the sensor free-runs.
  Serial.println("\n--- phase 1: passive listen, 5s, nothing sent ---");
  {
    size_t n = 0;
    uint32_t start = millis();
    while (millis() - start < 5000) {
      if (co2Serial.available()) {
        if (n == 0) Serial.print("  unsolicited bytes: ");
        Serial.printf("%02X ", co2Serial.read());
        n++;
      }
    }
    if (n == 0) {
      Serial.println("  (silent - no unsolicited traffic, so nothing is talking over us)");
    } else {
      Serial.printf("\n  %u unsolicited bytes - SOMETHING is driving this line\n", (unsigned)n);
    }
  }

  // ---- Phase 2: does the value move on its own? ----
  Serial.println("\n--- phase 2: 10 reads over ~20s, does CO2 change? ---");
  uint16_t firstVal = 0, lastVal = 0;
  bool moved = false;
  for (int i = 0; i < 10; i++) {
    size_t n = ask(0x01, resp, sizeof(resp));
    Serial.printf("  read %2d: ", i + 1);
    if (n == 0) {
      Serial.println("(no reply)");
    } else {
      dumpFrame(resp, n);
      uint16_t v = co2Of(resp, n);
      Serial.printf(" | CO2=%u %s", v, frameValid(resp, n) ? "checksum OK" : "CHECKSUM BAD");
      if (i == 0) firstVal = v;
      else if (v != lastVal) moved = true;
      lastVal = v;
      Serial.println();
    }
    delay(2000);
  }
  Serial.printf("  -> value %s over 10 reads (first=%u last=%u)\n",
                moved ? "CHANGED" : "NEVER CHANGED", firstVal, lastVal);

  // ---- Phase 2b: baud sweep, only if nothing answered above ----
  // Total silence cannot distinguish "dead" from "talking at another rate".
  if (firstVal == 0 && lastVal == 0) {
    Serial.println("\n--- phase 2b: no reply at the configured baud - sweeping ---");
    const uint32_t bauds[] = {9600, 115200, 38400, 19200, 57600, 4800};
    for (size_t i = 0; i < sizeof(bauds) / sizeof(bauds[0]); i++) {
      co2Serial.end();
      co2Serial.begin(bauds[i], SERIAL_8N1, PIN_CM1106_RX, PIN_CM1106_TX);
      delay(80);
      size_t n = ask(0x01, resp, sizeof(resp), 600);
      Serial.printf("  %6lu baud: ", (unsigned long)bauds[i]);
      if (n == 0) Serial.println("silent");
      else { dumpFrame(resp, n); Serial.println(); }
    }
    co2Serial.end();
    co2Serial.begin(CM1106_BAUD, SERIAL_8N1, PIN_CM1106_RX, PIN_CM1106_TX);
    delay(50);
  }

  // ---- Phase 2c: both crossover orientations x baud ----
  // pins.h names these nets from the SENSOR's point of view (PIN_CM1106_TX 6 =
  // "ESP TX -> sensor RX"), while the LoRa nets on this same board are named from
  // the MCU's. That inconsistency already produced one wrong pin map on the
  // Calypso, so test both orientations rather than trusting either reading of
  // "rx is on GPIO6". A single +reply here settles it empirically.
  {
    Serial.println("\n--- phase 2c: both orientations x baud ---");
    const int rxPins[2] = {PIN_CM1106_RX, PIN_CM1106_TX};
    const int txPins[2] = {PIN_CM1106_TX, PIN_CM1106_RX};
    const uint32_t bauds[] = {9600, 115200, 38400, 19200, 57600, 4800};
    bool found = false;

    for (int orient = 0; orient < 2 && !found; orient++) {
      Serial.printf("  orientation %d: ESP RX=GPIO%d TX=GPIO%d\n", orient, rxPins[orient],
                    txPins[orient]);
      for (size_t i = 0; i < sizeof(bauds) / sizeof(bauds[0]); i++) {
        co2Serial.end();
        co2Serial.begin(bauds[i], SERIAL_8N1, rxPins[orient], txPins[orient]);
        delay(80);
        size_t n = ask(0x01, resp, sizeof(resp), 600);
        Serial.printf("    %6lu baud: ", (unsigned long)bauds[i]);
        if (n == 0) {
          Serial.println("silent");
        } else {
          dumpFrame(resp, n);
          Serial.printf(" | CO2=%u %s\n", co2Of(resp, n),
                        frameValid(resp, n) ? "checksum OK" : "CHECKSUM BAD");
          Serial.printf("\n  *** ANSWERS at %lu baud, ESP RX=GPIO%d TX=GPIO%d ***\n",
                        (unsigned long)bauds[i], rxPins[orient], txPins[orient]);
          if (rxPins[orient] != PIN_CM1106_RX) {
            Serial.println("  *** This is the SWAPPED orientation - pins.h has RX/TX reversed ***");
          }
          found = true;
          break;
        }
      }
    }
    if (!found) {
      Serial.println("  No reply in either orientation at any baud. The UART is not the");
      Serial.println("  problem - check 5V at the sensor's own VCC pin, and its GND.");
    }
    co2Serial.end();
    co2Serial.begin(CM1106_BAUD, SERIAL_8N1, PIN_CM1106_RX, PIN_CM1106_TX);
    delay(50);
  }

  // ---- Phase 3: what commands does this part actually answer? ----
  // Only 0x11 0x01 0x01 has ever been sent to it. If a "start measurement"
  // trigger exists, it is in here somewhere. Checksums are computed, so every
  // frame sent is well-formed; anything that answers is real.
  Serial.println("\n--- phase 3: command discovery, 0x00..0x2F ---");
  for (uint8_t cmd = 0x00; cmd <= 0x2F; cmd++) {
    size_t n = ask(cmd, resp, sizeof(resp), 300);
    if (n == 0) continue;  // silence is the normal answer; only report replies
    Serial.printf("  cmd 0x%02X -> %u bytes: ", cmd, (unsigned)n);
    dumpFrame(resp, n);
    Serial.printf(" %s\n", frameValid(resp, n) ? "(valid)" : "(bad checksum)");
    delay(120);
  }

  // ---- Phase 4: does a power cycle move the value? ----
  // If it does, and nothing in phase 3 did, the part is single-shot and measures
  // on power-up - which makes EN cycling the only way to get a fresh reading
  // until a trigger command is identified.
  Serial.println("\n--- phase 4: EN power-cycle, then re-read ---");
  {
    size_t n = ask(0x01, resp, sizeof(resp));
    uint16_t before = co2Of(resp, n);
    Serial.printf("  before cycle: CO2=%u\n", before);

    digitalWrite(PIN_CM1106_EN, LOW);
    delay(1000);
    digitalWrite(PIN_CM1106_EN, HIGH);
    Serial.printf("  EN cycled, warm-up %dms...\n", CM1106_WARMUP_MS);
    delay(CM1106_WARMUP_MS);

    n = ask(0x01, resp, sizeof(resp));
    uint16_t after = co2Of(resp, n);
    Serial.printf("  after cycle:  CO2=%u  -> %s\n", after,
                  after != before ? "CHANGED (power-up triggers a measurement)"
                                  : "unchanged");
  }

  Serial.println("\n--- continuous reads below, every 5s ---");
}

void loop() {
  static uint32_t last = 0;
  if (millis() - last < 5000) return;
  last = millis();

  uint8_t resp[32];
  size_t n = ask(0x01, resp, sizeof(resp));
  if (n == 0) {
    Serial.println("(no reply)");
    return;
  }
  dumpFrame(resp, n);
  Serial.printf(" | CO2=%u %s\n", co2Of(resp, n), frameValid(resp, n) ? "" : "CHECKSUM BAD");
}
