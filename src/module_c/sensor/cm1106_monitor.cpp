/*
 * CM1106SL-NS long-run monitor - does it actually read, over time?
 *
 * The part is single-shot (established 2026-09-08): 0x11 0x01 0x01 reads back the
 * LAST measurement and never triggers a new one, and the sensor measures on
 * power-up. So the only way to get a fresh value is to cycle EN and wait the
 * warm-up. This sketch does that on a fixed interval and logs the result, which is
 * exactly the strategy the production firmware needs to adopt.
 *
 * Each cycle logs TWO reads:
 *   fresh  - taken after an EN power cycle
 *   stale  - taken immediately afterwards with no cycle
 *
 * Logging both makes the single-shot behaviour visible rather than asserted: the
 * fresh column should move with the room, the stale column should equal the fresh
 * one from the same cycle, and neither read ever signals staleness on its own -
 * checksums pass and the counter byte advances either way. That is the trap this
 * sensor sets, and the reason a reading must never be trusted just because the
 * frame is valid.
 *
 * Rails: GPIO10 (3V3) and GPIO11 (5V) have OPPOSITE polarity - see pins.h. The
 * CM1106 is on 5V, so GPIO11 must be HIGH or nothing here works at all.
 */

#include <Arduino.h>
#include "pins.h"
#include "Config.h"

#define co2Serial Serial2

// One reading every this often. Long enough that a real CO2 change can show up,
// short enough to gather a useful series in a few minutes.
#define MONITOR_INTERVAL_MS 20000UL

// How long EN is held low to force a genuine power-down before the next
// measurement. Too short and the sensor may not actually restart.
#define EN_OFF_MS 1000UL

static uint8_t checksum(const uint8_t *frame, size_t n) {
  uint16_t sum = 0;
  for (size_t i = 0; i < n; i++) sum += frame[i];
  return (uint8_t)(256 - (sum % 256));
}

static bool frameValid(const uint8_t *r, size_t n) {
  if (n < 4) return false;
  uint16_t sum = 0;
  for (size_t i = 0; i < n; i++) sum += r[i];
  return (sum % 256) == 0;
}

// Returns bytes received; CO2 in *out when a full valid frame arrived.
static size_t readCo2(uint16_t *out, uint8_t *counterOut) {
  uint8_t frame[4] = {0x11, 0x01, 0x01, 0};
  frame[3] = checksum(frame, 3);

  while (co2Serial.available()) co2Serial.read();
  co2Serial.write(frame, sizeof(frame));
  co2Serial.flush();

  uint8_t resp[16];
  size_t n = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < 600 && n < sizeof(resp)) {
    if (co2Serial.available()) resp[n++] = co2Serial.read();
  }
  if (n >= 8 && frameValid(resp, n)) {
    *out = (uint16_t)resp[3] * 256 + resp[4];
    *counterOut = resp[6];
  }
  return n;
}

static void powerCycleSensor() {
  digitalWrite(PIN_CM1106_EN, LOW);
  delay(EN_OFF_MS);
  digitalWrite(PIN_CM1106_EN, HIGH);
  delay(CM1106_WARMUP_MS);
  while (co2Serial.available()) co2Serial.read();
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  delay(500);

  pinMode(PIN_PCB_EN_A, OUTPUT);
  digitalWrite(PIN_PCB_EN_A, PIN_PCB_EN_A_ACTIVE);
  pinMode(PIN_PCB_EN_B, OUTPUT);
  digitalWrite(PIN_PCB_EN_B, PIN_PCB_EN_B_ACTIVE);
  delay(PIN_PCB_EN_SETTLE_MS);

  pinMode(PIN_CM1106_EN, OUTPUT);
  digitalWrite(PIN_CM1106_EN, HIGH);
  co2Serial.begin(CM1106_BAUD, SERIAL_8N1, PIN_CM1106_RX, PIN_CM1106_TX);
  delay(CM1106_WARMUP_MS);

  Serial.println();
  Serial.println("=== CM1106 long-run monitor ===");
  Serial.printf("Rails: GPIO%d=%s (3V3), GPIO%d=%s (5V)\n", PIN_PCB_EN_A,
                PIN_PCB_EN_A_ACTIVE == LOW ? "LOW" : "HIGH", PIN_PCB_EN_B,
                PIN_PCB_EN_B_ACTIVE == LOW ? "LOW" : "HIGH");
  Serial.printf("EN=GPIO%d, UART2 RX=GPIO%d TX=GPIO%d @ %d baud\n", PIN_CM1106_EN, PIN_CM1106_RX,
                PIN_CM1106_TX, CM1106_BAUD);
  Serial.printf("One reading every %lus, each preceded by an EN power cycle.\n\n",
                (unsigned long)(MONITOR_INTERVAL_MS / 1000));
  Serial.println("   t     fresh   stale   ctr   note");
  Serial.println("  ----   -----   -----   ---   ----");
}

void loop() {
  static uint32_t next = 0;
  static uint16_t prevFresh = 0;
  static bool havePrev = false;
  static uint32_t n = 0;

  if (millis() < next) return;
  next = millis() + MONITOR_INTERVAL_MS;
  n++;

  // Fresh: force a new measurement, then read it.
  powerCycleSensor();
  uint16_t fresh = 0;
  uint8_t ctr = 0;
  size_t got = readCo2(&fresh, &ctr);

  if (got == 0) {
    Serial.printf("  %4lu   ----    ----    ---   NO REPLY (check 5V rail / GPIO%d)\n",
                  (unsigned long)(millis() / 1000), PIN_PCB_EN_B);
    return;
  }

  // Stale: same request again with no cycle in between. Should equal fresh.
  uint16_t stale = 0;
  uint8_t ctr2 = 0;
  readCo2(&stale, &ctr2);

  const char *note = "";
  if (!havePrev) {
    note = "first";
  } else if (fresh == prevFresh) {
    note = "same as last cycle";
  } else {
    note = (fresh > prevFresh) ? "rising" : "falling";
  }
  prevFresh = fresh;
  havePrev = true;

  Serial.printf("  %4lu   %5u   %5u   %3u   %s%s\n", (unsigned long)(millis() / 1000), fresh, stale,
                ctr2, note, (stale != fresh) ? "  [!] stale != fresh" : "");
}
