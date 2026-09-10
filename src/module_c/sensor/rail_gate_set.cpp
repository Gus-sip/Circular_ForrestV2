/*
 * Rail-gate manual set - holds GPIO10/11 at chosen levels so the rails can be
 * metered.
 *
 * Context: pins.h documents both as high-side transistor gates with LOW = rail on,
 * but which one gates 5V and which gates 3V3 has never been confirmed, and neither
 * has the polarity of each individually. That matters right now: every working
 * sensor is on 3V3, and the only silent connected one (CM1106) is on 5V. If one of
 * these gates is actually active-HIGH, driving it LOW has been holding its rail off.
 *
 * This sketch takes no measurements of its own - it just parks the pins and says
 * what it did, so a meter on the rails is the instrument. Serial stays open and the
 * board never sleeps, so the state below holds indefinitely.
 *
 * Edit the two levels here, reflash, measure. HIGH_LEVEL/LOW_LEVEL are spelled out
 * rather than using pins.h's PIN_PCB_EN_ACTIVE, precisely because the point is to
 * test that assumption rather than inherit it.
 */

#include <Arduino.h>
#include "pins.h"

// Status LED as a "firmware is running" indicator, readable with no USB attached.
// This matters because the observable that led here was GPIO11 failing to go high
// on the board's own supply - and that pin is driven within a second of boot, so
// it staying low means the code never ran. Without a serial port there was no way
// to tell "not booting" from "booting but the gate is not driving". The LED
// answers that: lit means setup() ran and the gates below are being held.
//
// Driven both as WS2812 and as a plain level, since the pin is known but the type
// is not. Green, moderate brightness.
static void ledOn() {
  neopixelWrite(PIN_STATUS_LED, 0, 120, 30);
  delay(1);
  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, HIGH);
}

#define GATE_A_PIN 10
#define GATE_B_PIN 11
#define GATE_C_PIN 13  // RYLR998 LoRa supply gate

// What each gate is driven to. Change, reflash, measure.
#define GATE_A_LEVEL LOW   // GPIO10 - left at the documented "on" level
#define GATE_B_LEVEL HIGH  // GPIO11 - 5V rail ON. The "short" this was parked away
                           // from turned out to be sleep-cycle inrush, not a short:
                           // C1 runs fine with 5V on when SLEEP_ENABLED is 0.
#define GATE_C_LEVEL LOW   // GPIO13 - LoRa rail ON (this gate is active-low)

static const char *lvl(int v) { return v == HIGH ? "HIGH (3.3V)" : "LOW (0V)"; }

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  delay(500);

  // LED first, before anything else - so it lights even if a later step hangs.
  ledOn();

  pinMode(GATE_A_PIN, OUTPUT);
  digitalWrite(GATE_A_PIN, GATE_A_LEVEL);
  pinMode(GATE_B_PIN, OUTPUT);
  digitalWrite(GATE_B_PIN, GATE_B_LEVEL);
  pinMode(GATE_C_PIN, OUTPUT);
  digitalWrite(GATE_C_PIN, GATE_C_LEVEL);

  Serial.println();
  Serial.println("=== Rail gate manual set ===");
  Serial.printf("  GPIO%d = %s\n", GATE_A_PIN, lvl(GATE_A_LEVEL));
  Serial.printf("  GPIO%d = %s\n", GATE_B_PIN, lvl(GATE_B_LEVEL));
  Serial.printf("  GPIO%d = %s\n", GATE_C_PIN, lvl(GATE_C_LEVEL));
  Serial.println();
  Serial.println("Held indefinitely - no sleep, no sensor init, nothing else driven.");
  Serial.println("Meter the 5V and 3V3 rails now.");
  Serial.println();
  Serial.println("pins.h documents LOW = rail on for both. If a rail is UP with its");
  Serial.println("gate HIGH, that gate is active-high and pins.h is wrong for it.");
}

void loop() {
  // Re-assert every second. Cheap insurance against anything else touching the
  // pads, and the heartbeat confirms the board is still alive and holding.
  static uint32_t last = 0;
  if (millis() - last < 1000) return;
  last = millis();

  digitalWrite(GATE_A_PIN, GATE_A_LEVEL);
  digitalWrite(GATE_B_PIN, GATE_B_LEVEL);
  digitalWrite(GATE_C_PIN, GATE_C_LEVEL);
  ledOn();

  static uint32_t secs = 0;
  if (++secs % 10 == 0) {
    Serial.printf("[hold] t=%lus  GPIO%d=%s  GPIO%d=%s  GPIO%d=%s\n", (unsigned long)secs,
                  GATE_A_PIN, lvl(GATE_A_LEVEL), GATE_B_PIN, lvl(GATE_B_LEVEL), GATE_C_PIN,
                  lvl(GATE_C_LEVEL));
  }
}
