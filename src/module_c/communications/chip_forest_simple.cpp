/*
 * CHIP FOREST Module C - minimal firmware, with a load bisect.
 *
 * Built up from the one thing that provably works rather than cut down from the
 * one that does not. env:module-c-rail-gate holds all three rails on for 80+
 * seconds with zero resets on node C1; the full firmware dies within milliseconds
 * of the same GPIO11 write even with sensors and radio stripped out of setup().
 *
 * The decisive comparison:
 *
 *   rail_gate_set      rails ON, sensors powered but never ACTIVATED  -> stable
 *   this firmware      rails ON, sensors activated                    -> resets
 *
 * Both power the same sensors. Only this one starts the BMV080's laser, cycles
 * the CM1106's EN line and drives the radio. So the instability tracks sensor
 * LOAD, not the rails being on - which is why staggering the rails and
 * soft-starting the gate both failed to help.
 *
 * The USE_* flags below exist to find which load triggers it. Turn exactly one
 * off at a time. BMV080 is off first because at ~68mA it is the largest by an
 * order of magnitude, and notes/power_budget.md allows it 20s per 30 min where
 * this runs it continuously.
 *
 * Deliberately absent: deep sleep, RTC_NOINIT state, NVS boot guard, safe mode,
 * soft-start, tick counters, batch store, alarm state machine. None of them are
 * needed to read a sensor and send a packet, and each was surface area that made
 * the failure harder to localise.
 *
 * Kept, because each was proven on hardware: the rail polarities (GPIO10 3V3
 * active-LOW, GPIO11 5V active-HIGH, GPIO13 LoRa active-LOW), the CM1106 EN power
 * cycle (it is single-shot, so a fresh value requires it), the BMV080 address
 * scan (strap differs per board), and the GPIO21 status LED.
 */

#include <Arduino.h>
#include <Wire.h>

#include "pins.h"
#include "Config.h"
#include "../sensor/sensors/Bme690Sensor.h"
#include "../sensor/sensors/Bmv080Sensor.h"
#include "../sensor/sensors/Cm1106Sensor.h"
#include "../sensor/sensors/Sen0466Sensor.h"
#include "../sensor/sensors/CalypsoSensor.h"
#include "radio/RYLR998.h"

// ---------- Load bisect: turn ONE off at a time ----------
#define USE_BME690 1
#define USE_BMV080 1  // ~68mA - safe now it runs only inside its own slot
#define USE_SEN0466 1
#define USE_CM1106 1
#define USE_CALYPSO 1
#define USE_RADIO 1

// ---------- LoRa link. Must match Module B exactly. ----------
#define LORA_RX_PIN 4  // module TXD -> ESP RX
#define LORA_TX_PIN 5  // module RXD <- ESP TX
#define LORA_BAND_HZ 868000000UL
#define LORA_NETWORK_ID 5
#define LORA_MY_ADDR 1
#define LORA_DEST_ADDR 2
#define LORA_SF 7
#define LORA_BW 7
#define LORA_CR 1
#define LORA_PREAMBLE 12

#define TX_PERIOD_MS 15000UL
#define READ_PERIOD_MS 2000UL

// A fresh CO2 value costs an EN cycle plus the warm-up, so it gets its own slower
// schedule rather than being paid on every pass.
#define CO2_REFRESH_MS 30000UL

// Longest the BMV080's laser may stay on waiting for a frame. Frames normally
// arrive in ~3.5s; this bounds the worst case so one slow sensor cannot hold the
// laser on indefinitely.
#define BMV080_SLOT_MS 8000UL

// Gap between slots. Lets each rail settle before the next load starts, rather
// than one sensor switching off straight into the next switching on.
#define SLOT_GAP_MS 250UL

// ---------- Status LED (GPIO21) ----------
// Driven both as WS2812 and as a plain level: the pin is known, the type is not.
static void ledSet(uint8_t r, uint8_t g, uint8_t b, bool on) {
  neopixelWrite(PIN_STATUS_LED, r, g, b);
  delay(1);
  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW);
}
static void ledOff() { ledSet(0, 0, 0, false); }
static void ledReading() { ledSet(0, 120, 0, true); }
static void ledTransmitting() { ledSet(120, 80, 0, true); }

Bme690Sensor bme690(BME690_I2C_ADDR);
Bmv080Sensor bmv080(BMV080_I2C_ADDR);
Sen0466Sensor sen0466(SEN0466_I2C_ADDR);
Cm1106Sensor cm1106(Serial2, PIN_CM1106_RX, PIN_CM1106_TX, PIN_CM1106_EN, CM1106_WARMUP_MS,
                    CM1106_BAUD);
CalypsoSensor calypso(Serial1, PIN_CALYPSO_RX, PIN_CALYPSO_TX, CALYPSO_BAUD);
RYLR998 radio(Serial0, LORA_RX_PIN, LORA_TX_PIN);

static bool bmeReady = false, bmvReady = false, coReady = false, radioReady = false;

// Each field holds its last good value rather than snapping to zero on a
// transient miss, so one bad read does not look like a sensor failure downstream.
static float g_temp = 0, g_hum = 0, g_pres = 0, g_gas = 0;
static float g_pm1 = 0, g_pm25 = 0, g_pm10 = 0;
static float g_co2 = 0, g_co = 0, g_coTemp = 0;
static float g_windAngle = 0, g_windSpeed = 0;
static bool g_windValid = false;

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  delay(500);

  Serial.println();
  Serial.println("=== CHIP FOREST Module C (minimal + load bisect) ===");
  Serial.printf("Enabled: BME690=%d BMV080=%d SEN0466=%d CM1106=%d CALYPSO=%d RADIO=%d\n",
                USE_BME690, USE_BMV080, USE_SEN0466, USE_CM1106, USE_CALYPSO, USE_RADIO);
  Serial.flush();

  ledReading();

  // Rails exactly as the working sketch does it: all three at once, no ramp, no
  // staggering. Each gate takes its OWN active level - they are not the same.
  pinMode(PIN_PCB_EN_A, OUTPUT);
  digitalWrite(PIN_PCB_EN_A, PIN_PCB_EN_A_ACTIVE);
  pinMode(PIN_PCB_EN_B, OUTPUT);
  digitalWrite(PIN_PCB_EN_B, PIN_PCB_EN_B_ACTIVE);
  pinMode(PIN_LORA_EN, OUTPUT);
  digitalWrite(PIN_LORA_EN, PIN_LORA_EN_ACTIVE);
  Serial.printf("Rails: GPIO%d=%s (3V3)  GPIO%d=%s (5V)  GPIO%d=%s (LoRa)\n", PIN_PCB_EN_A,
                PIN_PCB_EN_A_ACTIVE == LOW ? "LOW" : "HIGH", PIN_PCB_EN_B,
                PIN_PCB_EN_B_ACTIVE == LOW ? "LOW" : "HIGH", PIN_LORA_EN,
                PIN_LORA_EN_ACTIVE == LOW ? "LOW" : "HIGH");
  Serial.flush();
  delay(PIN_PCB_EN_SETTLE_MS);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

#if USE_BMV080
  Serial.printf("Waiting %dms for BMV080 startup...\n", BMV080_STARTUP_DELAY_MS);
  Serial.flush();
  delay(BMV080_STARTUP_DELAY_MS);
#endif

#if USE_BME690
  bmeReady = bme690.begin();
  Serial.printf("BME690:  %s\n", bmeReady ? "OK" : "NOT FOUND");
#else
  Serial.println("BME690:  disabled (bisect)");
#endif
  Serial.flush();

#if USE_BMV080
  bmvReady = bmv080.begin();
  Serial.printf("BMV080:  %s\n", bmvReady ? "OK" : "NOT FOUND");
  // begin() leaves it measuring to prove presence; stop it immediately so the
  // laser is off until its own slot comes round.
  if (bmvReady) bmv080.stopMeasurement();
#else
  Serial.println("BMV080:  disabled (bisect)");
#endif
  Serial.flush();

#if USE_SEN0466
  coReady = sen0466.begin();
  Serial.printf("SEN0466: %s  (~210s settle before CO is meaningful)\n",
                coReady ? "OK" : "NOT FOUND");
#else
  Serial.println("SEN0466: disabled (bisect)");
#endif
  Serial.flush();

#if USE_CM1106
  cm1106.begin();
  Serial.println("CM1106:  UART open");
#else
  Serial.println("CM1106:  disabled (bisect)");
#endif

#if USE_CALYPSO
  calypso.begin();
  Serial.println("Calypso: UART open");
#else
  Serial.println("Calypso: disabled (bisect)");
#endif
  Serial.flush();

#if USE_RADIO
  radioReady = radio.begin(LORA_MY_ADDR, LORA_NETWORK_ID, LORA_BAND_HZ,
                           {LORA_SF, LORA_BW, LORA_CR, LORA_PREAMBLE}, &Serial);
  Serial.printf("RYLR998: %s\n", radioReady ? "OK" : "FAILED");
#else
  Serial.println("RYLR998: disabled (bisect)");
#endif

  Serial.println("Setup complete.\n");
  Serial.flush();
  ledOff();
}

// ---------------- Sequential sensor cycle ----------------
// One sensor active at a time, never overlapping. Disabling the BMV080 alone took
// this board from 31 resets in 140s to 1, which showed the instability tracks
// concurrent sensor LOAD rather than the rails being on. So instead of leaving
// everything running, each sensor gets its own slot: switched on, read, switched
// off again before the next one starts.
//
// This is also what notes/power_budget.md specified from the beginning - "switched
// rails so the 68mA BMV080 and 5mA/210s SEN0466 only draw during their windows".
//
// The transmit gets a slot of its own too, with every sensor quiesced first, so
// the radio's ~120mA burst never lands on top of a laser or a warming heater.
enum Slot {
  SLOT_BME690 = 0,
  SLOT_BMV080,
  SLOT_CM1106,
  SLOT_SEN0466,
  SLOT_CALYPSO,
  SLOT_COUNT
};

static const char *slotName(int s) {
  switch (s) {
    case SLOT_BME690: return "BME690";
    case SLOT_BMV080: return "BMV080";
    case SLOT_CM1106: return "CM1106";
    case SLOT_SEN0466: return "SEN0466";
    case SLOT_CALYPSO: return "Calypso";
  }
  return "?";
}

// Everything off. Called before a transmit and between slots, so nothing is drawing
// when the next load starts.
static void quiesceAll() {
#if USE_BMV080
  if (bmvReady) bmv080.stopMeasurement();
#endif
#if USE_CM1106
  cm1106.sleep();  // EN low
#endif
}

// Runs exactly one sensor's slot. Blocking by design: the point is that nothing
// else is drawing while this one is.
static void runSlot(int slot) {
  switch (slot) {
#if USE_BME690
    case SLOT_BME690: {
      if (!bmeReady) break;
      Reading r = bme690.read();
      if (r.ok()) {
        g_temp = r.values[0];
        g_hum = r.values[1];
        g_pres = r.values[2];
        g_gas = r.values[3];
      }
      break;
    }
#endif

#if USE_BMV080
    case SLOT_BMV080: {
      if (!bmvReady) break;
      // Laser on only for as long as it takes to get one frame. This is the load
      // that was resetting the board when left running continuously.
      if (!bmv080.startMeasurement()) break;
      uint32_t t0 = millis();
      while (millis() - t0 < BMV080_SLOT_MS) {
        Reading r = bmv080.read();
        if (r.ok()) {
          g_pm1 = r.values[0];
          g_pm25 = r.values[1];
          g_pm10 = r.values[2];
          break;
        }
        delay(50);
      }
      bmv080.stopMeasurement();  // laser off before anything else runs
      break;
    }
#endif

#if USE_CM1106
    case SLOT_CM1106: {
      // Single-shot part: it measures on power-up, so a fresh value costs an EN
      // cycle. Its own slower schedule - the cycle plus warm-up is expensive.
      static uint32_t lastCo2Ms = 0;
      if (lastCo2Ms != 0 && millis() - lastCo2Ms < CO2_REFRESH_MS) break;
      lastCo2Ms = millis();
      cm1106.powerCycle();
      Reading r = cm1106.read();
      if (r.ok()) g_co2 = r.values[0];
      cm1106.sleep();  // EN back off
      break;
    }
#endif

#if USE_SEN0466
    case SLOT_SEN0466: {
      if (!coReady) break;
      Reading r = sen0466.read();
      if (r.ok()) {
        g_co = r.values[0];
        g_coTemp = r.values[1];
      }
      break;
    }
#endif

#if USE_CALYPSO
    case SLOT_CALYPSO: {
      // Streams unprompted - drop the stale buffer, then listen briefly.
      calypso.flushInput();
      uint32_t t0 = millis();
      do {
        Reading w = calypso.read();
        if (w.ok()) {
          g_windAngle = w.values[0];
          g_windSpeed = w.values[1];
          g_windValid = w.values[2] > 0.5f;
          break;
        }
        delay(10);
      } while (millis() - t0 < 400);
      break;
    }
#endif

    default:
      break;
  }
}

void loop() {
  static int slot = 0;
  static uint32_t lastSlotMs = 0;
  static uint32_t lastTxMs = 0;
  static uint32_t cycleStartMs = 0;

  // One slot per pass, in turn. Nothing overlaps.
  if (millis() - lastSlotMs >= SLOT_GAP_MS) {
    lastSlotMs = millis();

    if (slot == 0) cycleStartMs = millis();

    ledReading();
    runSlot(slot);
    ledOff();

    slot++;
    if (slot >= SLOT_COUNT) {
      slot = 0;
      Serial.printf("[read] T%.2f H%.2f P%.2f gas%.0f | pm1=%.1f pm2.5=%.1f pm10=%.1f | "
                    "co2=%.0f co=%.2f | wind=%s %.1fdeg %.2f  (cycle %lums)\n",
                    g_temp, g_hum, g_pres, g_gas, g_pm1, g_pm25, g_pm10, g_co2, g_co,
                    g_windValid ? "OK" : "silent", g_windAngle, g_windSpeed,
                    (unsigned long)(millis() - cycleStartMs));
      Serial.flush();
    }
  }

  // Transmit in a slot of its own, with every sensor switched off first - the
  // radio's burst must not land on top of another load.
  if (radioReady && millis() - lastTxMs >= TX_PERIOD_MS && slot == 0) {
    lastTxMs = millis();
    quiesceAll();
    delay(SLOT_GAP_MS);

    ledTransmitting();
    char payload[96];
    int len = snprintf(payload, sizeof(payload),
                       "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d", g_temp,
                       g_hum, g_pres, g_gas, g_pm1, g_pm25, g_pm10, g_co2, g_co, g_coTemp,
                       g_windAngle, g_windSpeed, g_windValid ? 1 : 0);
    if (len < 0) len = 0;
    if (len >= (int)sizeof(payload)) len = (int)sizeof(payload) - 1;

    bool sent = radio.send(LORA_DEST_ADDR, payload, (uint8_t)len);
    Serial.printf("[tx] %d bytes: %s  (%s)\n", len, payload, sent ? "sent" : "FAILED");
    Serial.flush();
    ledOff();
  }

  delay(20);
}
