/*
 * CHIP FOREST bring-up + LoRa TX: all five sensors read each cycle, encoded
 * into a CSV payload, and transmitted over the RYLR998 to the ground-station
 * receiver (the separate pp1-lora-receiver project). Between cycles the node
 * light-sleeps (see SleepManager) rather than busy-looping.
 *
 * Payload schema (must match pp1-lora-receiver's TelemetryParser exactly):
 *   temp,hum,pres,gas,pm1,pm25,pm10,co2,co,coTemp,windAngle,windSpeed,windValid
 * See that project's src/telemetry/TelemetryParser.h for the authoritative
 * field list/order - provisional on both ends; if either side changes this
 * encoding, update the other to match.
 *
 * LoRa parameters (SF9/BW7/CR1/preamble12): NOT the generic "commonly
 * documented" RYLR998 defaults this file originally assumed (7,7,1,4) - that
 * combination came back +ERR=18 from this actual module. These values were
 * read directly off the module via AT+PARAMETER? (src/rylr998_param_probe.cpp)
 * instead of guessed a second time - ground truth beats another guess.
 *
 * TX interval and duty cycle: EU863-870 SRD regulations cap airtime on this
 * band, as low as 1% in the commonly used 868.0-868.6MHz sub-band that this
 * project's AT+BAND=868000000 falls in. Using Semtech's public LoRa airtime
 * formula (not measured on this exact module - treat as an estimate) at
 * SF7/BW125kHz(index 7 on this module)/CR 4/5/preamble 12 with an ~75-byte
 * payload:
 *   symbol time     = 2^7/125000s                         ~= 1.024ms
 *   preamble time   = (12 + 4.25) * 1.024ms               ~= 16.6ms
 *   payload symbols = 8 + ceil((8*75 - 4*7 + 28 + 16) / (4*7)) * 5 = 8 + 22*5 = 118
 *   payload time    = 118 * 1.024ms                       ~= 120.8ms
 *   total per packet ~= 137ms
 * A 1% duty cycle allows ~36s of airtime per hour. At TX_INTERVAL_MS = 15s
 * that's 240 packets/hour * 137ms ~= 33s/hour - under the 36s cap, ~9%
 * margin. This is TIGHT: it only holds at SF7 and this payload size. The old
 * SF9 setting (468ms/packet) made 15s a 3x duty-cycle violation - SF7 was
 * chosen specifically to make this cadence legal, at a real cost of ~5-6dB
 * link budget (roughly half the range, worse through foliage - no field
 * range test has confirmed the deployment still closes at SF7). If you
 * revert to SF9, you MUST raise TX_INTERVAL_MS back to >=50s. The interval
 * is runtime-mutable via downlink CFG (see below) but is always clamped to
 * TX_INTERVAL_MIN_MS so a remote push can't drive it under the legal floor -
 * recompute the floor above before touching SF, BW, payload size, or that
 * clamp: this is a legal constraint, not a tunable.
 *
 * Cellular-data cost note (Module B's NB-IoT uplink, not this link): at 15s
 * this node emits ~5760 readings/day, which Module B batches to ThingsBoard
 * roughly every 2 min. That is ~1.5MB/day of cellular traffic - fine for
 * attended/bench use, but it exhausts a 1NCE lifetime SIM (~500MB total) in
 * about a year. A long-term deployment should uplink 1-min averages from
 * Module B instead of every raw sample (not done here).
 *
 * Sleep cycle and remote config-push: the node light-sleeps between cycles
 * (SleepManager) rather than busy-looping, waking only on its own timer -
 * this is a scheduled Class-A-style cycle, not an asynchronous listener.
 * Each cycle: wake, read sensors, transmit telemetry, then open a short
 * LORA_POST_TX_LISTEN_MS window to receive a config command from Module B
 * (if one is queued), reply with an ACK if one was applied, then sleep
 * again. This is the *only* moment Module C can be reached - whatever
 * relays Module A's commands down to it must hold/queue a pending command
 * and send it the instant it sees this node's uplink, not at an arbitrary
 * time. Light sleep (not deep sleep) is deliberate even though the radio no
 * longer needs to stay listening: deep sleep would force a full reboot each
 * cycle, repaying BMV080's 5s startup delay + CM1106's 3s EN warmup + the
 * RYLR998's AT handshake every 120s, and risking BMV080 (which needs
 * "several seconds of continuous operation" per its own driver header)
 * never producing a properly stabilized reading. Light sleep preserves
 * RAM/peripheral driver state across cycles, so all of that only happens
 * once at boot - see SleepManager.h.
 *
 * Downlink command grammar (Module B -> this node), styled to match
 * pp1-lora-receiver's own Module A->B grammar (NbiotProtocol.h) so a future
 * relay is a thin translation rather than two incompatible formats:
 *   CFG,<key>=<value>[,<key>=<value>...]
 * Recognized keys: INTERVAL (seconds, clamped to
 * [TX_INTERVAL_MIN_MS/1000, TX_INTERVAL_MAX_MS/1000]), and one per sensor -
 * BME690/SEN0466/BMV080/CM1106/CALYPSO (0/1) - to mute/unmute it without a
 * reflash (e.g. BME690, which is known-not-detected on this board per
 * STATUS.md). Unrecognized keys are ignored, not fatal. Applied config is
 * volatile only - it resets to these compiled-in defaults on every reboot,
 * by design (no NVS persistence for this pass). Only commands from
 * LORA_RX_ADDR (Module B) are processed; this is basic sender-address
 * filtering, not authentication - no auth layer exists anywhere in this
 * system yet. On success this node replies ACK,<key>=<value>,... with the
 * actually-applied (post-clamp) values, so a clamp is visible to the sender
 * instead of silently assumed away. A command that doesn't parse as CFG, or
 * applies nothing recognized, gets no ACK - airtime isn't spent acking noise.
 *
 * Missing/not-yet-ready sensor readings: each field keeps its last
 * successfully read value rather than snapping to zero on a transient miss,
 * so one bad cycle doesn't look identical to the sensor genuinely reading
 * zero. A sensor that has never read successfully sends 0. This means the
 * CSV payload alone can't distinguish "sensor briefly busy" from "sensor
 * stuck since boot" - only whether packets are arriving at all, which the
 * receiver's own staleness timer already covers. Fine for this milestone.
 */

#include <Arduino.h>
#include <Wire.h>
#include <string.h>
#include <stdlib.h>
#include "../sensor/pins.h"
#include "../sensor/Config.h"
#include "../sensor/PowerManager.h"
#include "../sensor/SleepManager.h"
#include "../sensor/sensors/Bme690Sensor.h"
#include "../sensor/sensors/Sen0466Sensor.h"
#include "../sensor/sensors/Cm1106Sensor.h"
#include "../sensor/sensors/CalypsoSensor.h"
#include "../sensor/sensors/Bmv080Sensor.h"
#include "radio/RYLR998.h"

// ---------- RYLR998 wiring - GPIO4/5 are unused by pins.h's sensor map ----------
#define LORA_RX_PIN 4  // module TXD -> ESP RX
#define LORA_TX_PIN 5  // module RXD <- ESP TX

// ---------- LoRa link parameters - MUST match pp1-lora-receiver's Config.h ----------
#define LORA_BAND_HZ 868000000UL  // EU868, Spain deployment (confirmed)
#define LORA_NETWORK_ID 5
// SF7 (was SF9): halves airtime enough to make the 15s TX_INTERVAL_MS below
// legal under the EU868 1% duty cycle - see the file-header airtime math.
// Costs ~5-6dB link budget vs SF9. MUST match Module B's Config.h
// (LORA_PARAM_SF) exactly. Re-verify AT+PARAMETER=7,7,1,12 comes back +OK on
// real hardware - this SF/BW/CR/preamble combo has not been confirmed on
// this module (an earlier 7,7,1,4 attempt returned +ERR=18).
#define LORA_PARAM_SF 7
#define LORA_PARAM_BW 7
#define LORA_PARAM_CR 1
#define LORA_PARAM_PREAMBLE 12
#define LORA_MY_ADDR 1  // this node's AT+ADDRESS
#define LORA_RX_ADDR 2  // pp1-lora-receiver's AT+ADDRESS

// 15s cadence (2026-08-28): fast sampling for a responsive live view. Legal
// under the EU868 1% duty cycle ONLY at SF7 with this payload size - see the
// file-header airtime math (~33s airtime/hour vs the ~36s cap, ~9% margin).
// The 15000 floor below reflects that: at SF7 it's the tightest cadence that
// stays legal; do NOT lower it, and raise it back to >=50000 if SF ever goes
// back to 9. See the header's cellular-data-cost note before running this
// unattended for long.
#define TX_INTERVAL_MS 15000           // default sample/TX cadence - see file header
#define TX_INTERVAL_MIN_MS 15000       // legal-duty-cycle floor at SF7 - CFG,INTERVAL can never go below this
#define TX_INTERVAL_MAX_MS 86400000UL  // 24h sanity ceiling - guards a fat-fingered CFG bricking telemetry

#define LORA_POST_TX_LISTEN_MS 2000  // window after each TX to receive a queued CFG command - see file header

PowerManager power;
SleepManager sleepMgr;

Bme690Sensor bme690(BME690_I2C_ADDR);
Sen0466Sensor sen0466(SEN0466_I2C_ADDR);
Bmv080Sensor bmv080(BMV080_I2C_ADDR);
Cm1106Sensor cm1106(Serial2, PIN_CM1106_RX, PIN_CM1106_TX, PIN_CM1106_EN, CM1106_WARMUP_MS, CM1106_BAUD);
CalypsoSensor calypso(Serial1, PIN_CALYPSO_RX, PIN_CALYPSO_TX, CALYPSO_BAUD);
RYLR998 radio(Serial0, LORA_RX_PIN, LORA_TX_PIN);  // UART0 is free - UART1/UART2 taken above

bool bmeReady = false;
bool sen0466Ready = false;
bool bmvReady = false;
bool radioReady = false;

// Last-known-good readings, cached across cycles - see file header.
float g_temp = 0, g_hum = 0, g_pres = 0, g_gas = 0;
float g_pm1 = 0, g_pm25 = 0, g_pm10 = 0;
float g_co2 = 0;
float g_co = 0, g_coTemp = 0;
float g_windAngle = 0, g_windSpeed = 0;
bool g_windValid = false;

// Runtime-mutable config, pushed via downlink CFG - see file header. Volatile
// only: resets to these compiled-in defaults on every reboot, by design.
uint32_t g_txIntervalMs = TX_INTERVAL_MS;
bool g_bmeEnabled = true;
bool g_sen0466Enabled = true;
bool g_bmvEnabled = true;
bool g_cm1106Enabled = true;
bool g_calypsoEnabled = true;

// Parses/applies a "CFG,<key>=<value>[,<key>=<value>...]" downlink command
// into the g_* runtime globals above. Unrecognized keys are logged and
// skipped, not fatal. Returns true (with ackPayload filled, comma-joined
// "<key>=<value>" tokens of what was actually applied post-clamp) only if at
// least one key was recognized - see file header for why a no-op command
// gets no ACK.
static bool applyConfigCommand(const char *payload, uint8_t len, char *ackPayload, size_t ackCap) {
  static const char kPrefix[] = "CFG,";
  static const size_t kPrefixLen = sizeof(kPrefix) - 1;
  if (len <= kPrefixLen || memcmp(payload, kPrefix, kPrefixLen) != 0) return false;

  char buf[241];
  uint8_t bodyLen = len - (uint8_t)kPrefixLen;
  if (bodyLen >= sizeof(buf)) bodyLen = sizeof(buf) - 1;
  memcpy(buf, payload + kPrefixLen, bodyLen);
  buf[bodyLen] = '\0';

  ackPayload[0] = '\0';
  size_t ackLen = 0;
  bool appliedAny = false;

  char *saveptr = nullptr;
  char *tok = strtok_r(buf, ",", &saveptr);
  while (tok != nullptr) {
    char *eq = strchr(tok, '=');
    if (eq) {
      *eq = '\0';
      const char *key = tok;
      const char *valueStr = eq + 1;
      char appliedKv[24] = {0};
      bool matched = true;

      if (strcmp(key, "INTERVAL") == 0) {
        long seconds = atol(valueStr);
        const long minSeconds = TX_INTERVAL_MIN_MS / 1000;
        const long maxSeconds = TX_INTERVAL_MAX_MS / 1000;
        if (seconds < minSeconds) {
          Serial.printf("[cfg] INTERVAL=%ld below floor, clamped to %ld\n", seconds, minSeconds);
          seconds = minSeconds;
        } else if (seconds > maxSeconds) {
          Serial.printf("[cfg] INTERVAL=%ld above ceiling, clamped to %ld\n", seconds, maxSeconds);
          seconds = maxSeconds;
        }
        g_txIntervalMs = (uint32_t)seconds * 1000UL;
        snprintf(appliedKv, sizeof(appliedKv), "INTERVAL=%ld", seconds);
      } else if (strcmp(key, "BME690") == 0) {
        g_bmeEnabled = atoi(valueStr) != 0;
        snprintf(appliedKv, sizeof(appliedKv), "BME690=%d", g_bmeEnabled ? 1 : 0);
      } else if (strcmp(key, "SEN0466") == 0) {
        g_sen0466Enabled = atoi(valueStr) != 0;
        snprintf(appliedKv, sizeof(appliedKv), "SEN0466=%d", g_sen0466Enabled ? 1 : 0);
      } else if (strcmp(key, "BMV080") == 0) {
        g_bmvEnabled = atoi(valueStr) != 0;
        snprintf(appliedKv, sizeof(appliedKv), "BMV080=%d", g_bmvEnabled ? 1 : 0);
      } else if (strcmp(key, "CM1106") == 0) {
        g_cm1106Enabled = atoi(valueStr) != 0;
        snprintf(appliedKv, sizeof(appliedKv), "CM1106=%d", g_cm1106Enabled ? 1 : 0);
      } else if (strcmp(key, "CALYPSO") == 0) {
        g_calypsoEnabled = atoi(valueStr) != 0;
        snprintf(appliedKv, sizeof(appliedKv), "CALYPSO=%d", g_calypsoEnabled ? 1 : 0);
      } else {
        matched = false;
      }

      if (matched) {
        appliedAny = true;
        size_t kvLen = strlen(appliedKv);
        size_t sep = (ackLen > 0) ? 1 : 0;
        if (ackLen + sep + kvLen < ackCap) {
          if (sep) ackPayload[ackLen++] = ',';
          memcpy(ackPayload + ackLen, appliedKv, kvLen);
          ackLen += kvLen;
          ackPayload[ackLen] = '\0';
        }
      } else {
        Serial.printf("[cfg] unrecognized key \"%s\", ignored\n", key);
      }
    }
    tok = strtok_r(nullptr, ",", &saveptr);
  }

  return appliedAny;
}

// Handles one inbound LoRa message drained during the post-TX listen window -
// either a recognized CFG downlink (apply + ACK) or anything else (logged
// only; no command protocol beyond CFG exists yet).
static void handleInboundMessage(const LoRaMessage &msg) {
  Serial.printf("[downlink] from addr %u (%d bytes, rssi %d, snr %d): %.*s\n", msg.senderAddr, msg.length,
                msg.rssi, msg.snr, msg.length, msg.payload);

  if (msg.senderAddr != LORA_RX_ADDR) {
    Serial.println("[downlink] ignored - not from Module B (LORA_RX_ADDR)");
    return;
  }

  char kv[200];
  if (!applyConfigCommand(msg.payload, msg.length, kv, sizeof(kv))) {
    return;  // not a recognized CFG command, or nothing in it applied - no ACK
  }

  if (!radioReady) {
    Serial.println("[cfg] applied, but ACK skipped - radio not initialized");
    return;
  }

  char ack[220];
  int ackLen = snprintf(ack, sizeof(ack), "ACK,%s", kv);
  bool sent = radio.send(LORA_RX_ADDR, ack, (uint8_t)ackLen);
  Serial.printf("[cfg] applied, ACK sent: %s (%s)\n", ack, sent ? "ok" : "FAILED");
}

void setup() {
  Serial.begin(115200);
  uint32_t waitStart = millis();
  while (!Serial && millis() - waitStart < 3000) delay(10);
  delay(500);

  Serial.println();
  Serial.println("=== CHIP FOREST + LoRa TX: sensors -> RYLR998 -> ground station ===");

  power.enable3V3Sensors();
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  Serial.printf("Waiting %dms for BMV080 startup...\n", BMV080_STARTUP_DELAY_MS);
  delay(BMV080_STARTUP_DELAY_MS);

  bmeReady = bme690.begin();
  Serial.println(bmeReady ? "BME690: OK" : "BME690: NOT FOUND");

  sen0466Ready = sen0466.begin();
  Serial.println(sen0466Ready ? "SEN0466: OK" : "SEN0466: NOT FOUND");

  bmvReady = bmv080.begin();
  Serial.println(bmvReady ? "BMV080: OK" : "BMV080: NOT FOUND");

  cm1106.begin();
  calypso.begin();

  Serial.println("Configuring RYLR998 (link parameters must match pp1-lora-receiver):");
  radioReady = radio.begin(LORA_MY_ADDR, LORA_NETWORK_ID, LORA_BAND_HZ,
                            {LORA_PARAM_SF, LORA_PARAM_BW, LORA_PARAM_CR, LORA_PARAM_PREAMBLE}, &Serial);
  Serial.println(radioReady ? "Radio init OK"
                             : "Radio init FAILED - not transmitting until this is fixed "
                               "(see the AT exchange above for which step was rejected)");

  Serial.println("Setup complete.\n");
}

void loop() {
  sleepMgr.sleep((uint64_t)g_txIntervalMs * 1000ULL);

  // Wind sensor streams continuously - one snapshot per wake.
  if (g_calypsoEnabled) {
    Reading wind = calypso.read();
    if (wind.ok()) {
      g_windAngle = wind.values[0];
      g_windSpeed = wind.values[1];
      g_windValid = wind.values[2] > 0.5f;
    }
  }

  if (bmvReady && g_bmvEnabled) {
    Reading pm = bmv080.read();
    if (pm.ok()) {
      g_pm1 = pm.values[0];
      g_pm25 = pm.values[1];
      g_pm10 = pm.values[2];
    }
  }

  if (bmeReady && g_bmeEnabled) {
    Reading env = bme690.read();
    if (env.ok()) {
      g_temp = env.values[0];
      g_hum = env.values[1];
      g_pres = env.values[2];
      g_gas = env.values[3];
    }
  }

  if (sen0466Ready && g_sen0466Enabled) {
    Reading co = sen0466.read();
    if (co.ok()) {
      g_co = co.values[0];
      g_coTemp = co.values[1];
    }
  }

  if (g_cm1106Enabled) {
    Reading co2 = cm1106.read();
    if (co2.ok()) g_co2 = co2.values[0];
  }

  char payload[96];
  int len = snprintf(payload, sizeof(payload), "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d",
                      g_temp, g_hum, g_pres, g_gas, g_pm1, g_pm25, g_pm10, g_co2, g_co, g_coTemp, g_windAngle,
                      g_windSpeed, g_windValid ? 1 : 0);

  if (!radioReady) {
    // Never transmit under parameters the module didn't actually accept -
    // a "sent" here would be a false signal that the link is configured
    // correctly when it isn't.
    Serial.printf("TX skipped (radio not initialized): %s\n", payload);
    return;  // no radio - nothing to listen on either
  }

  bool sent = radio.send(LORA_RX_ADDR, payload, (uint8_t)len);
  Serial.printf("TX: %s  (%s)\n", payload, sent ? "sent" : "send FAILED");

  // Post-TX listen window: the only moment Module B can reach this node -
  // see file header for the sender-side contract this implies.
  LoRaMessage msg;
  uint32_t listenStart = millis();
  while (millis() - listenStart < LORA_POST_TX_LISTEN_MS) {
    if (radio.poll(msg)) {
      handleInboundMessage(msg);
      break;  // one command per cycle - see file header
    }
  }
}
