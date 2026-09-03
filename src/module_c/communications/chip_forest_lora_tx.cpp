/*
 * CHIP FOREST bring-up + LoRa TX: all five sensors sampled continuously into a
 * last-known-good cache, and that cache transmitted over the RYLR998 to the
 * ground-station receiver (Module B) once every LORA_TX_PERIOD_MS.
 *
 * Payload schema (must match Module B's TelemetryParser exactly):
 *   temp,hum,pres,gas,pm1,pm25,pm10,co2,co,coTemp,windAngle,windSpeed,windValid
 * See src/module_b/communications/telemetry/TelemetryParser.h for the
 * authoritative field list/order - provisional on both ends; if either side
 * changes this encoding, update the other to match.
 *
 * ---- Cadence (2026-09-01) --------------------------------------------------
 * Sample-then-send is split into two independent rates:
 *   - loop() re-reads EVERY sensor every SAMPLE_GAP_MS (~2s) and updates its
 *     cached value whenever the read succeeds. A sensor that hasn't produced a
 *     valid reading yet just keeps being retried; once it has, the cache holds
 *     the freshest good value.
 *   - The radio transmits that whole cached snapshot to Module B once every
 *     LORA_TX_PERIOD_MS (default 5 min; first packet 30s after boot). Module B
 *     in turn batches to Module A / ThingsBoard on its own ~10-min cadence
 *     (MQTT_BATCH_SECONDS in module_b/communications/Config.h).
 * So by the time a packet goes out, each sensor has had ~150 read attempts -
 * even a flaky sensor almost always has a fresh value in it.
 *
 * ---- LoRa parameters (SF7/BW7/CR1/preamble12) ----------------------------
 * SF7 (not the generic "7,7,1,4" default, which returned +ERR=18 from this
 * module - preamble had to be 12). At SF7/BW125/CR4-5/preamble12 with an
 * ~75-byte payload one packet is ~137ms on air. The EU868 1% duty cycle
 * allows ~36s airtime/hour; at a 5-min send period that's ~12 packets/hour
 * (~1.6s/hour) - three orders of magnitude under the cap, so the duty cycle
 * is never the binding constraint at this cadence. LORA_TX_PERIOD_MIN_MS is
 * kept at 15s purely as a sanity floor. SF7 costs ~5-6dB link budget vs SF9
 * (~half the range, worse through foliage) - no field range test has
 * confirmed the deployment closes at SF7 yet. MUST match Module B's
 * Config.h LORA_PARAM_SF.
 *
 * ---- No sleep yet -------------------------------------------------------
 * The node runs a plain delay(SAMPLE_GAP_MS) between sample passes - NOT
 * light or deep sleep. Light sleep was removed 2026-08-28: on the ESP32-S3 it
 * kills USB-Serial/JTAG (port re-enumerates every cycle, wedges the host) and
 * halts the Calypso UART ISR. delay() keeps USB + all UART ISRs serviced and
 * preserves driver state, so the BMV080 5s / CM1106 3s warmups are paid once
 * at boot. The cost is power - this firmware does NOT meet the supercap
 * budget in notes/power_budget.md. The deployment version (deep sleep,
 * switched rails, per-sensor sample periods, Normal/Prealarma/Alarma states)
 * is a separate build on top of this - see that doc.
 *
 * The post-TX LORA_POST_TX_LISTEN_MS window is still the only moment Module B
 * can reach this node with a CFG downlink - whatever relays a command down
 * must hold it and send it the instant it sees this node's uplink.
 *
 * Downlink command grammar (Module B -> this node), styled to match
 * pp1-lora-receiver's own Module A->B grammar (NbiotProtocol.h) so a future
 * relay is a thin translation rather than two incompatible formats:
 *   CFG,<key>=<value>[,<key>=<value>...]
 * Recognized keys: INTERVAL (the LoRa send period, seconds, clamped to
 * [LORA_TX_PERIOD_MIN_MS/1000, LORA_TX_PERIOD_MAX_MS/1000]), and one per sensor -
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

// LoRa send period - Module C -> Module B. Sampling runs continuously
// (SAMPLE_GAP_MS); this is only how often the cached snapshot is transmitted.
// Runtime-tunable via CFG,INTERVAL=<seconds>. At this cadence the EU868 duty
// cycle is a non-issue (see file header); the MIN floor is just a sanity guard.
#define LORA_TX_PERIOD_MS 300000UL      // 5 min
#define LORA_TX_PERIOD_MIN_MS 15000UL   // sanity floor - CFG,INTERVAL can't go below this
#define LORA_TX_PERIOD_MAX_MS 86400000UL  // 24h ceiling - guards a fat-fingered CFG
#define FIRST_TX_DELAY_MS 30000UL       // first packet this soon after boot, then every LORA_TX_PERIOD_MS

#define SAMPLE_GAP_MS 2000UL   // re-read every sensor into its cache this often
#define READ_LOG_GAP_MS 10000UL  // throttle the [read] diagnostic line to at most this rate

#define LORA_POST_TX_LISTEN_MS 2000  // window after each TX to receive a queued CFG command - see file header

// The Calypso streams NMEA continuously. flushInput() drops whatever's stale in
// the RX buffer, then read() is polled for up to this long for a fresh
// checksum-valid $--MWV. With the UART ISR running (no sleep) this usually
// returns almost immediately.
#define CALYPSO_READ_WINDOW_MS 1500

PowerManager power;

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

// Last-known-good readings, refreshed continuously by sampleSensors() - see
// file header. A field stays at its last good value (0 until the first ever
// success) rather than snapping to 0 on a transient miss.
float g_temp = 0, g_hum = 0, g_pres = 0, g_gas = 0;
float g_pm1 = 0, g_pm25 = 0, g_pm10 = 0;
float g_co2 = 0;
float g_co = 0, g_coTemp = 0;
float g_windAngle = 0, g_windSpeed = 0;
bool g_windValid = false;

// Status of each sensor's most recent read attempt - for the [read] diag line.
ReadingStatus g_bmeSt = ReadingStatus::NotInitialized;
ReadingStatus g_coSt = ReadingStatus::NotInitialized;
ReadingStatus g_bmvSt = ReadingStatus::NotInitialized;
ReadingStatus g_co2St = ReadingStatus::NotInitialized;
ReadingStatus g_calSt = ReadingStatus::NotInitialized;

// Runtime-mutable config, pushed via downlink CFG - see file header. Volatile
// only: resets to these compiled-in defaults on every reboot, by design.
uint32_t g_txPeriodMs = LORA_TX_PERIOD_MS;  // CFG,INTERVAL=<seconds>
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
        const long minSeconds = LORA_TX_PERIOD_MIN_MS / 1000;
        const long maxSeconds = LORA_TX_PERIOD_MAX_MS / 1000;
        if (seconds < minSeconds) {
          Serial.printf("[cfg] INTERVAL=%ld below floor, clamped to %ld\n", seconds, minSeconds);
          seconds = minSeconds;
        } else if (seconds > maxSeconds) {
          Serial.printf("[cfg] INTERVAL=%ld above ceiling, clamped to %ld\n", seconds, maxSeconds);
          seconds = maxSeconds;
        }
        g_txPeriodMs = (uint32_t)seconds * 1000UL;
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

static const char *statusName(ReadingStatus s) {
  switch (s) {
    case ReadingStatus::NotInitialized: return "NotInit";
    case ReadingStatus::NotReady:       return "NotReady";
    case ReadingStatus::Timeout:        return "Timeout";
    case ReadingStatus::NoAck:          return "NoAck";
    case ReadingStatus::InvalidFrame:   return "BadFrame";
    case ReadingStatus::Ok:             return "OK";
  }
  return "?";
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

// One pass over every enabled sensor, updating its cached g_* value on a
// successful read and its g_*St status either way. Called every SAMPLE_GAP_MS.
static void sampleSensors() {
  if (g_calypsoEnabled) {
    // Calypso streams NMEA - drop the stale buffer, then poll briefly for a
    // fresh checksum-valid $--MWV.
    calypso.flushInput();
    Reading wind;
    uint32_t t0 = millis();
    do {
      wind = calypso.read();
      if (wind.ok()) break;
      delay(10);
    } while (millis() - t0 < CALYPSO_READ_WINDOW_MS);
    g_calSt = wind.status;
    if (wind.ok()) {
      g_windAngle = wind.values[0];
      g_windSpeed = wind.values[1];
      g_windValid = wind.values[2] > 0.5f;
    }
  }

  if (bmvReady && g_bmvEnabled) {
    Reading pm = bmv080.read();
    g_bmvSt = pm.status;
    if (pm.ok()) {
      g_pm1 = pm.values[0];
      g_pm25 = pm.values[1];
      g_pm10 = pm.values[2];
    }
  }

  if (bmeReady && g_bmeEnabled) {
    Reading env = bme690.read();
    g_bmeSt = env.status;
    if (env.ok()) {
      g_temp = env.values[0];
      g_hum = env.values[1];
      g_pres = env.values[2];
      g_gas = env.values[3];
    }
  }

  if (sen0466Ready && g_sen0466Enabled) {
    Reading co = sen0466.read();
    g_coSt = co.status;
    if (co.ok()) {
      g_co = co.values[0];
      g_coTemp = co.values[1];
    }
  }

  if (g_cm1106Enabled) {
    Reading co2 = cm1106.read();
    g_co2St = co2.status;
    if (co2.ok()) g_co2 = co2.values[0];
  }
}

// Builds the CSV payload from the cache and transmits it to Module B, then
// holds a short window open for a CFG downlink.
static void transmitSnapshot() {
  char payload[96];
  int len = snprintf(payload, sizeof(payload), "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d",
                     g_temp, g_hum, g_pres, g_gas, g_pm1, g_pm25, g_pm10, g_co2, g_co, g_coTemp, g_windAngle,
                     g_windSpeed, g_windValid ? 1 : 0);

  if (!radioReady) {
    // Never claim "sent" under parameters the module didn't accept.
    Serial.printf("TX skipped (radio not initialized): %s\n", payload);
    return;
  }

  bool sent = radio.send(LORA_RX_ADDR, payload, (uint8_t)len);
  Serial.printf("TX: %s  (%s)\n", payload, sent ? "sent" : "send FAILED");

  // Post-TX listen window: the only moment Module B can push a CFG downlink.
  LoRaMessage msg;
  uint32_t t0 = millis();
  while (millis() - t0 < LORA_POST_TX_LISTEN_MS) {
    if (radio.poll(msg)) {
      handleInboundMessage(msg);
      break;  // one command per window
    }
  }
}

void loop() {
  uint32_t now = millis();

  sampleSensors();

  static uint32_t lastReadLogMs = 0;
  if (now - lastReadLogMs >= READ_LOG_GAP_MS) {
    lastReadLogMs = now;
    Serial.printf("[read] co2=%.1f(%s) co=%.2f(%s) | bme=%s bmv=%s calypso=%s(rx=%u) | T%.1f H%.1f gas%.0f pm2.5=%.1f\n",
                  g_co2, statusName(g_co2St), g_co, statusName(g_coSt), statusName(g_bmeSt), statusName(g_bmvSt),
                  g_calypsoEnabled ? (g_windValid ? "OK" : "silent") : "off", calypso.lastReadBytes(),
                  g_temp, g_hum, g_gas, g_pm25);
  }

  // Transmit the cached snapshot every g_txPeriodMs (first one FIRST_TX_DELAY_MS
  // after boot so it doesn't wait a full period for the first packet).
  static uint32_t lastTxMs = 0;
  static bool firstTxDone = false;
  uint32_t txDue = firstTxDone ? g_txPeriodMs : FIRST_TX_DELAY_MS;
  if (now - lastTxMs >= txDue) {
    lastTxMs = now;
    firstTxDone = true;
    transmitSnapshot();
  }

  // NOT sleep - see file header. Just paces the sample loop.
  delay(SAMPLE_GAP_MS);
}
