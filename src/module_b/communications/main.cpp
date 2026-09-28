/*
 * CHIP FOREST LoRa ground-station receiver.
 *
 * Standalone firmware, no sensors attached to this board - it only receives
 * telemetry over a RYLR998 LoRa module and serves it as a small live dashboard
 * over a self-hosted WiFi AP. No sensor data is transmitted from this end.
 *
 * Wiring (ESP32-S3-Zero, see include/Config.h):
 *   Module VDD  -> 3V3
 *   Module GND  -> GND
 *   Module TXD  -> GPIO4 (ESP RX)
 *   Module RXD  -> GPIO5 (ESP TX)
 *   Module NRST -> floating
 *
 * Architecture: radio/RYLR998 (LoRa transport) and telemetry/TelemetryParser
 * (payload -> SensorSnapshot) know nothing about the web server; this file is
 * the only place all three meet. loop() interleaves web-client servicing and
 * radio polling - neither blocks the other, so serving a page can never cause
 * a missed packet, and waiting on the radio (which poll() never does) can
 * never stall the server.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Config.h"
#include "modem_nbiot_mqtt.h"
#include "radio/RYLR998.h"
#include "telemetry/SensorSnapshot.h"
#include "telemetry/TelemetryParser.h"
#include "web/DashboardPage.h"

// -1 = no separate reset pin - this module's RESET ties to the ESP32's own
// reset, same convention as every other SSD1306 breakout on this project.
Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

RYLR998 radio(Serial1, LORA_RX_PIN, LORA_TX_PIN, LORA_BAUD);
// UART2 - deliberately not Serial1, which the LoRa radio above already owns.
// See the NB-IoT wiring comment in Config.h. Pins here are the
// hardware-confirmed GP44/GP43 (native UART0), not the stale GP8/GP7 an
// earlier assumption used - see Config.h's NB-IoT section.
ModemNBIoTMqtt modem(Serial2, NBIOT_RX_PIN, NBIOT_TX_PIN, NBIOT_EN_PIN, NBIOT_CHANNEL_PIN, NBIOT_PWRKEY_PIN);
WebServer server(80);

SensorSnapshot g_latest;

// Ring buffer of every successfully-parsed reading received over LoRa, in
// RAM only (lost on reboot/power loss) - see Config.h's LORA_HISTORY_CAPACITY.
// Independent of modem's own uplink ring (that one exists to batch for
// NB-IoT and gets drained on publish; this one is purely a local record for
// /history and is never drained, just wraps oldest-first once full).
SensorSnapshot g_history[LORA_HISTORY_CAPACITY];
uint16_t g_historyHead = 0;
uint16_t g_historyCount = 0;

static void handleRoot() {
  server.send_P(200, "text/html", DASHBOARD_HTML);
}

static void handleData() {
  uint32_t now = millis();
  bool stale = g_latest.hasData && (now - g_latest.lastHeardMs > DATA_STALE_MS);
  const char *status = !g_latest.hasData ? "no_data" : (stale ? "stale" : "ok");
  uint32_t secondsAgo = g_latest.hasData ? (now - g_latest.lastHeardMs) / 1000 : 0;

  char json[800];
  snprintf(json, sizeof(json),
           "{\"status\":\"%s\",\"secondsAgo\":%lu,\"rssi\":%d,\"snr\":%d,"
           "\"temp\":%.2f,\"hum\":%.2f,\"pres\":%.2f,\"gas\":%.2f,"
           "\"pm1\":%.2f,\"pm25\":%.2f,\"pm10\":%.2f,"
           "\"co2\":%.2f,\"co\":%.2f,\"coTemp\":%.2f,"
           "\"windAngle\":%.2f,\"windSpeed\":%.2f,\"windValid\":%s,"
           "\"nbiot\":{\"state\":\"%s\",\"lastError\":\"%s\",\"rssiDbm\":%d,"
           "\"attached\":%s,\"attachedSec\":%lu,\"mqttConnected\":%s,\"mqttReconnects\":%lu,"
           "\"sent\":%lu,\"failed\":%lu,\"dropped\":%lu,"
           "\"batchReadings\":%u,\"batchSeconds\":%lu,\"ringCount\":%u}}",
           status, (unsigned long)secondsAgo, g_latest.rssi, g_latest.snr, g_latest.temp, g_latest.hum,
           g_latest.pres, g_latest.gas, g_latest.pm1, g_latest.pm25, g_latest.pm10, g_latest.co2,
           g_latest.co, g_latest.coTemp, g_latest.windAngle, g_latest.windSpeed,
           g_latest.windValid ? "true" : "false",
           modem.stateName(), modem.lastError(), modem.rssiDbm(), modem.attached() ? "true" : "false",
           (unsigned long)(modem.attachedUptimeMs() / 1000), modem.mqttConnected() ? "true" : "false",
           (unsigned long)modem.mqttReconnects(), (unsigned long)modem.packetsSent(),
           (unsigned long)modem.packetsFailed(), (unsigned long)modem.packetsDropped(),
           modem.batchReadingsTarget(), (unsigned long)modem.batchSecondsTarget(), modem.ringCount());

  server.send(200, "application/json", json);
}

// Serves every buffered reading as a downloadable CSV - millisSinceBoot lets
// you compute wall-clock time if you know when the board booted, and stays
// monotonic across the whole buffer even though there's no RTC on this
// board. Oldest reading first. Built into a static buffer (not the Arduino
// String heap) sized for LORA_HISTORY_CAPACITY rows at this row width, with
// room to spare - see the size check below if either grows.
static char g_historyCsvBuf[LORA_HISTORY_CAPACITY * 110 + 256];

static void handleHistory() {
  size_t pos = 0;
  auto append = [&](const char *s) {
    size_t n = strlen(s);
    if (pos + n < sizeof(g_historyCsvBuf)) {
      memcpy(g_historyCsvBuf + pos, s, n);
      pos += n;
    }
  };

  append(
      "millisSinceBoot,senderAddr,rssi,snr,temp,hum,pres,gas,pm1,pm25,pm10,co2,co,coTemp,windAngle,"
      "windSpeed,windValid\n");

  for (uint16_t i = 0; i < g_historyCount; i++) {
    const SensorSnapshot &s = g_history[(g_historyHead + i) % LORA_HISTORY_CAPACITY];
    char row[110];
    snprintf(row, sizeof(row), "%lu,%u,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d\n",
             (unsigned long)s.lastHeardMs, s.senderAddr, s.rssi, s.snr, s.temp, s.hum, s.pres, s.gas, s.pm1,
             s.pm25, s.pm10, s.co2, s.co, s.coTemp, s.windAngle, s.windSpeed, s.windValid ? 1 : 0);
    append(row);
  }

  server.sendHeader("Content-Disposition", "attachment; filename=\"module_b_history.csv\"");
  server.send(200, "text/csv", g_historyCsvBuf);
}

// Draws a 4-bar phone-style signal indicator (bars grow left->right, filled
// up to the current level) at top-left corner (x, y), bottom-aligned within
// an 8px-tall row. Thresholds are the common cellular-bar convention, mapped
// onto parseCsq()'s -113..-51 dBm range (see NbiotProtocol.cpp); rssiDbm==0
// means "no AT+CSQ reading yet" (a real reading is never 0), drawn as all
// bars empty rather than misleadingly showing signal that hasn't been
// measured.
static void drawSignalBars(int x, int y, int rssiDbm) {
  int level;
  if (rssiDbm == 0) {
    level = 0;
  } else if (rssiDbm >= -70) {
    level = 4;
  } else if (rssiDbm >= -85) {
    level = 3;
  } else if (rssiDbm >= -100) {
    level = 2;
  } else {
    level = 1;
  }

  const int barW = 3, gap = 1, maxH = 8;
  for (int i = 0; i < 4; i++) {
    int h = (i + 1) * 2;  // 2, 4, 6, 8 px tall
    int bx = x + i * (barW + gap);
    int by = y + (maxH - h);
    if (i < level) {
      oled.fillRect(bx, by, barW, h, SSD1306_WHITE);
    } else {
      oled.drawRect(bx, by, barW, h, SSD1306_WHITE);
    }
  }
}

// Refreshes the OLED with a compact live-activity view - answers "what is
// this board doing right now" at a glance without needing the web dashboard:
// LoRa RX recency/link quality, and the NB-IoT/MQTT state machine's current
// state (stateName() literally reads "PUBLISHING" while a batch is actively
// being sent over NB-IoT, "ATTACHING"/"MQTT_CONNECT" while connecting, "IDLE"
// while just waiting on the next batch - see modem_nbiot_mqtt.h's State enum).
static void updateOledStatus() {
  uint32_t now = millis();

  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);

  oled.println("Modulo B");

  if (g_latest.hasData) {
    uint32_t ageS = (now - g_latest.lastHeardMs) / 1000;
    // "RX!" window is short (a fresh packet is momentary, not a stream) -
    // just long enough that a 500ms-refreshed screen visibly catches it.
    oled.printf("LoRa: %s hace %lus\n", ageS < 3 ? "RX!" : "inactivo", (unsigned long)ageS);
    oled.printf(" rssi %d snr %d\n", g_latest.rssi, g_latest.snr);
  } else {
    oled.println("LoRa: sin datos aun");
    oled.println("");
  }

  oled.printf("NBIoT: %s\n", modem.stateName());
  oled.printf("MQTT: %s\n", modem.mqttConnected() ? "activo" : "caido");
  oled.printf("Env %lu Err %lu Prd %lu\n", (unsigned long)modem.packetsSent(),
              (unsigned long)modem.packetsFailed(), (unsigned long)modem.packetsDropped());

  drawSignalBars(112, 0, modem.rssiDbm());  // top-right corner, same row as "Modulo B"

  oled.display();
}

void setup() {
  Serial.begin(115200);
  uint32_t waitStart = millis();
  while (!Serial && millis() - waitStart < 3000) delay(10);
  delay(500);

  Serial.println();
  Serial.println("=== CHIP FOREST LoRa ground-station receiver ===");

  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD);
  IPAddress apIP = WiFi.softAPIP();
  Serial.printf("WiFi AP: SSID=\"%s\"  password=\"%s\"\n", WIFI_AP_SSID, WIFI_AP_PASSWORD);
  Serial.printf("Dashboard: http://%s/\n", apIP.toString().c_str());

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/history", handleHistory);
  server.begin();
  Serial.printf("Web server started - reading history CSV at http://%s/history\n", apIP.toString().c_str());

  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  if (oled.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR)) {
    oled.clearDisplay();
    oled.display();
    Serial.println("OLED: init OK");
  } else {
    Serial.println("OLED: init FAILED (check wiring - SDA=GPIO1, SCL=GPIO2)");
  }

  pinMode(LORA_EN_PIN, OUTPUT);
  digitalWrite(LORA_EN_PIN, LOW);  // power on LoRa rail (Q4 gate) - confirmed LOW=on by pin scan
  delay(300);

  Serial.println("Configuring RYLR998 (link parameters must match the sensor node):");
  bool ok = radio.begin(LORA_MY_ADDR, LORA_NETWORK_ID, LORA_BAND_HZ,
                         {LORA_PARAM_SF, LORA_PARAM_BW, LORA_PARAM_CR, LORA_PARAM_PREAMBLE}, &Serial);
  Serial.println(ok ? "Radio init OK - listening for packets."
                     : "Radio init FAILED - check wiring/baud before assuming a parameter is wrong.");
  Serial.println();

  Serial.println("Starting NB-IoT/MQTT uplink to ThingsBoard - non-blocking, see modem_nbiot_mqtt.h:");
  modem.begin();
  Serial.println();
}

// Last charge state heard from each node, by LoRa address.
//
// Kept separate from the telemetry ring because STAT and telemetry arrive as
// different packets at different times - a node transmits its readings and its
// health independently, and either may be the one that gets through.
struct NodeCharge {
  uint16_t addr;
  int16_t pct;
  int16_t mv;
};
static NodeCharge g_nodeCharge[8];
static uint8_t g_nodeChargeCount = 0;

static void nodeChargeRemember(uint16_t addr, int pct, int mv) {
  for (uint8_t i = 0; i < g_nodeChargeCount; i++) {
    if (g_nodeCharge[i].addr == addr) {
      g_nodeCharge[i].pct = (int16_t)pct;
      g_nodeCharge[i].mv = (int16_t)mv;
      return;
    }
  }
  if (g_nodeChargeCount < (uint8_t)(sizeof(g_nodeCharge) / sizeof(g_nodeCharge[0]))) {
    g_nodeCharge[g_nodeChargeCount].addr = addr;
    g_nodeCharge[g_nodeChargeCount].pct = (int16_t)pct;
    g_nodeCharge[g_nodeChargeCount].mv = (int16_t)mv;
    g_nodeChargeCount++;
  }
}

// Last fire state heard from each node, by LoRa address. Same shape as the charge
// store and for the same reason: STAT and telemetry are separate packets that
// arrive at different times, and either may be the one that gets through.
struct NodeAlarm {
  uint16_t addr;
  int8_t state;
};
static NodeAlarm g_nodeAlarm[8];
static uint8_t g_nodeAlarmCount = 0;

static void nodeAlarmRemember(uint16_t addr, int state) {
  for (uint8_t i = 0; i < g_nodeAlarmCount; i++) {
    if (g_nodeAlarm[i].addr == addr) {
      g_nodeAlarm[i].state = (int8_t)state;
      return;
    }
  }
  if (g_nodeAlarmCount < (uint8_t)(sizeof(g_nodeAlarm) / sizeof(g_nodeAlarm[0]))) {
    g_nodeAlarm[g_nodeAlarmCount].addr = addr;
    g_nodeAlarm[g_nodeAlarmCount].state = (int8_t)state;
    g_nodeAlarmCount++;
  }
}

static void nodeAlarmAttach(SensorSnapshot &snap) {
  for (uint8_t i = 0; i < g_nodeAlarmCount; i++) {
    if (g_nodeAlarm[i].addr == snap.senderAddr) {
      snap.alarmState = g_nodeAlarm[i].state;
      return;
    }
  }
}

static void nodeChargeAttach(SensorSnapshot &snap) {
  for (uint8_t i = 0; i < g_nodeChargeCount; i++) {
    if (g_nodeCharge[i].addr == snap.senderAddr) {
      snap.chargePct = g_nodeCharge[i].pct;
      snap.capMv = g_nodeCharge[i].mv;
      return;
    }
  }
}

void loop() {
  server.handleClient();

  LoRaMessage msg;
  while (radio.poll(msg)) {
    Serial.printf("+RCV addr=%u len=%u rssi=%d snr=%d data=\"%.*s\"\n", msg.senderAddr, msg.length,
                  msg.rssi, msg.snr, msg.length, msg.payload);

    // FIRST, before any parsing: if Module A queued a command for this node,
    // fire it now. This uplink is the node's only listening moment - it opens
    // a ~2s window right after transmitting and then goes deaf until its next
    // send period. Anything we do before this send eats into that window.
    if (modem.nodeCommandPending()) {
      char name[24];
      nbiotResolveNodeName(msg.senderAddr, name, sizeof(name));
      if (strcmp(name, modem.nodeCommandDevice()) == 0) {
        const char *cfg = modem.nodeCommandCfg();
        bool sent = radio.send(msg.senderAddr, cfg, (uint8_t)strlen(cfg));
        Serial.printf("  -> downlink to %s: %s (%s)\n", name, cfg, sent ? "sent" : "SEND FAILED");
        if (sent) modem.onNodeCommandDelivered();
      }
    }

    // A node's reply to a config push, not telemetry - route it to the modem
    // so it can answer the original RPC, and don't try to parse it as a
    // reading (it would just be dropped as "wrong field count").
    if (msg.length >= 4 && strncmp(msg.payload, "ACK,", 4) == 0) {
      modem.onNodeCommandAck(msg.payload);
      continue;
    }

    // STAT - the node's own health packet, not telemetry.
    //
    //   STAT,<wake>,<resetReason>,<boots>,<bme><bmv><co2><co><wind>,<ageTicks>,<mV>,<pct>
    //
    // It is a separate packet precisely so it cannot break the 13-field telemetry
    // parser, but the charge state in it is worth having in ThingsBoard - a node
    // that is about to run out of energy should say so where someone will see it.
    // So the values are remembered per node here and attached to that node's next
    // telemetry publish.
    if (msg.length >= 5 && strncmp(msg.payload, "STAT,", 5) == 0) {
      char buf[96];
      size_t n = msg.length < sizeof(buf) - 1 ? msg.length : sizeof(buf) - 1;
      memcpy(buf, msg.payload, n);
      buf[n] = 0;

      // Fields 7 and 8 (0-based 6 and 7) are the sense reading and the percentage.
      int field = 0;
      int mv = -1, pct = -1, alarm = -1;
      char *tok = strtok(buf, ",");
      while (tok) {
        if (field == 6) mv = atoi(tok);
        if (field == 7) pct = atoi(tok);
        if (field == 8) alarm = atoi(tok);  // 0 normal, 1 pre-alarm, 2 alarm
        field++;
        tok = strtok(nullptr, ",");
      }

      // FIRE STATE - the reason this whole path exists.
      //
      // Remembered per node and attached to that node's next telemetry publish,
      // and when raised it also short-circuits the batch window so it goes out now
      // rather than up to a minute later. Batching is right for telemetry and
      // wrong for a fire.
      //
      // Field 8 is absent on older node firmware, which leaves alarm at -1 and is
      // reported as unknown rather than as NORMAL. A dashboard confidently showing
      // NORMAL for a node that never said is worse than one showing nothing.
      if (alarm >= 0) {
        nodeAlarmRemember(msg.senderAddr, alarm);
        // Readings for this node are already queued carrying the previous state -
        // the node sends telemetry before its STAT. Restamp them, or the flush
        // below publishes the very value it exists to correct.
        modem.applyAlarmToQueued(msg.senderAddr, (int8_t)alarm);
        if (alarm > 0) {
          char who[32];
          nbiotResolveNodeName(msg.senderAddr, who, sizeof(who));
          Serial.printf("  *** %s EN %s - publicando YA ***\n", who,
                        alarm >= 2 ? "ALARMA" : "PRE-ALARMA");
          Serial.flush();
          modem.requestFlush();
        }
      }

      // THREE outcomes here, not two. A node that sends no charge fields at all
      // and a node that sends them as "unknown" are different problems, and
      // collapsing them sends someone hunting a firmware version that is fine.
      //
      // pct == -1 is the NORMAL state today: it means SUPERCAP_CALIBRATED is 0 on
      // the node because the GPIO14 divider ratio has never been measured. Every
      // node reports that. It is not old firmware.
      if (field <= 7) {
        // Genuinely short packet - the mV/percentage fields do not exist.
        Serial.printf("  STAT has only %d fields - node predates the charge report\n", field);
      } else {
        // The fields are present. Remember them EVEN WHEN pct is -1: the raw
        // millivolts are what the calibration will be derived from, so throwing
        // them away because the percentage is unknown discards the one number
        // that would let us work the percentage out.
        nodeChargeRemember(msg.senderAddr, pct, mv);
        if (pct >= 0) {
          Serial.printf("  node health: charge %d%% (%d mV at the sense pin)\n", pct, mv);
        } else {
          Serial.printf("  node health: %d mV at the sense pin, percentage not yet "
                        "calibrated (SUPERCAP_CALIBRATED=0 on the node)\n", mv);
        }
      }
      continue;
    }

    // A node sends either a single reading or a BATCH of them. Both shapes end up
    // here; keep them apart, because squeezing a batch through the single-reading
    // parser is what corrupted the data reaching ThingsBoard - the "B" and the
    // count were read as temperature and humidity, every field shifted by two, and
    // the batch's second reading was discarded outright.
    SensorSnapshot batch[BATCH_RX_MAX];
    uint16_t ages[BATCH_RX_MAX] = {};
    uint8_t count = 0;
    bool isBatch = (msg.length >= 2 && msg.payload[0] == 'B' && msg.payload[1] == ',');

    if (isBatch) {
      count = TelemetryParser::parseBatch(msg.payload, msg.length, batch, BATCH_RX_MAX, ages);
      if (count == 0) {
        Serial.println("  (batch payload didn't parse - dropped)");
      } else {
        Serial.printf("  batch of %u reading(s)\n", (unsigned)count);
      }
    } else if (TelemetryParser::parse(msg.payload, msg.length, batch[0])) {
      ages[0] = 0;  // a single reading is current by definition
      count = 1;
    }

    if (count == 0 && !isBatch) {
      Serial.println("  (payload didn't match the expected schema - dropped, see "
                      "telemetry/TelemetryParser.h)");
    }

    for (uint8_t b = 0; b < count; b++) {
      SensorSnapshot &snap = batch[b];
      snap.hasData = true;
      // Backdate by the record's age so a batched reading is timestamped when it
      // was TAKEN, not when the packet happened to arrive. Single readings have
      // age 0 and are unaffected.
      uint32_t ageMs = (uint32_t)ages[b] * (uint32_t)NODE_TICK_SECONDS * 1000UL;
      snap.lastHeardMs = millis() - ageMs;
      snap.senderAddr = msg.senderAddr;
      snap.rssi = msg.rssi;
      snap.snr = msg.snr;
      nodeChargeAttach(snap);  // most recent STAT from this node, if any
      nodeAlarmAttach(snap);
      g_latest = snap;
      modem.enqueue(snap);

      uint16_t idx = (uint16_t)((g_historyHead + g_historyCount) % LORA_HISTORY_CAPACITY);
      g_history[idx] = snap;
      if (g_historyCount < LORA_HISTORY_CAPACITY) {
        g_historyCount++;
      } else {
        g_historyHead = (uint16_t)((g_historyHead + 1) % LORA_HISTORY_CAPACITY);
      }
    }
  }

  modem.tick();

  static uint32_t lastOledMs = 0;
  if (millis() - lastOledMs >= 500) {
    lastOledMs = millis();
    updateOledStatus();
  }
}
