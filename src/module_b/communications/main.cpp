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
#include "Config.h"
#include "modem_nbiot.h"
#include "radio/RYLR998.h"
#include "telemetry/SensorSnapshot.h"
#include "telemetry/TelemetryParser.h"
#include "web/DashboardPage.h"

RYLR998 radio(Serial1, LORA_RX_PIN, LORA_TX_PIN, LORA_BAUD);
// UART2 - deliberately not Serial1, which the LoRa radio above already owns.
// See the NB-IoT wiring comment in Config.h.
ModemNBIoT modem(Serial2, NBIOT_RX_PIN, NBIOT_TX_PIN, NBIOT_EN_PIN, NBIOT_RST_PIN);
WebServer server(80);

SensorSnapshot g_latest;

static void onNbiotConfigApplied(uint8_t version, uint16_t batchReadings, uint32_t batchSeconds) {
  Serial.printf("[nbiot] applied downlink CFG v%u: batch=%u readings / %lus\n", version, batchReadings,
                (unsigned long)batchSeconds);
}

static void handleRoot() {
  server.send_P(200, "text/html", DASHBOARD_HTML);
}

static void handleData() {
  uint32_t now = millis();
  bool stale = g_latest.hasData && (now - g_latest.lastHeardMs > DATA_STALE_MS);
  const char *status = !g_latest.hasData ? "no_data" : (stale ? "stale" : "ok");
  uint32_t secondsAgo = g_latest.hasData ? (now - g_latest.lastHeardMs) / 1000 : 0;

  char json[768];
  snprintf(json, sizeof(json),
           "{\"status\":\"%s\",\"secondsAgo\":%lu,\"rssi\":%d,\"snr\":%d,"
           "\"temp\":%.2f,\"hum\":%.2f,\"pres\":%.2f,\"gas\":%.2f,"
           "\"pm1\":%.2f,\"pm25\":%.2f,\"pm10\":%.2f,"
           "\"co2\":%.2f,\"co\":%.2f,\"coTemp\":%.2f,"
           "\"windAngle\":%.2f,\"windSpeed\":%.2f,\"windValid\":%s,"
           "\"nbiot\":{\"state\":\"%s\",\"lastError\":\"%s\",\"rssiDbm\":%d,"
           "\"attached\":%s,\"attachedSec\":%lu,\"sent\":%lu,\"failed\":%lu,\"dropped\":%lu,"
           "\"cfgVersion\":%u,\"batchReadings\":%u,\"batchSeconds\":%lu,\"ringCount\":%u}}",
           status, (unsigned long)secondsAgo, g_latest.rssi, g_latest.snr, g_latest.temp, g_latest.hum,
           g_latest.pres, g_latest.gas, g_latest.pm1, g_latest.pm25, g_latest.pm10, g_latest.co2,
           g_latest.co, g_latest.coTemp, g_latest.windAngle, g_latest.windSpeed,
           g_latest.windValid ? "true" : "false",
           modem.stateName(), modem.lastError(), modem.rssiDbm(), modem.attached() ? "true" : "false",
           (unsigned long)(modem.attachedUptimeMs() / 1000), (unsigned long)modem.packetsSent(),
           (unsigned long)modem.packetsFailed(), (unsigned long)modem.packetsDropped(),
           modem.appliedCfgVersion(), modem.batchReadingsTarget(), (unsigned long)modem.batchSecondsTarget(),
           modem.ringCount());

  server.send(200, "application/json", json);
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
  server.begin();
  Serial.println("Web server started.");

  Serial.println("Configuring RYLR998 (link parameters must match the sensor node):");
  bool ok = radio.begin(LORA_MY_ADDR, LORA_NETWORK_ID, LORA_BAND_HZ,
                         {LORA_PARAM_SF, LORA_PARAM_BW, LORA_PARAM_CR, LORA_PARAM_PREAMBLE}, &Serial);
  Serial.println(ok ? "Radio init OK - listening for packets."
                     : "Radio init FAILED - check wiring/baud before assuming a parameter is wrong.");
  Serial.println();

  Serial.println("Starting NB-IoT uplink (Module B backhaul) - non-blocking, see modem_nbiot.h:");
  modem.setConfigAppliedCallback(onNbiotConfigApplied);
  modem.begin();
  Serial.println();
}

void loop() {
  server.handleClient();

  LoRaMessage msg;
  while (radio.poll(msg)) {
    Serial.printf("+RCV addr=%u len=%u rssi=%d snr=%d data=\"%.*s\"\n", msg.senderAddr, msg.length,
                  msg.rssi, msg.snr, msg.length, msg.payload);

    SensorSnapshot snap;
    if (TelemetryParser::parse(msg.payload, msg.length, snap)) {
      snap.hasData = true;
      snap.lastHeardMs = millis();
      snap.rssi = msg.rssi;
      snap.snr = msg.snr;
      g_latest = snap;
      modem.enqueue(snap);
    } else {
      Serial.println("  (payload didn't match the expected schema - dropped, see "
                      "telemetry/TelemetryParser.h)");
    }
  }

  modem.tick();
}
