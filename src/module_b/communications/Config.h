#pragma once

#include <stdint.h>
#include <stdio.h>
#include <string.h>

// LoRa ground-station receiver configuration. Everything tunable lives here,
// same convention as the sensor-node repo's Config.h.

// ---------- RYLR998 wiring (ESP32-S3-Zero) ----------
// Same crossover convention as the sensor node: module TXD -> ESP RX, module
// RXD <- ESP TX. Not wired to any sensors on this board - LoRa module only.
#define LORA_RX_PIN 4  // module TXD -> ESP RX
#define LORA_TX_PIN 5  // module RXD <- ESP TX
#define LORA_BAUD 115200
#define LORA_EN_PIN 13  // Q4 power-rail gate, LOW = on - 2026-08-19 pin-scan discovery (see project memory)

// ---------- LoRa link parameters - MUST match the sensor node exactly ----------
// SF/BW/CR/preamble are NOT the generic "commonly documented" RYLR998 defaults
// (7,7,1,4) originally assumed here - that combination came back +ERR=18 from
// the actual sensor-node-side module. These values were read directly off that
// module via AT+PARAMETER? (pp1-optimize's src/rylr998_param_probe.cpp)
// instead of guessed a second time. If you ever change these, change them on
// both ends together and re-verify with that same probe.
#define LORA_BAND_HZ 868000000UL  // EU868 - Spain deployment (confirmed, do not change)
#define LORA_NETWORK_ID 5
// SF7 (was SF9) - MUST match the sensor node's chip_forest_lora_tx.cpp
// (LORA_PARAM_SF). Dropped from 9 to 7 (2026-08-28) so the node's 15s TX
// cadence stays under the EU868 1% duty cycle; costs ~5-6dB link budget. See
// that file's header for the airtime math and the range caveat.
#define LORA_PARAM_SF 7
#define LORA_PARAM_BW 7
#define LORA_PARAM_CR 1
#define LORA_PARAM_PREAMBLE 12

#define LORA_NODE_ADDR 1  // sensor node's AT+ADDRESS
#define LORA_MY_ADDR 2    // this receiver's own AT+ADDRESS - must differ from the node's

// ---------- Node name table (LoRa AT+ADDRESS -> ThingsBoard device name) ----------
// Used by the MQTT gateway publish to name each node in the
// v1/gateway/telemetry payload. One entry per node deployed so far - add to
// this as more nodes join, never remove/renumber an existing entry (the
// name is what ThingsBoard uses to identify the device going forward).
struct NodeNameEntry {
  uint16_t addr;
  const char *name;
};

// Address 2 is this receiver (LORA_MY_ADDR) and is deliberately absent - it is
// never a sender. Nodes were all shipped on address 1, which made them
// indistinguishable on the air; they now get one address each, set per build via
// the env:module-c-node* environments.
static const NodeNameEntry NBIOT_NODE_NAMES[] = {
    {LORA_NODE_ADDR, "NodoC-1"},  // 1
    {3, "NodoC-2"},
    {4, "NodoC-3"},
};

// Resolves addr to its configured name, or "NodoDesconocido-<addr>" if addr
// isn't in the table above. Deliberately never drops a reading just because
// its sender isn't recognized - a node with a placeholder name is visible
// and fixable in ThingsBoard; a node whose data is silently discarded isn't.
inline void nbiotResolveNodeName(uint16_t addr, char *outName, size_t outCap) {
  for (size_t i = 0; i < sizeof(NBIOT_NODE_NAMES) / sizeof(NBIOT_NODE_NAMES[0]); i++) {
    if (NBIOT_NODE_NAMES[i].addr == addr) {
      strncpy(outName, NBIOT_NODE_NAMES[i].name, outCap - 1);
      outName[outCap - 1] = '\0';
      return;
    }
  }
  snprintf(outName, outCap, "NodoDesconocido-%u", addr);
}

// ---------- OLED status display (GME12864, SSD1306-compatible 128x64 I2C) ----------
#define OLED_SDA_PIN 1
#define OLED_SCL_PIN 2
#define OLED_I2C_ADDR 0x3C
#define OLED_WIDTH 128
#define OLED_HEIGHT 64

// ---------- WiFi AP ----------
// PLACEHOLDER credentials - change before field deployment. WPA2 requires an
// 8-63 char passphrase.
#define WIFI_AP_SSID "CHIP-FOREST-RX"
#define WIFI_AP_PASSWORD "chipforest1"

// ---------- Local history log ----------
// In-RAM ring buffer of received readings, downloadable as CSV from
// /history - lets you record what Module C sent even when Module A/MQTT is
// unreachable (or you just don't want to wait on ThingsBoard for a quick
// bench test). Not persisted across a reboot/power loss - this is a bench
// convenience, not a replacement for the real ThingsBoard uplink.
#define LORA_HISTORY_CAPACITY 200

// ---------- Dashboard staleness ----------
// If no packet has arrived within this window, the dashboard shows "signal
// lost" instead of the last numbers, so stale data is never mistaken for live.
// Must stay comfortably above the node's LoRa send period (LORA_TX_PERIOD_MS,
// 5 min default in chip_forest_lora_tx.cpp) or the dashboard would flag "lost"
// between every normal transmission. Set to ~2.4x that (12 min) - roughly two
// missed packets before it reads stale. The node's send period is remotely
// tunable via CFG,INTERVAL=..., so a fixed threshold can't track an
// arbitrarily-pushed value; tracking the last-applied interval dynamically is
// future work, contingent on Module B relaying CFG downlink (it doesn't yet).
#define DATA_STALE_MS 720000

// ---------- NB-IoT uplink (Quectel BC660K-GL, "Module B" cellular backhaul) ----------
// UART2 - deliberately NOT Serial1 (UART1), which the RYLR998 above already
// owns on GPIO4/5. ESP32-S3 only has three UART controllers (0/1/2) and
// USB-CDC is separate from all of them; reusing UART1 for the modem would
// collide with the LoRa radio the moment both run in the same firmware.
#define NBIOT_BAUD 115200

// Power/pin sequence hardware-confirmed 2026-08-19 on the real PCB (see
// project memory "project_nbiot_uplink_bringup") - supersedes everything
// this section previously assumed, none of which had been validated against
// real hardware timing before that session:
//   GP9 LOW (VIN) -> GP10 LOW (a transistor-gated channel not in any
//   schematic reviewed - without it, UART carries no signal at all) ->
//   settle -> GP11 pulsed LOW ~1000ms then released HIGH (PWRKEY - contrary
//   to the old assumption below, this module does NOT auto-boot on VIN
//   alone) -> wait for boot -> AT works on RX=GP44/TX=GP43 @ 115200 (the
//   schematic's UARTDB_TX/RX native-UART0 pins were right all along; the
//   module just never booted far enough to use them before).
#define NBIOT_RX_PIN 44  // ESP RX <- module TXD
#define NBIOT_TX_PIN 43  // ESP TX -> module RXD

#define NBIOT_EN_PIN 9
#define NBIOT_EN_ACTIVE LOW
#define NBIOT_DISABLE (!NBIOT_EN_ACTIVE)

#define NBIOT_CHANNEL_PIN 10  // transistor-gated channel, LOW = open - 2026-08-19 discovery
#define NBIOT_CHANNEL_ACTIVE LOW

#define NBIOT_PWRKEY_PIN 11
#define NBIOT_PWRKEY_ACTIVE LOW
#define NBIOT_PWRKEY_PULSE_MS 1000

// RST, P1 pin 5 (the schematic mislabels this net "RTS" - it is not a UART
// flow-control line, it's the module's reset input). Active-low pulse. Not
// exercised by the confirmed bring-up sequence above - kept for a future
// explicit-reset recovery path, not currently wired into POWERING.
#define NBIOT_RST_PIN 6
#define NBIOT_RST_ACTIVE LOW
#define NBIOT_RST_PULSE_MS 200

// ---------- Network (1NCE SIM) ----------
// "iot.1nce.net" is 1NCE Platform 1.0 and gets registration denied on this
// account (Platform 2.0) - confirmed both by 1NCE support directly and by
// hardware bring-up 2026-08-19. Use sensor.net, PAP, empty user/pass.
#define NBIOT_APN "sensor.net"
// BC660K-GL's own PDP context is numbered from 0, NOT 1 - the BC66 docs (and
// most Quectel AT-command examples) use context 1, which is a common trap on
// this specific module. IMPORTANT: context 0 is this module's built-in
// default context, and the standard AT+CGDCONT command does NOT work on it
// (rejected with "+CME ERROR: operation not supported", confirmed
// 2026-08-19) - use AT+QCGDEFCONT="IP",<apn> instead. Also: AT+CFUN=1 must
// be sent explicitly before AT+CPIN? - this module does not auto-enable its
// radio/protocol stack on boot (AT+CPIN? fails "+CME ERROR: ue not power
// on" until CFUN=1 is issued).
#define NBIOT_CONTEXT_ID 0

// ---------- MQTT / ThingsBoard (ModemNBIoTMqtt) ----------
// test-moduloa.home.kg is Module A's self-hosted ThingsBoard instance - not
// ThingsBoard Cloud/demo.thingsboard.io, confirmed by the user 2026-08-19.
//
// Port 1883 is direct-LAN only and NOT reachable from the public internet -
// found 2026-08-19 by comparing against a working ESPHome device's config
// for this same broker: internet-connected clients (this one, over
// cellular) go through an Azure relay + reverse SSH tunnel on port 18831
// instead. Connecting to 1883 over the internet doesn't fail to connect at
// the TCP/MQTT-framing level - it produces a real, well-formed CONNACK,
// just always with reason code 5 ("not authorized"), which looks exactly
// like a credentials problem and cost real debugging time before the port
// mismatch was found. Verified directly: a plain Python paho-mqtt client
// got CONNACK 5 on 1883 and CONNACK 0 (success) on 18831 with the exact
// same token, no other change.
#define MQTT_BROKER_HOST "test-moduloa.home.kg"
#define MQTT_BROKER_PORT 18831
#define MQTT_CLIENT_ID "CON-1"
// ThingsBoard access-token auth: token goes in as the MQTT username, no
// password. This is the gateway device's ("CON-1" in ThingsBoard, created
// 2026-08-28 with "Is gateway" enabled to replace the earlier
// "CON-MODB_TEST") token, not any individual node's - the hub publishes on
// nodes' behalf via the Gateway API (see modem_nbiot_mqtt.h).
#define MQTT_ACCESS_TOKEN "QRPJgyk5COJPCavycmpp"
#define MQTT_CLIENT_IDX 0  // AT+QMTOPEN/QMTCONN/QMTPUB client index - only one MQTT client is ever open, so a fixed 0 is fine

// ---------- MQTT batching / publish cadence (ModemNBIoTMqtt) ----------
// Decoupled from the legacy NBIOT_BATCH_* binary-frame constants below (those
// still feed the retired raw-UDP path and its host test). The MQTT path
// buffers LoRa readings and uplinks them in time-driven batches:
//
//   MQTT_BATCH_SECONDS       - publish this often. Module C TXes every ~5 min,
//                              so a 10-min batch carries ~2 readings.
//   MQTT_BATCH_MAX_READINGS  - count trigger: publish early if the ring hits
//                              this many before the timer (guards against a
//                              slowed-down uplink letting the ring overflow).
//   MQTT_PUB_CHUNK_READINGS  - a single AT+QMTPUB payload must stay under the
//                              BC660K-GL's ~1400-byte message limit; one
//                              batch is sent as ceil(N/chunk) back-to-back
//                              QMTPUBs over the already-open session. A
//                              timestamped reading is ~230 bytes typical,
//                              ~300 worst case, so 4 (~1200B worst case)
//                              keeps margin under 1400.
#define MQTT_BATCH_SECONDS 600UL
#define MQTT_BATCH_MAX_READINGS 6
#define MQTT_PUB_CHUNK_READINGS 4

// Both batch targets are runtime-mutable from Module A - see the downlink
// section below. Clamped so a fat-fingered command can't stop telemetry dead
// or hammer the SIM's data allowance:
//   SECONDS floor 60  - below this the uplink costs more in cellular overhead
//                       than the readings are worth (Module C only produces
//                       one reading per LORA_TX_PERIOD_MS anyway).
//   READINGS ceiling  - never above NBIOT_RING_CAPACITY, or the count trigger
//                       could never fire before the ring wraps and drops.
#define MQTT_BATCH_SECONDS_MIN 60UL
#define MQTT_BATCH_SECONDS_MAX 86400UL
#define MQTT_BATCH_READINGS_MIN 1
#define MQTT_BATCH_READINGS_MAX NBIOT_RING_CAPACITY

// ---------- Downlink: Module A -> Module B (-> Module C) ----------
// Two subscriptions, because ThingsBoard addresses the gateway itself and its
// child devices on different topics:
//   MQTT_TOPIC_DEVICE_RPC_SUB  - RPC aimed at THIS device (the CON-1 gateway),
//                                i.e. Module B's own settings. Response goes to
//                                MQTT_TOPIC_DEVICE_RPC_RESP + the request id.
//   MQTT_TOPIC_GATEWAY_RPC     - RPC aimed at a child device (NodoC-1 etc).
//                                Module B translates it into the LoRa
//                                "CFG,<key>=<value>" grammar Module C already
//                                speaks, queues it, and sends it during that
//                                node's next post-TX listen window. The reply
//                                is published back on this same topic.
#define MQTT_TOPIC_DEVICE_RPC_SUB "v1/devices/me/rpc/request/+"
#define MQTT_TOPIC_DEVICE_RPC_RESP "v1/devices/me/rpc/response/"
#define MQTT_TOPIC_GATEWAY_RPC "v1/gateway/rpc"
#define MQTT_DOWNLINK_QOS 1

// A node-directed command can only be delivered during the target node's
// ~2s post-TX listen window, which comes round once per its send period
// (5 min by default). Give up and answer the RPC with a timeout after this
// long rather than holding a command queued forever against a dead node.
#define DOWNLINK_QUEUE_TIMEOUT_MS 900000UL  // 15 min = 3 missed windows at 5 min

// Per-reading timestamps: without a "ts" ThingsBoard stamps every reading in
// a batch at receipt time, collapsing a 2-min batch to one instant. Module B
// has no RTC, so it reads network time once per attach via AT+CCLK? (needs
// AT+CTZU=1, set in CONFIG) and timestamps each reading at its LoRa-receive
// moment. If the network never provides time (CCLK year < 2023), the uplink
// falls back to the flat no-"ts" form automatically.
#define NBIOT_MIN_VALID_EPOCH_MS 1672531200000LL  // 2023-01-01T00:00:00Z - anything earlier = "no network time"

// Keepalive set comfortably above NBIOT_BATCH_DEFAULT_SECONDS (below) rather
// than pinging between real sends - see project memory
// "project_thingsboard_mqtt_plan" for why: a PINGREQ cadence tight enough to
// matter (e.g. every 90s) would run ~13x the actual telemetry traffic, all
// of it spent saying nothing, against the SIM's data allowance and Module
// B's battery. Instead: if the MQTT session is found dead at send time
// (either +QMTSTAT fired, or nothing was ever connected), reconnect lazily
// right there before publishing - reconnecting once per batch is cheap,
// holding a connection open with pings is not. Set to the maximum
// AT+QMTCFG="keepalive" allows (3600s) rather than just above the batch
// interval - a higher value costs nothing (it only controls how long the
// broker waits before giving up on us; our own PUBLISH traffic resets its
// timer every send regardless), and it buys margin against a delayed/
// retried send pushing past a tighter value.
#define MQTT_KEEPALIVE_S 3600UL

// JSON is far more verbose than the legacy binary batch frame (~15 fields/
// reading as "key":value text runs well over 100 bytes/reading, vs. the
// binary format's fixed 30) - sized generously for a full
// NBIOT_BATCH_DEFAULT_READINGS-reading batch with room to spare, not
// trimmed tight the way the old NBIOT_PAYLOAD_MAX_BYTES was.
#define MQTT_PAYLOAD_MAX_BYTES 4096

// ---------- Batching ----------
// Uplink payload is a packed binary frame (see nbiot/NbiotProtocol.h), not
// text CSV: 10 full 13-field readings as text (matching TelemetryParser's
// schema, one line per reading) runs to ~700-800 bytes even at 1 decimal
// place, well over budget. Binary gets one reading down to
// kBatchRecordSize (30) bytes, so the numbers below are chosen to keep
// worst-case payload size (see the static_assert next to kBatchRecordSize)
// under NBIOT_PAYLOAD_MAX_BYTES with room to spare.
// Legacy raw-UDP binary-frame path only (retired ModemNBIoT + its host test).
// The live MQTT path uses MQTT_BATCH_* in the MQTT section above instead.
#define NBIOT_BATCH_DEFAULT_READINGS 10
#define NBIOT_BATCH_DEFAULT_SECONDS (20UL * 60UL)
#define NBIOT_RING_CAPACITY 24  // > max batch so a slow upload doesn't force an immediate drop
#define NBIOT_PAYLOAD_MAX_BYTES 500

// Ceiling used by NbiotProtocol.h's batch-frame static_assert. The downlink
// CFG mechanism that used to retune this at runtime (ModemNBIoT/legacy UDP
// path) has been removed with that class - ModemNBIoTMqtt doesn't do
// downlink yet (see its header for why), so this is compile-time fixed.
#define NBIOT_CFG_READINGS_MAX 16  // (500 - kBatchHeaderSize) / kBatchRecordSize, see static_assert

// ---------- State timeouts (ms) - every state in ModemNBIoT::tick() has one ----------
#define NBIOT_TIMEOUT_POWERING_MS 3000
// Time allowed for the modem to answer a bare AT after PWRKEY is released.
//
// RAISED 8000 -> 30000 (2026-09-11). The old budget gave the BC660K-GL only
// ~9.7s from PWRKEY release to its first AT reply before the firmware declared
// failure and CUT ITS POWER: PWRKEY is released at t=1300ms, POWERING hands over
// at t=3000ms, and WAIT_AT gave up 8s later. A BC660K-GL routinely needs longer
// than that to reach a UART-ready state, so the recovery path was power-cycling a
// modem that was still booting - forever, since each retry restarted the same
// race. Observed symptom: "-> AT" repeating indefinitely with not one "<- OK".
//
// This must stay comfortably longer than the module's worst-case boot time. It
// costs nothing when the modem is healthy, because WAIT_AT exits as soon as the
// first OK arrives - it is a ceiling, not a delay.
#define NBIOT_TIMEOUT_WAIT_AT_MS 30000
#define NBIOT_TIMEOUT_CONFIG_MS 8000
#define NBIOT_TIMEOUT_ATTACH_MS 60000  // AT+CEREG? polling until state 1 or 5
#define NBIOT_TIMEOUT_SOCKET_MS 15000  // covers OK ack + the async +QIOPEN URC
#define NBIOT_TIMEOUT_SEND_MS 10000
#define NBIOT_AT_CMD_TIMEOUT_MS 5000  // ceiling for a plain query/set command (AT, ATE0, CPIN?, ...)
#define NBIOT_ATTACH_POLL_INTERVAL_MS 2000  // how often AT+CEREG? is re-issued while attaching
#define NBIOT_AT_PING_GAP_MS 1000  // how often a bare "AT" liveness ping is retried in WAIT_AT
#define NBIOT_CSQ_POLL_INTERVAL_MS 30000  // how often RSSI is refreshed for the dashboard while IDLE

// ---------- Escalating recovery / backoff ----------
// retry send -> reopen socket -> re-attach -> power cycle. A cold power cycle
// forces a full network attach (slow - can be tens of seconds - and it's the
// most expensive thing this modem does), so it must only ever be the last
// resort, never the first response to a single failed send.
#define NBIOT_MAX_SEND_RETRIES 2
#define NBIOT_MAX_SOCKET_RETRIES 2
#define NBIOT_MAX_ATTACH_RETRIES 2
#define NBIOT_BACKOFF_BASE_MS 2000
#define NBIOT_BACKOFF_MAX_MS 120000
// EN held deasserted this long before re-asserting on a power cycle. RAISED
// 500 -> 3000 (2026-09-11): 500ms is not long enough for the module's own supply
// rail to actually collapse, so the "power cycle" could leave it half-powered in
// an undefined state rather than giving it the clean cold start intended.
#define NBIOT_POWER_OFF_SETTLE_MS 3000
