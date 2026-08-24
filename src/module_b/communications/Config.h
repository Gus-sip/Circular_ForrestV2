#pragma once

// LoRa ground-station receiver configuration. Everything tunable lives here,
// same convention as the sensor-node repo's Config.h.

// ---------- RYLR998 wiring (ESP32-S3-Zero) ----------
// Same crossover convention as the sensor node: module TXD -> ESP RX, module
// RXD <- ESP TX. Not wired to any sensors on this board - LoRa module only.
#define LORA_RX_PIN 4  // module TXD -> ESP RX
#define LORA_TX_PIN 5  // module RXD <- ESP TX
#define LORA_BAUD 115200

// ---------- LoRa link parameters - MUST match the sensor node exactly ----------
// SF/BW/CR/preamble are NOT the generic "commonly documented" RYLR998 defaults
// (7,7,1,4) originally assumed here - that combination came back +ERR=18 from
// the actual sensor-node-side module. These values were read directly off that
// module via AT+PARAMETER? (pp1-optimize's src/rylr998_param_probe.cpp)
// instead of guessed a second time. If you ever change these, change them on
// both ends together and re-verify with that same probe.
#define LORA_BAND_HZ 868000000UL  // EU868 - Spain deployment (confirmed, do not change)
#define LORA_NETWORK_ID 5
#define LORA_PARAM_SF 9
#define LORA_PARAM_BW 7
#define LORA_PARAM_CR 1
#define LORA_PARAM_PREAMBLE 12

#define LORA_NODE_ADDR 1  // sensor node's AT+ADDRESS
#define LORA_MY_ADDR 2    // this receiver's own AT+ADDRESS - must differ from the node's

// ---------- WiFi AP ----------
// PLACEHOLDER credentials - change before field deployment. WPA2 requires an
// 8-63 char passphrase.
#define WIFI_AP_SSID "CHIP-FOREST-RX"
#define WIFI_AP_PASSWORD "chipforest1"

// ---------- Dashboard staleness ----------
// If no packet has arrived within this window, the dashboard shows "signal
// lost" instead of the last numbers, so stale data is never mistaken for live.
// Must stay comfortably above the node's default TX interval (120s, in
// pp1-optimize's chip_forest_lora_tx.cpp) or the dashboard would flag "lost"
// between every normal transmission. Kept at ~2.5x that default (same margin
// convention chip_forest_lora_tx.cpp uses for its own duty-cycle floor), not
// exactly 1 missed packet, since the node's TX interval is now remotely
// tunable via downlink CFG,INTERVAL=... - a fixed threshold can't track an
// arbitrarily-pushed interval, so a much longer pushed interval will still
// eventually show "stale" before its own next packet. Tracking the
// last-applied interval dynamically is future work, contingent on Module B
// actually relaying CFG downlink to the node (it doesn't yet).
#define DATA_STALE_MS 300000

// ---------- NB-IoT uplink (Quectel BC660K-GL, "Module B" cellular backhaul) ----------
// UART2 - deliberately NOT Serial1 (UART1), which the RYLR998 above already
// owns on GPIO4/5. ESP32-S3 only has three UART controllers (0/1/2) and
// USB-CDC is separate from all of them; reusing UART1 for the modem would
// collide with the LoRa radio the moment both run in the same firmware.
// (src/bc660k_bridge.cpp's standalone prototype used HardwareSerial(1) on
// GP5/GP10 - that never collided only because its build env excludes main.cpp
// and the radio driver. Don't copy that instantiation into the real hub.)
#define NBIOT_RX_PIN 8  // ESP RX <- module TXD (breakout P14 TXD)
#define NBIOT_TX_PIN 7  // ESP TX -> module RXD (breakout P14 RXD)
#define NBIOT_BAUD 115200

// Power gating on the production PCB: P-FET, active LOW enables 3V3_NBIoT_EN
// (same polarity/topology as bc660k_bridge.cpp's NBIOT_EN_PIN - LOW = powered).
#define NBIOT_EN_PIN 9
#define NBIOT_EN_ACTIVE LOW
#define NBIOT_DISABLE (!NBIOT_EN_ACTIVE)

// RST, P1 pin 5 (the schematic mislabels this net "RTS" - it is not a UART
// flow-control line, it's the module's reset input). Active-low pulse.
#define NBIOT_RST_PIN 6
#define NBIOT_RST_ACTIVE LOW
#define NBIOT_RST_PULSE_MS 200

// PWRKEY (PWR, P1 pin 8) is unconnected on this board - no define needed. The
// breakout auto-boots once VIN is present, consistent with what bc660k_bridge.cpp
// observed on real hardware. If a future rev wires PWRKEY, this file is where
// its pin/pulse timing would go.

// ---------- Network (1NCE SIM) ----------
#define NBIOT_APN "iot.1nce.net"
// BC660K-GL's own PDP context is numbered from 0, NOT 1 - the BC66 docs (and
// most Quectel AT-command examples) use context 1, which is a common trap on
// this specific module. Confirmed by hand: AT+CGDCONT=0,... is what the
// verified bring-up sequence used.
#define NBIOT_CONTEXT_ID 0

// TODO fill in before flashing - left blank deliberately rather than a
// plausible-looking fake value, so a forgotten edit fails loudly (empty host
// string / port 0) instead of silently sending traffic nowhere useful.
#define NBIOT_SERVER_HOST ""
#define NBIOT_SERVER_PORT 0
#define NBIOT_LOCAL_PORT 0  // 0 = let the modem pick an ephemeral source port
#define NBIOT_CONNECT_ID 0  // AT+QIOPEN/QISEND/QIRD/QICLOSE session id - only one socket is ever open, so a fixed 0 is fine

// 1NCE's outbound NAT means nothing can dial in - the socket the device opens
// for uplink is also the only path anything downlink can ever arrive on.

// ---------- Batching ----------
// Uplink payload is a packed binary frame (see nbiot/NbiotProtocol.h), not
// text CSV: 10 full 13-field readings as text (matching TelemetryParser's
// schema, one line per reading) runs to ~700-800 bytes even at 1 decimal
// place, well over budget. Binary gets one reading down to
// kBatchRecordSize (30) bytes, so the numbers below are chosen to keep
// worst-case payload size (see the static_assert next to kBatchRecordSize)
// under NBIOT_PAYLOAD_MAX_BYTES with room to spare.
#define NBIOT_BATCH_DEFAULT_READINGS 10
#define NBIOT_BATCH_DEFAULT_SECONDS (20UL * 60UL)
#define NBIOT_RING_CAPACITY 24  // > max batch so a slow upload doesn't force an immediate drop
#define NBIOT_PAYLOAD_MAX_BYTES 500

// Downlink CFG can retune the two batching knobs above (see the ACK,<seq>,CFG,...
// grammar in NbiotProtocol.h) but only within these compile-time ranges - a
// value outside its range is dropped, never applied, and never stops uplink.
#define NBIOT_CFG_READINGS_MIN 1
#define NBIOT_CFG_READINGS_MAX 16  // (500 - kBatchHeaderSize) / kBatchRecordSize, see static_assert
#define NBIOT_CFG_SECONDS_MIN 60UL
#define NBIOT_CFG_SECONDS_MAX (6UL * 3600UL)

// ---------- State timeouts (ms) - every state in ModemNBIoT::tick() has one ----------
#define NBIOT_TIMEOUT_POWERING_MS 3000
#define NBIOT_TIMEOUT_WAIT_AT_MS 8000
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
#define NBIOT_POWER_OFF_SETTLE_MS 500  // EN held deasserted this long before re-asserting on a power cycle

// ---------- PSM ----------
// Cheaper to stay attached and let the modem sleep (AT+QSCLK=1) between
// batches than to drop the socket/attach and pay full re-attach cost next
// time. Idle this long with nothing queued before requesting sleep; woken
// (AT+QSCLK=0) as soon as a batch is ready to send.
#define NBIOT_PSM_IDLE_MS 5000
