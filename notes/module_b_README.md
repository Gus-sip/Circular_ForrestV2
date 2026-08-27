# CHIP FOREST — LoRa Ground-Station Receiver

Standalone firmware for the receiving ESP32-S3-Zero. This board has no
sensors attached - its only job is to receive telemetry over a RYLR998 LoRa
module and serve it as a live dashboard over WiFi. Separate PlatformIO project
from the sensor node (Module C, `src/module_c/` in this same project,
originally the sibling `../../sensor_node/` project); the two only need to agree on the LoRa
link parameters and the telemetry payload format.

## 2026-08-25 — PCB bring-up session

- **`LORA_EN_PIN` (GPIO13) added.** The physical Module B PCB gates the LoRa
  module's power rail through a transistor (Q4 on the schematic) that this
  firmware never drove - `radio.begin()` was being called with the module
  unpowered. Found via the same kind of pin-scan bring-up that found the
  NB-IoT PWRKEY/channel pins earlier; confirmed LOW = powered on. Now set
  LOW in `setup()`, with a 300ms settle before `radio.begin()`.
- **Module B's firmware in `circular_forest_v2` re-synced against this
  project's current state** - it had drifted onto an older snapshot (still
  on the retired raw-UDP `ModemNBIoT`, missing every NB-IoT hardware fix
  below). Now matches: MQTT/ThingsBoard uplink, hardware-confirmed NB-IoT
  pins/PWRKEY sequence, `sensor.net` APN, `AT+QCGDEFCONT`.
- **OLED status display added** - a GME12864 (SSD1306-compatible 128x64 I2C)
  wired to GPIO1 (SDA) / GPIO2 (SCL), `Config.h`'s `OLED_*` defines. Refreshed
  every 500ms in `loop()`: LoRa RX recency + RSSI/SNR, the NB-IoT/MQTT state
  machine's current state name (`PUBLISHING` while actively sending, etc.),
  MQTT connection status, and sent/failed/dropped counters. Labels are in
  Spanish (`Modulo B`, `inactivo`, `activo`/`caido`, `Env`/`Err`/`Prd`) at the
  user's request; state names themselves stay in English to match the serial
  log and web dashboard. Libraries: `adafruit/Adafruit SSD1306` +
  `adafruit/Adafruit GFX Library`, both in `module-b-main`'s `lib_deps`.
- **NB-IoT signal strength as bars, and a real bug behind it.** The OLED's
  `NBIoT:` line now shows a 4-bar indicator (`drawSignalBars()`) instead of a
  raw dBm number. Fixing it exposed a genuine bug: `AT+CSQ` was only ever
  polled after `ModemNBIoTMqtt` reached `IDLE`, i.e. only after a *successful
  MQTT connect* - so the signal reading (and the bars) stayed empty/unmeasured
  the entire time Module A/the broker was unreachable, even when cellular
  attach and signal were both fine. Fixed by polling `AT+CSQ` once
  immediately after network attach (`tickAttaching()`), independent of
  whether MQTT ever connects. Ported to both `relay` and here.
- **Splash/decorative boot image removed** - was drawn once in `setup()` and
  held until the first status refresh ~1s later (visible as a brief "flash"
  on every reset, which read as a bug until traced back to this). OLED now
  goes straight to the live status screen; `OledSplash.h` deleted.
- **Two physical Module B units now exist**, both flashed identically as
  plain spares (same `LORA_MY_ADDR`, MQTT identity, AP SSID - not the
  distinct-per-cell multi-relay setup floated earlier and then scrapped).
  One board initially couldn't attach to the NB-IoT network at all (stuck at
  `+CEREG: 0,2` / "searching" indefinitely, despite SIM ready and APN config
  succeeding) - moving it next to a window fixed it, confirming this was a
  signal/antenna-positioning issue, not a config problem. The second unit hit
  the identical symptom, but repositioning it (and swapping in first the
  known-good antenna, then a bigger one) made no difference - see below.
- **Second Module B unit's NB-IoT: hardware fault found, not
  antenna/positioning.** Repositioning didn't help this unit like it did the
  first, so built a proper diagnostic tool
  (`pcb_bringup_test/src/NBIoT_SignalDiag_test.cpp`, env
  `nbiot-signal-diag`) that queries `AT+CSQ` (raw signal, works without
  registration), `AT+QENG="servingcell"`, `AT+QCCID` (which SIM is
  physically inserted), and a full `AT+COPS=?` operator scan. Result on this
  unit: SIM reads fine (`+CPIN: READY`, ICCID `89882280666800022158`), `AT`
  itself intermittently drops out entirely (recovers on its own, correlated
  with handling the antenna connector), and `AT+CSQ` consistently reads
  `99,99` ("no signal detectable") regardless of antenna - tried the
  known-good antenna from the working unit, then a bigger one, no change.
  Same firmware confirmed working fine on the other (good) unit, ruling out
  a code issue. Continuity-tested by hand: the GPIO10 (`NBIOT_CHANNEL_PIN`)
  transistor's *control* signal (GPIO -> gate) is fine, but its **3.3V power
  path has a broken/marginal continuity** - not the antenna, not the EN
  (GPIO9) transistor, not firmware. This is the same GPIO10 channel gate
  flagged during last week's bring-up as being on the path needed for the
  module to work at all (see `Config.h`'s NB-IoT section) - a flaky
  connection there plausibly explains both symptoms: intermittent AT
  (enough power to limp along sometimes) and permanently dead RF (never
  enough clean power for the radio to actually operate). Needs a hardware
  repair (trace/solder joint on that specific 3.3V segment) - not a
  firmware fix, and not resolved by this session's end.

## Hardware

| Module pin | ESP32-S3-Zero |
|---|---|
| VDD | 3V3 |
| GND | GND |
| TXD | GPIO4 (ESP RX) |
| RXD | GPIO5 (ESP TX) |
| NRST | floating |

Same crossover convention as the sensor node's radio bring-up: ESP TX (GPIO5)
-> module RXD, ESP RX (GPIO4) -> module TXD.

## LoRa link parameters — must match the sensor node exactly

All in `include/Config.h`. SF/BW/CR/preamble are **not** the generic
"commonly documented" RYLR998 defaults (`7,7,1,4`) this project originally
assumed - that combination came back `+ERR=18` from the actual sensor-node
module. These values were read directly off that module via `AT+PARAMETER?`
(the sensor node's `src/rylr998_param_probe.cpp`) rather than guessed a second
time. If you ever retune them, change both ends together and re-verify with
that same probe:

```cpp
#define LORA_BAND_HZ 868000000UL  // EU868 - confirmed, Spain deployment
#define LORA_NETWORK_ID 5
#define LORA_PARAM_SF 9
#define LORA_PARAM_BW 7
#define LORA_PARAM_CR 1
#define LORA_PARAM_PREAMBLE 12

#define LORA_NODE_ADDR 1  // sensor node's AT+ADDRESS
#define LORA_MY_ADDR 2    // this receiver's own AT+ADDRESS
```

On boot, every `AT` command sent to the module and its reply is printed over
USB serial (115200 baud), so you can confirm the radio actually accepted each
setting rather than trusting a single pass/fail result:

```
Configuring RYLR998 (link parameters must match the sensor node):
  -> AT
  <- +OK
  -> AT+ADDRESS=2
  <- +OK
  -> AT+NETWORKID=5
  <- +OK
  -> AT+BAND=868000000
  <- +OK
  -> AT+PARAMETER=9,7,1,12
  <- +OK
Radio init OK - listening for packets.
```

If any step doesn't print `+OK`, check wiring/baud first (a bad crossover or
wrong baud means *nothing* replies, not just the one command that "failed").

## Telemetry payload schema (provisional)

The sensor node repo's `chip_forest_lora_tx.cpp` encodes readings using this
schema - still provisional (chosen for simplicity, not from a spec), but it's
what the node actually sends now, not a placeholder awaiting a real encoder:
fixed-order CSV, no header, exactly 13 fields, all plain decimal numbers
except the last:

```
temp,hum,pres,gas,pm1,pm25,pm10,co2,co,coTemp,windAngle,windSpeed,windValid
```

`windValid` is a literal `0` or `1`. See `src/module_b/communications/telemetry/TelemetryParser.h` for
the authoritative field list/order. **If the node-side encoding ends up
different, update both `TelemetryParser.cpp` here and whatever encodes the
payload on the node - they must agree exactly**, and a payload that doesn't
split into exactly 13 fields is rejected outright (logged, not guessed at).

## WiFi / finding the dashboard

AP mode - this device creates its own hotspot, no router or existing network
needed:

- **SSID:** `CHIP-FOREST-RX` (see `WIFI_AP_SSID` in `include/Config.h`)
- **Password:** `chipforest1` (see `WIFI_AP_PASSWORD` - **placeholder, change
  before field deployment**)
- **Dashboard URL:** `http://192.168.4.1/` (the ESP32 SoftAP default; also
  printed over serial at boot)

Connect a phone or laptop to that SSID, then browse to the URL above. The page
polls `/data` (JSON) every 3 seconds - no full page reload, no external
scripts/CDNs, works fully offline.

## What the dashboard shows

- **No data yet** (gray banner) - nothing received since boot.
- **Live readings** - a tile grid (temp/humidity/pressure/gas, PM1/2.5/10,
  CO2, CO + its onboard temp, wind angle/speed), plus time-since-last-packet
  and RSSI/SNR of the most recent one.
- **Signal lost** (orange banner) - a packet was received at some point, but
  not within the last `DATA_STALE_MS` (30s, in `include/Config.h`). Stale
  numbers are never shown as if they were live.

## Serial diagnostics

At 115200 baud: the AP's SSID/password/IP, every AT command's exchange during
radio setup, and every received packet as both the raw `+RCV=` line's parsed
fields (sender/len/rssi/snr/data) and, if it matched the schema, a dropped
notice if it didn't.

## Architecture

Diagram below reflects this doc's original standalone project layout; in
`circular_forest_v2` these same files live under `src/module_b/communications/`
(see `../circular_forest_v2_README.md` for the current layout). Note also
that the RYLR998 driver shown under `radio/` here is now the driver shared
with Module C, at `src/shared/radio/`.

```
include/
  Config.h                  - pins, LoRa link params, WiFi AP credentials, staleness timeout
src/
  main.cpp                  - the only file that knows about radio + telemetry + web together
  radio/
    RYLR998.{h,cpp}          - AT-command driver: begin() (blocking, optional debug print),
                                send() (blocking, kept for a possible future LoRa ACK),
                                poll() (non-blocking, parses +RCV= lines)
  telemetry/
    SensorSnapshot.h         - plain data struct; the web layer's only view of "the latest reading"
    TelemetryParser.{h,cpp}  - payload text -> SensorSnapshot; knows nothing about LoRa or HTTP
  web/
    DashboardPage.h          - inline HTML/CSS/JS (PROGMEM), fetch()-polls /data every 3s
```

`radio/` and `telemetry/` don't include anything from `web/`, and `web/` (the
page itself) doesn't know a radio exists - `main.cpp` is the only place all
three are wired together, via the shared `SensorSnapshot` global.
