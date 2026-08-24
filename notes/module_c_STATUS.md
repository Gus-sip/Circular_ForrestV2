# CHIP FOREST — Session Status

> Carried over from the standalone `../../sensor_node/` project as of this
> restructure into `circular_forest_v2`. Paths and env names below reflect
> that project's layout, not this one's `src/module_c/sensor/` +
> `src/module_c/communications/` split - see `../circular_forest_v2_README.md`
> for the current layout.

Snapshot of where the two-repo LoRa sensor system stands, written to pick back
up from without re-deriving context. Covers **two separate PlatformIO
projects**:

- `sensor_node` (this repo) — the sensor node, all 5 sensors + LoRa TX
- `relay` (sibling folder) — the ground-station receiver, LoRa RX
  + WiFi dashboard, no sensors

## TL;DR — what's confirmed working, what isn't

| Piece | Status |
|---|---|
| Sensor node: 4 of 5 sensors (SEN0466, BMV080, CM1106, Calypso) | ✅ confirmed on real hardware |
| Sensor node: BME690 | ❌ **not detected** (`NOT FOUND` on every recent boot, not just intermittent) - deprioritized for now, see below |
| Sensor node: RYLR998 answers AT commands, full link config succeeds | ✅ confirmed (`AT`/`ADDRESS`/`NETWORKID`/`BAND`/`PARAMETER` all `+OK`) |
| Sensor node: sensors + LoRa module coexist without interference | ✅ confirmed (tested back-to-back on the same board) |
| Sensor node: combined sensors→LoRa TX firmware (`chip-forest-lora-tx`) | ✅ confirmed - sensors read, radio configured, real `TX: ...  (sent)` observed live |
| Receiver: WiFi AP + web dashboard | ✅ confirmed (AP broadcasts, page serves) |
| Receiver: RYLR998 module (replacement) | ✅ confirmed - isolated `AT`→`+OK`, then full link config (`ADDRESS`/`NETWORKID`/`BAND`/`PARAMETER`) all `+OK` |
| **End-to-end (node TX → receiver RX → dashboard)** | ✅ **confirmed** - a real packet traveled node → receiver with the payload intact (see below) |

## Root cause found (and fixed): receiver's original RYLR998 was dead hardware

The original receiver module gave zero reply to `AT` under every test
condition, including full isolation from WiFi/web code - consistent with a
burned-out unit, not a wiring or code bug. The sensor node's own module
(different physical unit) worked throughout, confirming it was module-specific.
A replacement module was ordered, installed, and verified: isolated `AT`→`+OK`
first, then the full receiver firmware's link config all succeeded.

## End-to-end test result

With both boards powered simultaneously (sensor node on COM5, receiver on
COM4):

```
Sensor node:  TX: 0.00,0.00,0.00,0.00,13.00,28.00,39.00,888.00,0.00,30.24,0.00,0.00,1  (sent)
Receiver:     +RCV addr=1 len=67 rssi=-23 snr=11 data="0.00,0.00,0.00,0.00,13.00,28.00,39.00,888.00,0.00,30.24,0.00,0.00,1"
```

Payload matches exactly, sender address (`1`) correctly identifies the node,
RSSI/SNR strong (as expected at bench range). The 13-field CSV parses
cleanly, so the dashboard at `http://192.168.4.1/` (join the `CHIP-FOREST-RX`
AP) should show live numbers rather than "no data yet."

## What's left (polish, not blockers)

- **Telemetry payload schema** (13-field CSV: `temp,hum,pres,gas,pm1,pm25,pm10,co2,co,coTemp,windAngle,windSpeed,windValid`)
  works and both ends agree, but it was invented for this project rather than
  drawn from a spec - treat it as revisable, and update both
  `chip_forest_lora_tx.cpp` and `TelemetryParser.h` together if it changes.
- **WiFi AP credentials** (`CHIP-FOREST-RX` / `chipforest1`) are placeholders
  in the receiver's `include/Config.h` — change before real field deployment.
- **`rylr998-demo`'s band (868500000)** does NOT match the real deployment
  band (868000000) — that demo was only ever for range-testing two
  sensor-node-repo boards against each other, not for talking to the receiver.
- **BME690 is not being detected** (`NOT FOUND` on every recent boot, not just
  occasionally as first thought) - a real hardware/wiring issue on this board,
  not a code bug (same firmware detects the other 4 sensors fine). Checked
  wiring/reflashed multiple times with the same result. **Deliberately
  deprioritized for now** - the rest of the pipeline (SEN0466, BMV080, CM1106,
  Calypso, LoRa TX/RX, dashboard) doesn't depend on it, so temp/humidity/
  pressure/gas will read `0.00` in the CSV until this is revisited.
- **No range test done yet** - both boards were tested on the same bench.
  Actual deployment range/obstruction behavior is unverified.

## LoRa link parameters (confirmed, not placeholders)

These came from actually querying the sensor node's module via `AT+PARAMETER?`
(`src/rylr998_param_probe.cpp`) after the generic "commonly documented"
RYLR998 defaults (`7,7,1,4`) came back `+ERR=18` - not guessed:

```
AT+BAND=868000000       (EU868, Spain deployment)
AT+NETWORKID=5
AT+PARAMETER=9,7,1,12   (spreadingFactor, bandwidth, codingRate, preamble)
node address = 1, receiver address = 2
```

Both `chip_forest_lora_tx.cpp` (this repo) and `relay`'s
`Config.h` use these exact values. **If the replacement receiver module ever
rejects these too, re-run `rylr998-param-probe` against it rather than
assuming - a different physical unit could legitimately have different
accepted defaults.**

TX cadence is duty-cycle-constrained, not arbitrary: at SF9/BW7/CR1/preamble12
with the ~80-byte sensor payload, time-on-air is ~468ms/packet, and EU868's
1% duty-cycle sub-band allows one packet roughly every 47s at the legal limit.
`TX_INTERVAL_MS` in `chip_forest_lora_tx.cpp` is set to 120000 (2.5x margin).
The receiver's `DATA_STALE_MS` (180000) is set above that so the dashboard
doesn't falsely flag "signal lost" between normal transmissions. Recompute
both before changing SF/BW/CR/preamble or payload size - see that file's
header comment for the math.

## Where things live

- `sensor_node/src/chip_forest_lora_tx.cpp` — the combined sensors+LoRa TX
  firmware (env `chip-forest-lora-tx`)
- `sensor_node/src/radio/RYLR998.*` — the LoRa driver (shared by
  `chip_forest_lora_tx.cpp`, `rylr998_bridge.cpp`, `rylr998_demo.cpp`,
  `rylr998_param_probe.cpp`)
- `sensor_node/src/rylr998_param_probe.cpp` — queries a module's actual
  current AT+PARAMETER/BAND/ADDRESS/NETWORKID/VER; use this on the new
  receiver module too if anything doesn't come back `+OK`
- `sensor_node/src/radio/README.md` — RYLR998 wiring/band/parameter notes for
  this repo
- `relay/src/main.cpp` — the receiver firmware (WiFi AP + LoRa RX
  + dashboard)
- `relay/src/telemetry/TelemetryParser.h` — the authoritative
  payload schema
- `relay/README.md` — receiver wiring/WiFi/dashboard notes
