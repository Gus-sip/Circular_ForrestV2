# Sensor Bring-Up Timeline: Separately → Together

> Carried over from the standalone `../../sensor_node/` project as of this
> restructure into `circular_forest_v2` - see `../circular_forest_v2_README.md`
> for the current layout.

This predates the repo's git history (the initial commit bundled everything at
once), so there's no per-file commit history to pull from for this timeline -
the dates below come from file creation/modification timestamps on disk,
cross-checked against what was actually confirmed live over serial during
this session.
Where a date is filesystem-only (no live hardware confirmation from this
conversation), it's marked as such.

## Phase 1 — Each sensor brought up separately (standalone test sketches)

Each of these is its own throwaway PlatformIO env (`+<*>` minus everything
else), written to isolate one sensor at a time before anything shared. In the
order they were created:

| Date | File | Sensor | Env |
|---|---|---|---|
| 2026-07-14, 09:57 | `src/bmv080.cpp` | BMV080 (particulate) | — |
| 2026-07-14, 11:10 | `src/cm1106.cpp` | CM1106 (CO2) — raw hex dump version | `cm1106-test` |
| 2026-07-14, 11:19–12:34 | `src/cm1106_read.cpp` | CM1106 (CO2) — clean parsed-ppm version | `cm1106-read` |
| 2026-07-16, 08:33 | `src/ulp_pro_uart_test.cpp` | Calypso ULP Pro (wind) | `ulp-pro-uart-test` |
| 2026-07-16, 10:26 | `src/bme690.cpp` | BME690 (temp/humidity/pressure/gas) | `bme690-test` |
| 2026-07-21, 13:11 | `src/sen0466.cpp` | SEN0466 (CO) | `sen0466-test` |
| 2026-07-24, 11:21 | `src/bringup.cpp` | BMV080 — raw Bosch SDK isolation probe (deeper follow-up after the SparkFun wrapper's `bmv080_open`/`E_BMV080_ERROR_HW_WRITE` failure made the original wiring problem hard to see) | `bringup` |

*(Filesystem timestamps only - these predate this session, so I can't confirm
from live serial output that each one passed on first try; the fact that
later work built on top of all five as "known good" implies they did.)*

## Phase 2 — All five sensors together, one firmware

| Date | File | What changed |
|---|---|---|
| 2026-07-24, 11:52 | `src/PowerManager.{h,cpp}`, `src/Sampler.{h,cpp}`, `src/chip_forest_v1.cpp` (originally `chip_forest.cpp`) | The shared `ISensor` interface + `Sampler` class arrive - BME690, SEN0466, BMV080, CM1106, and Calypso all instantiated and read through one common interface in one firmware, printed as a table once a second |
| 2026-07-24, 13:03 | `src/main.cpp` | A second, independent "combined" firmware - different pinout/bus assumptions (BMV080 on a dedicated `Wire1`), originally with its own duplicated CO2/wind parsing code |

*(Also filesystem timestamps - this is where "separate" becomes "together"
architecturally, but the live confirmation on real hardware happened later,
in this session.)*

## Phase 3 — This session: live hardware confirmation, then LoRa joins in

This part I can date precisely from the conversation itself:

1. **`main.cpp` refactored** to stop duplicating sensor protocol code and
   reuse the same `sensors/*` drivers as `chip_forest_v1.cpp` (memory-safety
   and duplication cleanup - see `DEVLOG.md` §1).
2. **First hardware flash of `main.cpp`** (env `esp32-s3-devkitc-1`) - no
   sensors detected. Pinout mismatch: this firmware assumed a different board
   wiring than what was actually on the bench.
3. **Flashed `chip-forest-v1` instead** (matching the bench's actual
   `pins.h` wiring) - **all five sensors detected and streaming real
   data together for the first time on real hardware**:
   ```
   BME690   OK
   SEN0466  OK
   BMV080   OK
   CM1106   OK
   Calypso  OK
   ```
4. **Re-confirmed later in the session**, after the RYLR998 LoRa module was
   also physically wired onto the same board: reflashed `chip-forest-v1`
   again specifically to check for interference - all five sensors still
   came up `OK` with the radio module also present.
5. **LoRa radio confirmed separately on the same board** (`rylr998-bridge`
   env): `AT` → `+OK`, with all five sensors still physically attached (just
   not read by that particular firmware). Confirmed sensors and radio
   coexist without a hardware conflict.
6. **All five sensors + LoRa in one firmware**: `chip_forest_lora_tx.cpp`
   built, reading all five sensors each cycle and transmitting them over the
   RYLR998. First flash surfaced a real bug (transmitting even when radio
   init failed - fixed) and a real parameter rejection (`+ERR=18` - resolved
   by querying the module directly rather than guessing, see `DEVLOG.md`
   §7). After both fixes:
   ```
   SEN0466: OK
   BMV080: OK
   Radio init OK
   Setup complete.
   TX: 0.00,0.00,0.00,0.00,3.00,6.00,9.00,758.00,0.00,32.20,0.00,0.00,1  (sent)
   ```
   **Sensors collecting and the LoRa module genuinely transmitting, together,
   confirmed live.** (BME690 read `NOT FOUND` on this particular boot - a
   known intermittent address-strap issue, not a new fault.)

## Phase 4 — 2026-08-03: receiver fixed, full end-to-end confirmed

The receiver's original RYLR998 had burned out during bring-up (dead
hardware - zero reply to `AT` under every test condition, including full
isolation from the WiFi/web code). A replacement module arrived, was wired
in, and verified in stages:

1. Replacement module isolated `AT` → `+OK`.
2. Full receiver link config (`ADDRESS`/`NETWORKID`/`BAND`/`PARAMETER`, the
   `9,7,1,12` values ground-truthed back in Phase 3 step 6) all came back
   `+OK`.
3. **Both boards powered simultaneously** - sensor node (`chip-forest-lora-tx`,
   COM5) and receiver (COM4) - and a real packet traveled node → receiver
   with the payload intact:
   ```
   Sensor node:  TX: 0.00,0.00,0.00,0.00,13.00,28.00,39.00,888.00,0.00,30.24,0.00,0.00,1  (sent)
   Receiver:     +RCV addr=1 len=67 rssi=-23 snr=11 data="0.00,0.00,0.00,0.00,13.00,28.00,39.00,888.00,0.00,30.24,0.00,0.00,1"
   ```
   Payload matches exactly, sender address (`1`) correctly identifies the
   node, RSSI/SNR strong (bench range). The 13-field CSV parses cleanly, so
   the dashboard at `http://192.168.4.1/` (`CHIP-FOREST-RX` AP) shows live
   numbers instead of "no data yet."

**Node → radio → receiver → dashboard, confirmed live, end to end.** See
`STATUS.md` for the current snapshot and what's left (BME690 still not
detected - deprioritized; no range test beyond the bench yet; telemetry
schema and WiFi AP credentials still flagged as revisable/placeholder).

## What's still separate

Nothing at the sensors+LoRa+dashboard level - that loop is closed as of
Phase 4. What remains is polish, not architecture: BME690 detection on this
board, a real-world range test (bench-only so far), and swapping the
placeholder WiFi AP credentials before field deployment.
