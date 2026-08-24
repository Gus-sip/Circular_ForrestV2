# CHIP FOREST — Development Log

> Carried over from the standalone `../../sensor_node/` project as of this
> restructure into `circular_forest_v2`. Paths and env names below reflect
> that project's layout, not this one's `src/module_c/sensor/` +
> `src/module_c/communications/` split - see `../circular_forest_v2_README.md`
> for the current layout.

A narrative record of everything done across this project, in the order it
happened, including the reasoning behind decisions and the dead ends along
the way. Covers **two PlatformIO projects**:

- `sensor_node` (this repo) — the sensor node
- `relay` (sibling folder) — the ground-station receiver

For the current live state (what's flashed where, what's blocked, what's a
placeholder), see [`STATUS.md`](STATUS.md) instead — this file is history,
that one is a snapshot.

---

## 1. Memory-safety and duplication cleanup (sensor node)

**Starting point:** the repo had two parallel implementations of the sensor
node - `main.cpp` (a hand-rolled "combined production firmware" reimplementing
CO2 UART framing and wind NMEA parsing inline) and `chip_forest.cpp` +
`sensors/*` (a cleaner ISensor-based bring-up harness). `main.cpp` also used
Arduino `String` for continuous NMEA parsing, which risks heap fragmentation
on a device meant to run for days.

**Changes made:**
- `sensors/CalypsoSensor` rewritten off `String` entirely - fixed 120-byte
  `char` buffer, in-place tokenizing instead of `substring()`/concatenation.
- `sensors/Bmv080Sensor` gained an optional `TwoWire&` constructor parameter
  (default `Wire`, so existing call sites don't break) so it can run on a
  second I2C bus - needed because `main.cpp`'s board wires BMV080 on a
  dedicated `Wire1` bus.
- `sensors/Cm1106Sensor` gained a `kNoEnPin` sentinel and made the warm-up
  delay conditional (`warmupMs == 0` skips it) - `main.cpp`'s CM1106 breakout
  has no EN pin at all, unlike the bench harness's wiring.
- `main.cpp` rewritten to instantiate and call the same `sensors/*` drivers
  instead of reimplementing their protocols, cutting roughly 150 lines of
  duplicated/heap-unsafe logic. Custom per-sensor print formatting was kept
  (temperature/humidity/etc. labels), just sourced from `Reading` structs
  instead of raw protocol parsing.
- `platformio.ini`: the `esp32-s3-devkitc-1` env stopped excluding
  `sensors/*` from its build, since `main.cpp` now depends on it.

**Left untouched, deliberately:** the standalone one-off bring-up/test
sketches (`cm1106.cpp`, `bme690.cpp`, `sen0466.cpp`, `bmv080.cpp` /
`bringup.cpp`, `ulp_pro_uart_test.cpp`) - these are explicitly documented as
throwaway wiring-verification probes, not production code, so folding them
into the shared driver layer would work against their stated purpose.

**Verified:** both `esp32-s3-devkitc-1` and the bring-up env built clean.

---

## 2. Hardware bring-up of the sensor node board

Flashed the refactored `esp32-s3-devkitc-1` firmware to the physical board -
no sensors detected. Investigation showed `main.cpp`'s assumed pinout
(I2C on GPIO8/9 + dedicated `Wire1` for BMV080, GPIO7/4 for CM1106) didn't
match how this particular bench board was actually wired, which followed the
bring-up harness's `pins.h` layout instead (shared I2C on GPIO1/2, CM1106 on
GPIO6/7 with EN on GPIO3, wind UART on GPIO8/9).

Flashed the `chip-forest` bring-up env instead (matching `pins.h`) - **all
five sensors (BME690, SEN0466, BMV080, CM1106, Calypso) detected and
streaming real data**, confirming the refactored `sensors/*` drivers work
correctly against actual hardware.

---

## 3. Prototype versioning: `chip_forest` → `chip_forest_v1`

Renamed the working bring-up prototype to mark it as a versioned milestone:
- `src/chip_forest.cpp` → `src/chip_forest_v1.cpp`
- `platformio.ini`: `[env:chip-forest]` → `[env:chip-forest-v1]`, plus every
  other env's `build_src_filter` exclusion of the old filename updated
- `README.md` updated throughout
- Renaming the env moved its PlatformIO libdeps folder, so the vendored
  Bosch BMV080 SDK blobs (`bosch_bmv080_sdk/*`) had to be manually re-copied
  into the new `.pio/libdeps/chip-forest-v1/...` path (same manual step the
  README already documented for a clean checkout)

Verified: `chip-forest-v1` builds and runs identically to the old `chip-forest`
env on real hardware.

---

## 4. RYLR998 LoRa bring-up (sensor node side)

Checked the repo first for any existing LoRa code or vendored REYAX AT
reference docs - none existed, so this was built from scratch as a new,
fully standalone addition (no sensor code touched).

**Stage 1 - `rylr998_bridge.cpp`** (env `rylr998-bridge`): sends `AT` at boot,
prints the reply, then becomes a transparent USB↔UART1 bridge for driving the
module by hand. Region confirmed **EU868** with the project owner before
writing anything band-specific.

- **Hardware result:** `AT` → `+OK` on first try. Wiring (GPIO4 RX / GPIO5 TX
  crossover, matching the schematic's MCU-relative net naming) confirmed
  correct.

**Stage 2 - `radio/RYLR998.{h,cpp}` driver + `rylr998_demo.cpp`** (env
`rylr998-demo`): `begin()`/`send()` are blocking with a bounded timeout;
`poll()` is fully non-blocking, tolerates partial lines across reads, and
parses `+RCV=<addr>,<len>,<data>,<rssi>,<snr>` in a length-bounded way (not a
naive comma-split, so payload bytes containing a comma don't desync later
fields). A two-role (`#define ROLE_TX`/`ROLE_RX`) counter-and-RSSI demo was
built on top, plus `radio/README.md` documenting wiring, band/parameter
tuning, and duty-cycle constraints.

- **Hardware result:** radio confirmed live again via the bridge sketch.

---

## 5. LoRa ground-station receiver (`relay`, new project)

Built as an explicitly separate, fresh PlatformIO project (per the requesting
prompt) rather than folded into the sensor node repo - it's a different
physical board (ESP32-S3-Zero) with no sensors, whose only job is receiving
telemetry and serving a dashboard.

**Decisions made (asked rather than guessed, per the prompt's own
instructions):** ESP32-S3-Zero board; **AP mode** WiFi (self-contained
hotspot, no router needed - right call for field deployment); WiFi
credentials as config constants (not hardcoded inline); new sibling project
folder. Where the user said "choose something simple": plain `WebServer.h` +
`WiFi.h` (bundled in the ESP32 Arduino core, zero extra `lib_deps`) over the
async web server library; placeholder AP SSID/password; and - since the
sensor node had no real telemetry encoder yet - a simple fixed-order 13-field
CSV payload schema, chosen and documented as provisional on both ends.
**Structure**, kept deliberately decoupled per the prompt's requirement that
"the parser shouldn't know a web server exists":
- `radio/RYLR998.{h,cpp}` - transport only (ported from the sensor-node repo,
  plus one addition: an optional `Print *debug` parameter on `begin()` so
  every AT command and its reply prints live - this turned out to matter a
  lot later, see §7).
- `telemetry/SensorSnapshot.h` + `TelemetryParser.{h,cpp}` - payload text →
  struct, no knowledge of LoRa or HTTP.
- `web/DashboardPage.h` - inline PROGMEM HTML/CSS/JS, `fetch()`-polls `/data`
  every 3s, distinct "no data yet" / "signal lost" (stale > 30s→later 180s,
  see §7) / live states.
- `main.cpp` - the only file that wires all three together via one shared
  `SensorSnapshot` global.

**Verified:** builds clean (57.5% flash / 14% RAM).

---

## 6. Receiver hardware bring-up — the radio silence mystery

Flashed to the physical receiver board (a different ESP32-S3-Zero, distinct
MAC from the sensor node). WiFi AP and web server came up fine, but
`radio.begin()` failed - **zero reply to a plain `AT`**, not even a rejection.

To rule out any interaction between the newly-added WiFi/web server code and
the radio, built an isolated diagnostic (`radio_bridge_test.cpp`, env
`radio-bridge`) with no WiFi/web code at all, same wiring. **Still zero
reply.** This ruled out firmware/WiFi-stack interference entirely and pointed
squarely at hardware: wiring, ground, power, or baud - confirmed by having
the user re-check that the module was in fact wired to GPIO4/5 as documented
(it was).

**Cross-check:** flashed the sensor node board with both its sensors *and*
its own LoRa module physically attached at the same time, to rule out any
general interference concern - `chip-forest-v1` (all 5 sensors) and
`rylr998-bridge` (radio) both worked perfectly together on that board. This
confirmed the silence was specific to the receiver's module/board, not a
systemic issue.

**Root cause, later confirmed by the user: the receiver's RYLR998 module had
physically burned out.** Zero reply to `AT` under every test condition,
including full isolation, is exactly the signature of dead hardware - in
hindsight, no amount of further wiring/firmware debugging would have found
anything, since there was nothing to find. A replacement module was ordered.
Before wiring it in, flagged the likely causes of the first burnout to check
first (VDD actually 3.3V and not 5V, correct polarity, no pin-to-pin short) so
the replacement doesn't suffer the same fate.

---

## 7. Combined sensors + LoRa TX firmware (`chip_forest_lora_tx.cpp`)

With both the sensor drivers and the LoRa driver independently proven, built
the actual integration: all five sensors read each cycle, encoded into the
13-field CSV schema the receiver expects, and transmitted via `radio.send()`.

**A bug found via real hardware testing, not review:** the first flash showed
`radio.begin()` failing (see below) but the firmware sent anyway, printing a
false `TX: ...  (sent)`. The code never checked whether `begin()` actually
succeeded before calling `send()` in `loop()`. Fixed by gating the send on a
`radioReady` flag - now it prints "TX skipped (radio not initialized)"
instead of transmitting under parameters the module never accepted.

**A parameter rejection, resolved by asking the hardware instead of guessing
again:** `AT+PARAMETER=7,7,1,4` (the "commonly documented" RYLR998 default
assumed everywhere up to this point) came back `+ERR=18` - and since the
three commands *before* it (`AT`, `AT+ADDRESS`, `AT+NETWORKID`, `AT+BAND`) all
succeeded, this ruled out wiring/baud and pointed specifically at that
parameter combination. Rather than guess a second combination, built
`rylr998_param_probe.cpp` to query the module directly: `AT+PARAMETER?` came
back **`9,7,1,12`** - the module's actual accepted values. Ground truth, not
a second guess.

**That parameter change had a knock-on effect on legal duty-cycle timing:**
SF9 (vs. the assumed SF7) and a longer preamble (12 vs. 4) roughly triple the
per-packet airtime. Recomputed the EU868 duty-cycle math (Semtech's public
LoRa airtime formula) for the new parameters and:
- Raised `chip_forest_lora_tx.cpp`'s `TX_INTERVAL_MS` from 30s → **120s**
  (legal floor ~47s at these parameters; keeps >2.5x margin)
- Raised the receiver's `DATA_STALE_MS` from 30s → **180s** (so the dashboard
  doesn't falsely show "signal lost" between normal transmissions)
- Fixed `rylr998_demo.cpp`'s same wrong assumed parameters and recomputed
  *its* interval too (5s → 30s, for its much smaller counter payload)
- Corrected `relay`'s `Config.h` and README to the same `9,7,1,12`
  values, since both ends of a LoRa link must agree exactly
- Updated `radio/README.md` in both repos to stop citing the wrong defaults

**Final verification on real hardware:** rebuilt, reflashed, and confirmed
the full AT sequence (`AT`/`ADDRESS`/`NETWORKID`/`BAND`/`PARAMETER`) all
returning `+OK`, followed - after waiting through a full 120s interval - by a
genuine transmission:

```
TX: 0.00,0.00,0.00,0.00,3.00,6.00,9.00,758.00,0.00,32.20,0.00,0.00,1  (sent)
```

Sensors collecting, radio genuinely configured and transmitting, end to end
on the sensor node side. (Noted in passing: BME690 came up `NOT FOUND` on
this particular boot, consistent with its previously-documented floating
address-strap behavior - not a new issue, just something that means its CSV
fields read `0.00` whenever it happens.)

---

## Where things stand now

See [`STATUS.md`](STATUS.md) for the current snapshot - as of the end of this
log, the sensor node side (sensors + LoRa TX) is fully verified working, and
the only remaining blocker to a full end-to-end test is wiring in the
receiver's replacement RYLR998 module.

## File map

**`sensor_node`** (sensor node):
- `src/sensors/*` - per-sensor ISensor drivers (BME690, SEN0466, BMV080,
  CM1106, Calypso)
- `src/main.cpp` - production firmware (5 sensors, no LoRa)
- `src/chip_forest_v1.cpp` - ISensor/Sampler bring-up harness (5 sensors, no
  LoRa)
- `src/chip_forest_lora_tx.cpp` - the combined sensors→LoRa TX firmware
- `src/radio/RYLR998.{h,cpp}` - the LoRa driver (shared by
  `chip_forest_lora_tx.cpp`, `rylr998_bridge.cpp`, `rylr998_demo.cpp`,
  `rylr998_param_probe.cpp`)
- `src/rylr998_bridge.cpp` - stage-1 AT bridge/liveness test
- `src/rylr998_demo.cpp` - stage-2 TX/RX range-test demo
- `src/rylr998_param_probe.cpp` - queries a module's actual current AT
  parameters instead of guessing
- `src/radio/README.md` - RYLR998 wiring/band/parameter notes
- `STATUS.md` - current live-state snapshot (read this for "what's true now")

**`relay`** (ground-station receiver, sibling project):
- `src/main.cpp` - WiFi AP + LoRa RX + dashboard, the only file that wires
  the other three modules together
- `src/radio/RYLR998.{h,cpp}` - the LoRa driver (own copy, plus the debug
  Print* addition)
- `src/telemetry/SensorSnapshot.h`, `TelemetryParser.{h,cpp}` - payload
  schema and parsing, decoupled from radio/web
- `src/web/DashboardPage.h` - the inline dashboard page
- `src/radio_bridge_test.cpp` - isolated AT test used to diagnose the burned
  module
- `include/Config.h` - link parameters, WiFi AP credentials, staleness timeout
- `README.md` - wiring/WiFi/dashboard notes
