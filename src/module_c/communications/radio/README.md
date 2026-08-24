# RYLR998 LoRa bring-up

Self-contained radio module for the CHIP FOREST node. No sensor code depends on
this, and this doesn't depend on any sensor code - it drops into the existing
node scheduler later without touching `sensors/`, `Sampler`, or `PowerManager`.

## Region

**EU868.** Confirmed with the project owner - do not retarget to another band
(e.g. US915) without re-confirming legal ISM limits for wherever this actually
deploys. Getting this wrong is both non-functional (radios on different bands
can't hear each other) and potentially not legal to transmit.

`AT+BAND=868000000` (868.0 MHz) is the real deployment band - used by
`chip_forest_lora_tx.cpp` and by Module B (`src/relay/` in this same project,
originally the sibling `../../../relay/` project). `rylr998_demo.cpp` uses
`868500000` instead, but only for its own self-contained two-board range test;
it's not meant to interoperate with the receiver.

## Wiring

Same for every stage (bridge sketch, driver, demo):

| Module pin | ESP32-S3 |
|---|---|
| VDD | 3V3 |
| GND | GND |
| TXD | GPIO4 (ESP RX) |
| RXD | GPIO5 (ESP TX) |
| NRST | floating (not wired yet) |

**Net-name warning:** this project's schematic names the LoRa UART nets from
the MCU's point of view (`UART1_TX` = the ESP's transmit pin) - the opposite
convention used for the CM1106/Calypso nets on the same board. GPIO5 (ESP TX)
crosses to module RXD, GPIO4 (ESP RX) crosses to module TXD. That's correct as
a crossover; it isn't a bug and doesn't need to match the other sensors' wiring
convention.

Default module UART is 115200 8N1. If `AT` gets no reply at all, wrong baud is
the first suspect, not a code bug - use `rylr998-bridge` to check by hand.

TX bursts draw ~120mA. If the module resets mid-send, that's a brownout from
insufficient bulk decoupling on the supply rail, not a firmware problem - add
capacitance, don't add retry logic to paper over it.

## Environments

- **`rylr998-bridge`** (`src/rylr998_bridge.cpp`) - stage 1. Sends `AT` at boot
  and prints the reply, then becomes a transparent USB<->UART1 bridge so any AT
  command can be typed by hand. Use this first on new hardware, and again any
  time something upstream looks wrong, before assuming the driver is buggy.
- **`rylr998-demo`** (`src/rylr998_demo.cpp` + `src/radio/RYLR998.*`) - stage 2.
  The driver class plus a two-role range test.
- **`rylr998-param-probe`** (`src/rylr998_param_probe.cpp`) - diagnostic. Queries
  the module's current `AT+PARAMETER?`/`AT+BAND?`/`AT+ADDRESS?`/`AT+NETWORKID?`/
  `AT+VER?` instead of guessing a new combination blind. Use this whenever
  `AT+PARAMETER=...` comes back `+ERR=...` from `begin()` - see "Changing band
  or RF parameters" below for what that error actually meant here.
- **`chip-forest-lora-tx`** (`src/chip_forest_lora_tx.cpp`) - all five sensors
  read each cycle, encoded as CSV, sent to the ground-station receiver
  (Module B, `src/relay/` in this same project) over this driver.

```
pio run -e rylr998-bridge -t upload -t monitor       # stage 1: talk to it by hand
pio run -e rylr998-demo -t upload -t monitor         # stage 2: TX/RX range test
pio run -e rylr998-param-probe -t upload -t monitor  # diagnostic: query current params
pio run -e chip-forest-lora-tx -t upload -t monitor  # sensors -> LoRa -> receiver
```

## Driver shape (`radio/RYLR998.h`)

```cpp
bool begin(uint16_t addr, uint16_t networkId, uint32_t bandHz, const RYLR998Params &params,
           Print *debug = nullptr);
bool send(uint16_t destAddr, const char *data, uint8_t len);
bool poll(LoRaMessage &outMsg);
```

- `begin()`/`send()` are blocking, bounded by a timeout waiting on `+OK`/`+ERR`
  - the module doesn't offer anything faster to init or hand off a send with.
  Pass `&Serial` as `debug` to `begin()` to print every AT command and its
  reply as it happens - this is how the `+ERR=18` below was actually caught,
  instead of just seeing a single opaque "init failed."
- `poll()` never blocks. It only drains bytes already sitting in the UART's RX
  buffer and returns `true` the instant a complete `+RCV=` line parses cleanly.
  Safe to call every `loop()` iteration.
- `LoRaMessage` carries `senderAddr`, `payload`/`length`, `rssi`, `snr`. Payload
  is capped at 240 bytes - the commonly documented RYLR998 `AT+SEND` limit, not
  re-derived from a vendored datasheet (none exists in this repo). Treat that
  cap as a starting assumption; confirm it against this module's actual
  firmware/datasheet before relying on it near the limit.

## Changing band or RF parameters

Live in `rylr998_demo.cpp` and `chip_forest_lora_tx.cpp` (and would be
caller-supplied args anywhere else the driver is used):

```cpp
#define LORA_BAND_HZ 868000000UL  // EU868 (chip_forest_lora_tx.cpp; rylr998_demo.cpp uses 868500000
                                   // for its own internal two-board range test - the two are not
                                   // meant to talk to each other)
RYLR998Params params = {9, 7, 1, 12};  // spreadingFactor, bandwidth, codingRate, preamble
```

**These are NOT the generic "commonly documented" RYLR998 defaults** (`7,7,1,4`)
this project originally assumed. That combination came back `+ERR=18` from the
actual module - three prior commands (`AT`, `AT+ADDRESS`, `AT+NETWORKID`,
`AT+BAND`) all succeeded first, which ruled out wiring/baud and pointed
specifically at the parameter combination itself. Rather than guess a second
time, `rylr998-param-probe` queried the module directly via `AT+PARAMETER?`
and got back `9,7,1,12` - its actual current/accepted values - which is what
both this repo and the receiver project now use. If you ever retune these,
change both ends together and re-verify with that same probe; don't assume a
new guessed combination will be accepted.

- **Higher spreading factor (SF)** = more range, more airtime per byte sent.
- **Lower bandwidth (BW)** = more range, more airtime per byte sent.
- **Bandwidth is a module-defined index, not a raw kHz number** - don't assume
  "7" means "7 kHz" or "7 MHz"; confirm the index-to-kHz mapping against this
  module's actual AT command reference before changing it.
- **Coding rate (CR)** trades error-correction overhead against airtime.
- **Preamble** adds fixed airtime per packet regardless of payload size (12
  symbols' worth here, not the smaller "4" originally assumed).

**Duty cycle is a legal constraint, not a tuning knob.** EU863-870 SRD
regulations cap the duty cycle on this band - as low as 1% in the most commonly
used 868.0-868.6MHz sub-band. At SF9/BW125kHz/CR4:5/preamble12:
- `chip_forest_lora_tx.cpp`'s ~80-byte sensor payload: ~468ms airtime/packet ->
  legal floor ~47s between packets -> `TX_INTERVAL_MS` set to 120000 (2.5x margin).
- `rylr998_demo.cpp`'s ~10-byte counter payload: ~160ms airtime/packet -> legal
  floor ~16s between packets -> `SEND_INTERVAL_MS` set to 30000 (~2x margin).

If SF/BW/CR/preamble or payload size change, on-air time per packet changes
too - recompute (Semtech's public LoRa airtime formula; the math is shown in
each file's header comment) and re-check the duty-cycle budget before
shrinking either interval. This isn't optional or "flag and move on" -
transmitting outside the legal duty cycle is a compliance violation, not just
a firmware bug.

## What a passing range test looks like

Flash one board with `ROLE_TX` defined in `rylr998_demo.cpp`, the other with
`ROLE_RX` (comment/uncomment the `#define` pair at the top of the file, one
role per board). TX side:

```
=== RYLR998 range demo: TX role ===
Radio init OK
TX #0: sent
TX #1: sent
TX #2: sent
```

RX side:

```
=== RYLR998 range demo: RX role ===
Radio init OK
RX from 1: "0"  RSSI=-42  SNR=9
RX from 1: "1"  RSSI=-43  SNR=9
RX from 1: "2"  RSSI=-41  SNR=8
```

The counter in the payload should increment by exactly 1 each time with no
gaps at close range. Walk the RX board away from the TX board and watch RSSI
(more negative = weaker) and SNR (lower = noisier relative to floor) degrade
before packets start dropping - that degradation curve, and the range at which
drops start, is the actual range-test result to record.

If `Radio init FAILED` prints on either board: go back to `rylr998-bridge` on
that specific board and confirm `AT` still gets `+OK` before debugging the
driver - most failures at this stage are wiring/power, not parsing logic.
