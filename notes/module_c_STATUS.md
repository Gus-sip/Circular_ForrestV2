# CHIP FOREST - Session Status

## 2026-09-10 (later) - SLEEP CYCLE WORKING END TO END ON C2

Node C2 sleeps, wakes on schedule, reads, transmits, and Module B receives it.
`env:module-c-lora-tx`, `SLEEP_ENABLED 1`.

### Proof, from Module B's own serial

```
+RCV addr=1 len=72  rssi=-30 snr=13  "31.07,21.78,940.66,15705.52,4.00,9.00,12.00,681.00,..."
+RCV addr=1 len=73  rssi=-29 snr=13  "31.36,22.96,940.62,17802.50,4.00,9.00,12.00,1079.00,..."
+RCV addr=1 len=116 rssi=-29 snr=12  "B,2,31.5,21.8,...;31.6,21.8,...;"
```

The third is the two-reading BATCH at the 12-tick realignment. BMV080 is
reporting real particulates (pm1/pm2.5/pm10 = 4/9/12), not zeros. RSSI -29dBm.

### The cycle as it now runs

- 10s deep-sleep tick. Green LED wave on EVERY wake, idle ticks included.
- Every 3rd tick: rails on, sensors read ONE AT A TIME, store, sleep. Yellow LED.
- Every 4th tick: rails on, radio up, **sensor rails cut**, transmit, sleep.
- Any other tick: wave, then straight back to sleep with no rail powered at all.

3 and 4 do not divide evenly, so the two schedules drift and realign every 12
ticks - that tick both reads and transmits, sending 2 readings. Every reading is
still sent; only the grouping varies.

### Fixed this session

**1. LoRa transmit brownout.** The board took `POWERON` resets - true supply
collapse - immediately after `Setup complete.` on EVERY transmit tick, at
`radio.send()`. `quiesceAll()` stopped the sensors but left both rails powered, so
everything on them kept drawing idle current under the RYLR998's ~120mA burst.
Fix: `powerDownSensorRails()` cuts 3V3 and 5V, settles 400ms, then transmits. The
radio needs an EXCLUSIVE slot, not merely idle neighbours - the sequential-sensor
principle applied to the last load that was still sharing.

**2. BME690 first read always fails.** Read 1 `Timeout`, read 2 OK, read 3 OK, on
every boot. `getData()` treats any non-OK status from `bme69x_get_data()` as
failure, and Bosch WARNINGS (e.g. `NO_NEW_DATA`) are positive status codes.
Continuous sampling hid it by discarding one reading in a hundred; under sleep,
every wake is a fresh `begin()` plus exactly ONE read - the one that always fails -
so the BME690 sent nothing but zeros while reporting `OK` at init. Fix: a
discarded priming read in `begin()` plus one retry in `read()`.

**Generalises:** any sensor validated under continuous sampling must be
re-validated under sleep. "First read after init" goes from a rare case to the
ONLY case.

**3. Status LED is NEO_RGB, not NEO_GRB.** `neopixelWrite()` hardcodes the
WS2812B-standard GRB, so every colour was wrong. This was NOT a channel swap -
single-channel probing gave results no permutation can explain (one channel
producing teal) and two runs contradicted each other, because the error is in the
FRAME: neopixelWrite puts the green byte first and this part reads the first byte
as red. Settled by sweeping all six orders with `Adafruit_NeoPixel` and asking each
for pure green. Do not revert this to `neopixelWrite`.

**4. LED brightness was a power bug.** 120 and 120/80, commented as "a few percent
of full scale" - actually nearly half, on two channels, held through the whole
working tick. Unnoticed because the wrong colour order meant it never lit and drew
almost nothing. Now 30 and 30/18, and blanked across the radio burst.

**5. Module B logger dropped batches.** It required exactly 13 fields and skipped
anything else, so `B,<n>,...` payloads - about one transmit in three - vanished.
Now parses both, and uses each batch record's age-in-ticks to BACKDATE its
timestamp, so a batched reading lands at the time it was taken rather than the time
it was sent. Same 17 columns.

### Also removed

The bisect stop and the dead soft-start PWM ramp (written for the inrush theory the
bisect disproved). `gpio_hold_dis` re-enabled - sleep REQUIRES it, since the gates
are latched through sleep and the rails never come back without it.

### Open

- **Module B is not registered on NB-IoT.** Stuck at `+CEREG: 1,2` (searching),
  cycling `AT+CFUN=0/1` with `+CPIN: NOT READY`. LoRa C->B works; nothing reaches
  ThingsBoard. This is the next thing to chase.
- **Opening COM10 resets Module B** (~35s re-attach), so USB monitoring perturbs
  it. Non-invasive alternative: its WiFi AP `CHIP-FOREST-RX` / `chipforest1`, history
  at `http://192.168.4.1/history`.
- SEN0466 still excluded from the sleeping cycle - its 210s settle would dominate
  every wake. Needs a multi-phase wake.
- Fire thresholds are still invented placeholders; the `alarma`/`prealarma` tabs
  were never transcribed.
- LoRa TX power still 14 dBm, reduced during debugging. Raise to 22 for range work.
- `LED_WAVE_MS 500` and `SLEEP_USB_DRAIN_MS 400` are bench instrumentation - both
  are pure waste in the field.

### Reflashing note

With sleep enabled the port exists only ~1s per idle tick, so uploads fail with
"port is busy or doesn't exist". Retry in a loop; it catches the next wake. Or
power-cycle and flash inside the 30s cold-boot window.

## 2026-09-10 - RESETS FIXED: run the sensors one at a time

### The fix

Sensors must be read **sequentially, never concurrently**. That was the cause of
the resets that blocked this project for days.

| firmware | resets | reads | transmits |
|---|---|---|---|
| all sensors active together | 31 / 140s | 16 | 2 |
| BMV080 disabled | 1 / 140s | 62 | 9 |
| **sequential slots, everything on** | **2 / 150s** | **36** | **8** |

The two resets in the last run were the flash and reconnect.

**The bisect that found it.** `env:module-c-rail-gate` holds all three rails on
for 80+ seconds with zero resets - because it powers the sensors but never
ACTIVATES them. Disabling the BMV080 alone (~68mA, the largest load by an order
of magnitude) took the full firmware from 31 resets to 1. So the instability
tracked concurrent sensor LOAD, not the rails being on.

That is why staggering the rail bring-up and soft-starting the gate had both
failed: they addressed inrush, and the problem was steady-state overlap.
`power_budget.md` had specified the answer all along - "switched rails so the
68mA BMV080 and 5mA/210s SEN0466 only draw during their windows".

### New firmware: env:module-c-simple

`chip_forest_simple.cpp`, rebuilt from the rail sketch that works rather than cut
down from the one that did not. Each sensor gets a slot: switched on, read,
switched off before the next starts. The BMV080's laser fires only inside its own
slot. The transmit gets a slot with everything quiesced first, so the radio's
~120mA burst never lands on another load.

Deliberately absent: deep sleep, RTC_NOINIT state, NVS boot guard, safe mode,
soft-start, tick counters, batch store, alarm state machine.

Kept, each proven on hardware: rail polarities, CM1106 EN power cycle (single-shot
part), BMV080 address scan, GPIO21 status LED.

**Note:** the BMV080 SDK restore is per-env. After the first build of a new env:
`bosch_bmv080_sdk/restore.ps1 -EnvName <env>`.

### Node C1 sensor state

| Sensor | State |
|---|---|
| BMV080 | Working - real particulates, laser only in its slot |
| CM1106 | Working - 827 -> 1192 -> 1223 -> 1541, fresh on each EN cycle |
| SEN0466 | Initialises; needs its 210s settle before CO means anything |
| RYLR998 | OK, transmits accepted |
| **BME690** | **Absent from the I2C bus** - physical |
| **Calypso** | **Silent under every configuration** - physical |

**BME690** - the boot scan shows `ACK at 0x57` (BMV080) and `ACK at 0x74`
(SEN0466) only. Neither 0x76 nor 0x77 answers, so the bus is proven good by its
neighbours replying on the same wires at the same instant. It WAS working earlier
the same day with live values and nothing was touched in between, which is the
classic signature of a marginal solder joint opening. The board also took dozens
of hard power collapses that day while the reset fault was being chased.

**Calypso** - eliminated: both pins, six baud rates, six framings, a 20s boot wait
plus 3 minutes of continuous listening. Zero bytes throughout. Pads self-echo.
Power, ground and continuity confirmed by meter. Confirmed ULP PRO UART variant,
never app-configured. Both faults survived a hard reset.

### Diagnostics worth keeping

Two additions made the BME690 diagnosable in one line, and would have caught it
hours earlier:

- **`scanI2C()` at boot.** Separates "sensor gone from the bus" from "sensor
  present but not answering", and proves the bus by what else replies.
- **Per-sensor read status in the `[read]` line.** Previously a failed read was
  silently discarded by `if (r.ok())` and the last good value kept, so a dead
  sensor and a stable one printed identically. The status showed `never read` -
  the string's initial value - which meant the slot was being skipped entirely
  because `bmeReady` was false, not that reads were failing.

**These belong in the main firmware too, not just the minimal one.**

### Wrong turns, recorded so they are not repeated

The reset was misdiagnosed repeatedly before the bisect found it: as a shorted 5V
rail, as the RYLR998 VDD bypass wire, and as VBUS sag. **A surge, a short and
concurrent load all look identical from firmware.** Only disabling one sensor at
a time separated them.

Also: adding `esp_task_wdt_deinit()` broke a working build, and the RTC_NOINIT
"domain lost" messages were treated as evidence about power for hours when the
mechanism was something else entirely.

### Next

Node C2 for sleep-cycle development - its BME690 works, it has no Calypso, and it
transmitted to Module B cleanly. C1 needs a meter on two sensors before it can be
a complete node.

## 2026-09-09 - Sleep cycle causes the brownouts, not a short; LED found; Calypso silent

### THE BOARD IS SHIPPED WITH SLEEP DISABLED

`SLEEP_ENABLED 0` in `chip_forest_lora_tx.cpp`. In that mode the node stays awake,
samples continuously and transmits every `NOSLEEP_TX_GAP_MS` (15s) - the tick
counters cannot drive it without reboots, so they would sit at 1/3 forever. Set it
back to 1 to re-enable the tick cycle. **Do not forget this is off.**

### The brownouts are the sleep cycle, not a hardware short

This corrects yesterday's diagnosis. Node C1 was recorded as having a shorted 5V
rail on the strength of a gate sweep: every combination with GPIO11 HIGH browned
out, every one with it LOW survived. That reading was wrong.

With `SLEEP_ENABLED 0`, node C1 - same board, same rails, GPIO11 held HIGH
throughout - runs cleanly:

```
TX attempts:  7      (all "sent")
sensor reads: 23
resets:       1      (the reflash)
```

With the sleep cycle running, the same board: 11 resets and 3 true power losses in
four minutes, and **zero** transmits - it died at `Tick: 3/3`, the working tick,
every time.

So the fault is the sleep cycle's rail handling. Every wake latches all three rails
off, then releases and re-asserts them together, which is an inrush event on every
working tick. Steady-state the supply is fine, which is what the user's voltage
measurements found. A surge the supply cannot absorb and a short look identical
from firmware; they are not the same thing.

**Next step:** stagger the rail bring-up - 3V3, settle, 5V, settle, LoRa, settle -
instead of asserting GPIO10 and GPIO11 together. If that is not enough, test
keeping the rails powered through sleep, which sacrifices most of the saving but
would confirm inrush as the whole story.

### Node C2 will not run on the bench supply

Reproduced in four minutes what the overnight test showed in twenty hours: C2
transmits every ~76s on USB, and stops dead the moment USB is removed. The
overnight run captured **zero** packets; C2's RTC counters had been wiped, so it
had lost power entirely and its record of the night with it.

Bench supply: ~6V injected ahead of two **step-down** converters (5V and 3V3).
Initially it read 5.25V at 0.150A - the voltage sag was the supply in
constant-current mode against a 150mA limit, and at 5.25V in, the 5V buck has only
250mV of headroom, below dropout. Raising the limit did not fix it, and the board
still draws a flat 150mA with no pulsing - meaning it never reaches the sleep call
at all. Unresolved.

### LED found on GPIO21, and the trap that hid it

`PIN_STATUS_LED 21` in pins.h. Three wrong answers first, all of which compiled
cleanly and failed silently:

- `RGB_BUILTIN` is **97**, not a pin. The esp32s3 variant defines it as
  `SOC_GPIO_PIN_COUNT + PIN_NEOPIXEL`, a sentinel `digitalWrite()` special-cases.
  `neopixelWrite()` wants a real GPIO, so 97 addressed a pin that does not exist.
- The variant's `PIN_NEOPIXEL 48` is wrong for these modules. Some boards
  (Waveshare ESP32-S3-DEV-KIT-N8R8) use GPIO38; ours is 21.
- An `#ifdef RGB_BUILTIN` guard added "for safety" made a wrong-pin bug look like
  a missing-macro bug.

The indicator now has three states: **green flash** on every wake (including idle
ticks), **yellow** held for the whole of a working tick, **off** asleep. Each state
is driven both as WS2812 and as a plain level, since the pin is known but the type
is not.

### Idle ticks no longer power anything

The tick decision moved from `loop()` into `setup()`, before any rail is touched.
Previously every wake brought up all three rails, paid the BMV080's 5s preheat and
initialised the radio, then discovered it had nothing to do - two ticks in three
were pure waste, and the measured cadence was 76s against a configured 30s.

Also fixed: **the tick counters were never incremented at all.** They were
declared, reset in five places and compared against their thresholds, but nothing
advanced them, so the node would have slept forever without ever reading or
transmitting. It failed silently and looked like healthy low-power behaviour.

### Confirmed working

- **CM1106 self-recovery.** Detected 20 consecutive frozen values, power-cycled EN,
  and the reading genuinely moved: 622 -> 500.
- **Batch transmit.** `TX [0..3/4] 228 bytes: B,4,...` - four readings in one
  packet, inside the 240-byte cap.

### Calypso: silent, and it is physical

Twelve combinations - GPIO8 and GPIO9 x six baud rates - all **0 bytes**. Not bad
checksums, not garbled framing: nothing arrives at all. Our own pads self-echo, so
the UART is good, and the user measured 5V present at the sensor itself.

It worked on the old setup - the CSV from 2026-09-03 has `windAngle=336.00,
windSpeed=0.50, windValid=1` - so the sensor was good then. That points at the
wiring on this PCB rather than a dead part.

Remaining checks are continuity: sensor TX pad to GPIO8, and sensor GND to board
GND. A missing shared ground has faked a dead peripheral twice on this project.

### Hardware inventory

| | MAC | Notes |
|---|---|---|
| node C1 | `AC:27:6E:CB:8D:A8` | Calypso soldered. Runs with sleep off; brownouts with it on |
| node C2 | `28:84:85:6F:8F:CC` | Works on USB, will not run on the bench supply |
| node C3 | `28:84:85:6F:92:A4` | New this session, blank flash on arrival |
| Module B | `28:84:85:6F:8F:DC` | Healthy. Now on its own supply, so not observable over USB |

### New tools

- `env:module-c-rail-sweep` - walks all 8 GPIO10/11/13 combinations, recording
  verdicts to **NVS** rather than RTC, because the brownout being measured wipes
  the RTC domain.
- `env:module-c-led-hunt` - drives each PCB-free GPIO HIGH/LOW then as WS2812.
  Restricted to pins the design does not use, so it is safe on a populated board.
- `env:module-c-led-test` - reports what the build defines for the LED, then sweeps
  candidate pins.
- `logs/overnight_summary.py` - digests the Module B CSV into ~15 lines: packet
  count, span, mean gap, gaps over 2 minutes with timestamps, and min/max/last per
  field with a FLAT marker on anything that never changed.

## 2026-09-08 - Rail gate polarity found; CM1106 was never faulty

Two long-standing mysteries resolved, and they turned out to be one bug and one
misreading of a datasheet behaviour.

### GPIO11 is ACTIVE-HIGH and gates the 5V rail

Swept all four GPIO10/11 combinations while reading the CM1106 - the only
connected 5V device - in each:

```
GPIO10=LOW  GPIO11=LOW   -> silent
GPIO10=LOW  GPIO11=HIGH  -> answers, CO2=793     <- 5V rail up
GPIO10=HIGH GPIO11=LOW   -> silent
GPIO10=HIGH GPIO11=HIGH  -> answers              <- 5V rail up
```

The firmware drove BOTH gates LOW from a single shared `PIN_PCB_EN_ACTIVE`, so
the **5V rail was switched off every time it believed it was turning the rails
on**. Everything on 5V - the CM1106 and the Calypso - was unpowered.

The floating-ground fault masked this for days: with no solid reference, driving
GPIO11 "LOW" was not a true low at the transistor, so the 5V rail stayed up by
accident. Repairing the ground made LOW a real low and switched 5V properly off,
which read as the CM1106 dying overnight. **A fix revealing a second, older bug
is the pattern to expect on this board.**

`pins.h` now carries per-gate macros: `PIN_PCB_EN_A_ACTIVE LOW` (GPIO10, 3V3)
and `PIN_PCB_EN_B_ACTIVE HIGH` (GPIO11, 5V). Note especially the sleep path -
computing one shared `offLevel` for both gates would switch one rail off and the
other ON, powering a rail for the entire sleep.

### The CM1106 is single-shot, not broken

`0x11 0x01 0x01` only reads back the LAST measurement; it never triggers a new
one. The part measures on power-up, so the value is fixed for a whole boot and
fresh after a power cycle - exactly the "frozen value" symptom chased for weeks.
Proven by cycling EN: `before cycle: CO2=793` -> `after cycle: CO2=671`.

`power_budget.md` had said so all along ("BASE_ESCALADA - single-shot"), but only
one command had ever been sent to this part. Sweeping `0x00-0x2F` with computed
checksums mapped the real command set: `0x01` the 8-byte CO2 frame, `0x04` a
14-byte extended frame containing the CO2, `0x0D` an 11-byte frame carrying
`0x1388` = 5000 (the range ceiling), `0x06`/`0x0F` short status frames, and
`06 01 02 F7` as the NAK for everything else.

To read it correctly: power-cycle EN, wait the warm-up, then read. Reading
without cycling returns a stale value that looks perfectly healthy - checksum
passes, the counter byte advances, nothing signals staleness.

### New this session

- `env:module-c-rail-gate` (`rail_gate_set.cpp`) - parks GPIO10/11 at chosen
  levels and holds, so the rails can be metered. No sensors, no sleep.
- `env:module-c-cm1106-probe` (`cm1106_probe.cpp`) - gate sweep, line-state and
  pad self-check, passive listen, orientation x baud sweep, command discovery,
  EN power-cycle test.
- `notes/module_c_REFERENCE.md` - consolidated reference for the whole node.

### Still open

- **The transmit brownout.** Unchanged: the board loses power inside
  `radio.send()`, confirmed via `RTC_NOINIT_ATTR`. Next step is still to remove
  the RYLR998 VDD bypass wire and let the radio run on its GPIO13-gated rail.
- **The sleep cycle is written and building but not yet flashed.**
- Fire thresholds are still placeholders; the `alarma`/`prealarma` spreadsheet
  tabs were never transcribed.

## 2026-09-04 - Fabbed Module C PCB bring-up: two hardware faults found, one open

First session on the **real Module C PCB** (not the bench harness). New board:
COM13, MAC `ac:27:6e:cb:8d:a8`, arrived with blank flash. The old COM5
error-31 USB fault does not apply to this unit.

### Two hardware faults found and fixed

- **Floating ground.** The single biggest cause. With no shared reference the
  RYLR998 was silent to a bare `AT` at every baud and in both crossover
  orientations, and the BMV080 initialised only intermittently. It also made
  every meter reading misleading: continuity passed (probing within a net) and
  voltages looked fine (measured against the same floating reference). A GPIO
  line-probe reported BOTH UART pins as "driven high", which is impossible for
  a pin wired to a module input - that impossibility was the tell, but it was
  misread as external pull-ups at the time. Connecting a real ground fixed the
  radio and the BMV080 together.
- **Unpowered LoRa rail gate.** `GPIO13` is a P-FET gate on the RYLR998 supply,
  LOW = on - the same pin and convention as Module B's `LORA_EN_PIN` (Q4 gate,
  pin-scan 2026-08-19). Module C's firmware never drove it. Now `PIN_LORA_EN`
  in `pins.h`, asserted at the top of setup().

Also confirmed: `GPIO10`/`GPIO11` are high-side transistor gates for the 5V and
3V3 sensor rails (LOW = on). They must be asserted before `Wire.begin()` and any
sensor `begin()`, or every sensor reads as dead hardware.

### Sensor state

| Sensor | State |
|---|---|
| BME690 | Working - T/H/P/gas all live |
| SEN0466 CO | Working |
| BMV080 PM | Working, now duty-cycled (laser on ~3.6s per sample, not continuous) |
| CM1106 CO2 | **Device alive, measurement field frozen** - see below |
| RYLR998 | Initialises and accepts config; **has never completed a transmit** |
| Calypso wind | Not connected - untested |

**CM1106 diagnosis (from raw wire bytes, not the parsed ppm):** the frames are
NOT byte-identical - `resp[6]` is a counter that increments every read with the
checksum tracking it, while `resp[3..4]` (the CO2 value) stays pegged for a whole
boot and changes only across power cycles. So the device is not hung and the
driver is not serving cache; only the measurement is dead. A freeze detector
therefore MUST compare the value bytes - whole-frame equality can never fire
against an incrementing counter. `Cm1106Sensor::frozenValueRun()` does this, and
past `CM1106_FREEZE_LIMIT` the firmware power-cycles EN (GPIO3).

### Open: the board browns out during LoRa transmit

Pinpointed to inside `radio.send()`:

```
[tx] payload 73 bytes, calling radio.send ...
ESP-ROM:esp32s3-20210327        <- never reaches "radio.send returned"
```

A boot counter in `RTC_NOINIT_ATTR` re-initialises on every one of these, so the
**RTC power domain is being lost** - a genuine power collapse, not a soft reset
and not a firmware panic (a panic keeps the RTC domain and prints a backtrace).
Note `RTC_DATA_ATTR` is useless for this test: `.rtc.data` is reloaded from the
image on every reset, so a counter there reads 1 forever.

Dropping TX power 22 -> 14 dBm (`AT+CRFOP`, accepted by the module) changed
nothing, so the supply is not marginally short - it collapses on any transmit.

**Leading suspect: the VDD bypass wire.** The RYLR998's VDD was bridged straight
to 3V3 to rule out the GPIO13 gate, back when the real fault was the floating
ground. That leaves a ~120mA pulsed load hanging directly on the ESP32's own 3V3
node, bypassing the PCB's gating and local decoupling. **Next step: remove the
bypass and let the radio run on its designed GPIO13-gated rail.** If it still
browns out, it is bulk capacitance at the module's VDD - as
`rylr998_bridge.cpp`'s header has warned from the start ("add capacitance, don't
add retry/backoff to paper over it").

### Firmware changes this session

- **Transmit is now readiness-gated, not timer-driven.** A packet goes out when
  every *expected* sensor has a fresh reading since the last TX. "Expected"
  means enabled AND present - a sensor that failed to init is never waited on,
  or the gate would never open, and the Calypso joins only once it has ever
  produced a reading. Guarded by `LORA_TX_MIN_GAP_MS` (15s floor - sampling runs
  every 2s, so without it the radio would transmit almost continuously) and by a
  deadline (`CFG,INTERVAL`, default 5 min) that sends anyway and names the stale
  sensors. Confirmed working: `[tx] all sensors fresh after 18s - sending`.
- **BMV080 duty-cycled.** It draws ~68mA - the largest load in the system by an
  order of magnitude, and `power_budget.md` allows it 20s per 30 min. The
  firmware had it running continuously (~90x its allowance). Now started only
  around a sample and stopped as soon as a frame lands.
- Payload length clamp in `transmitSnapshot()` - `snprintf`'s return was passed
  to `send()` even when truncated, which would read past the 96-byte buffer.
- New `env:rylr998-bench` + `rylr998_bench_probe.cpp`: interrogates an RYLR998
  on a breadboard, and **self-checks its own pads before judging the module**
  (a verdict from an unverified rig is worthless). This is what proved the
  module healthy while the PCB was at fault.
- `RYLR998::setTxPower()` added to the shared driver - deliberately not folded
  into `begin()`, since Module B shares the file and has no reason to lose range.

### Still diagnostic, not fixes

- **Init order is swapped** (CM1106 before BMV080) in `chip_forest_lora_tx.cpp`,
  labelled as a probe. It was the test for cumulative-load-vs-inrush; with the
  BMV080 now duty-cycled it should be safe to revert, but that has not been
  retested.
- `[step]` markers and the reset-reason print in setup(), and the `[cm1106 raw]`
  per-cycle dump. Noisy in normal operation; kept because they are what
  localised each fault.

### Known-bad measurement techniques (recorded so they are not repeated)

- A serial capture loop must catch `TimeoutException` *inside* its read loop. An
  earlier version let it escape, reopening the port constantly, and reported
  "127 USB re-enumerations" that were entirely self-inflicted.
- `RTC_DATA_ATTR` does not survive a chip reset. Use `RTC_NOINIT_ATTR` with a
  magic word.

## 2026-08-28 — Fast cadence live, Calypso wind sensor confirmed dead on the wire

- **End-to-end pipeline running:** Module C (SF7, ~18s cycle) → LoRa → Module B
  → NB-IoT/MQTT → ThingsBoard `NodoC-1`, with per-reading timestamps. PC-side
  serial logger resumed (`logs/log_module_b_lora.py COM10` →
  `logs/module_b_lora_log.csv`).
- **Sensor status in the live telemetry:** BME690 temp/hum/pres/gas — good.
  CM1106 CO2 — reading 1400-ish ppm now (briefly spiked to its 5000 ceiling
  earlier in the session, recovered on its own). SEN0466 — coTemp reads, CO
  stays 0. BMV080 PM — mostly 0, occasional 1-2. Calypso wind — **0/0/false,
  see below.**
- **UPDATE 2026-09-01: Calypso is FIXED.** It needed **5V VCC** (silent on
  3.3V) and its TX/green wire is on **GPIO8, not GPIO9**. `pins.h` swapped to
  `PIN_CALYPSO_RX 8`. Now streams valid `$IIMWV` - `Wind: 65.0 deg 0.00
  valid(A)`. The exhaustive-ruled-out notes below stand as the diagnosis path
  but the conclusion ("physical, needs multimeter") resolved to power+pin.
- **Calypso wind sensor: not a software problem.** Ruled out exhaustively:
  a standalone diagnostic that does nothing but listen on UART1 (swept
  GPIO8↔9, baud 4800/9600/19200/38400/115200, RX pull-ups on, TX poked for
  poll-mode) saw **0 bytes on every combination**. The ESP UART is fine; the
  wire is electrically silent. `chip_forest_lora_tx.cpp` now polls the
  Calypso up to `CALYPSO_READ_WINDOW_MS` (1500ms) after wake and logs
  `Wind: no valid NMEA this cycle (rx bytes=N)` - N has been 0 every cycle.
  Root cause is physical (no common ground with the 5V supply / dead sensor /
  broken green-wire contact / UART-vs-I2C mode strap on the CMI1032) - needs
  a multimeter on the sensor's TX line and GND continuity. NOTE the repo has
  **conflicting Calypso pin maps**: `pins.h` (and the LoRa-TX firmware) use
  GPIO9/8; `sensor/main.cpp` and `ulp_pro_uart_test.cpp` use GPIO5/6. User
  confirmed the physical wiring is green→GPIO9, yellow→GPIO8 (matches
  pins.h). `ulp_pro_uart_test.cpp` was rewritten as the diagnostic sweep and
  left pointing at GPIO9/8.
- **Light sleep removed from Module C** - it broke the ESP32-S3 USB-Serial/
  JTAG (port wedged every cycle) and starved the Calypso UART. Now a plain
  `delay()`. See `chip_forest_lora_tx.cpp` header + git.


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
