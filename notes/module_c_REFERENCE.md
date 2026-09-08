# Module C — Field Reference

Everything established about the **fabbed Module C sensor node**: pin map, sensor
state, fault diagnoses, firmware architecture, and the questions still open.
Written from measurements taken on the board, not from the schematic.

| | |
|---|---|
| Board | ESP32-S3 rev v0.2, 4 MB XMC flash, 2 MB PSRAM |
| MAC | `ac:27:6e:cb:8d:a8` |
| Port | COM13 |
| Repo HEAD at writing | `05ea5cf` |
| Last verified | 2026-09-08 |

> Companion documents: `module_c_STATUS.md` (dated session log),
> `power_budget.md` (energy model), `module_c_README.md` (original bring-up notes).

---

## 1. Current state

| Subsystem | Bus / rail | State | Evidence |
|---|---|---|---|
| BME690 | I2C `0x77`, 3V3 | **Working** | T/H/P/gas all live and tracking |
| BMV080 PM | I2C `0x57`, 3V3 | **Working** | Sensor ID `D0ML6909194C`, frames each second |
| SEN0466 CO | I2C `0x74`, 5V | Excluded | Works, but its 210 s settle keeps it out of the sleep cycle |
| CM1106 CO2 | UART2, 5V | **Working** | Single-shot; needs an EN power cycle per reading |
| Calypso wind | UART1, 5V | Not fitted | Never connected to this PCB |
| RYLR998 LoRa | UART0, gated | Init only | `Radio init OK`, but browns out inside `send()` |
| Rail gates | GPIO10/11/13 | **Working** | All three switch as expected |

> **Read this before trusting the link.** `Radio init OK` means the module accepted
> its configuration. It does **not** mean a packet has reached Module B — the board
> has never survived long enough to finish a transmit. Treat the Module C → Module B
> path as unverified.

---

## 2. Pin map and rails

Every value below was confirmed on hardware. `pins.h` still carries a header
describing a pre-fab bench harness — ignore that framing; the entries are current.

| Signal | GPIO | Peripheral | Notes |
|---|---|---|---|
| I2C SDA | 1 | shared bus | BME690, SEN0466, BMV080 |
| I2C SCL | 2 | shared bus | Bus proven good - 3 devices ACK |
| CM1106 EN | 3 | - | Active high, 1k series resistor on board |
| LoRa RX | 4 | UART0 | ESP RX <- module TXD |
| LoRa TX | 5 | UART0 | ESP TX -> module RXD |
| CM1106 TX | 6 | UART2 | ESP TX -> sensor RX |
| CM1106 RX | 7 | UART2 | ESP RX <- sensor TX |
| Calypso RX | 8 | UART1 | Sensor TX (green), 38400 baud |
| Calypso TX | 9 | UART1 | Unused - sensor streams unprompted |
| Rail gate A | 10 | - | 3V3 rail, **LOW = on** |
| Rail gate B | 11 | - | 5V rail, **HIGH = on** (opposite polarity) |
| Spare | 12 | - | Reserved for a CM1106 RDY line |
| LoRa gate | 13 | - | P-FET on radio supply, **LOW = on** |

### Three independent power gates

GPIO10 gates the 3.3 V sensor rail, GPIO11 gates 5 V, and GPIO13 gates the radio.
**The two sensor gates have OPPOSITE polarity:** GPIO10 is active-low, GPIO11 is
active-**high**. GPIO13 is active-low, matching Module B's `LORA_EN_PIN`.

Never compute a single shared "off" level for both sensor gates - the inverse of
one gate's active level is the other gate's ON level, so a shared value powers a
rail for an entire sleep.

**Assert all three at the very top of `setup()`**, before `Wire.begin()` and every
sensor `begin()`. With them deasserted the sensors have no power at all, so an I2C
scan or a UART probe finds nothing and reads exactly like dead hardware - which is
precisely how the radio fault was misdiagnosed for hours.

> **Resolved 2026-09-08.** Sweeping all four GPIO10/11 combinations while reading
> the CM1106 (the only connected 5 V device) settled both the mapping and the
> polarity:
>
> ```
> GPIO10=LOW  GPIO11=LOW   -> silent
> GPIO10=LOW  GPIO11=HIGH  -> answers, CO2=793   <- 5V rail up
> GPIO10=HIGH GPIO11=LOW   -> silent
> GPIO10=HIGH GPIO11=HIGH  -> answers            <- 5V rail up
> ```
>
> The firmware had driven both gates LOW, so the **5 V rail was switched off every
> time it believed it was turning the rails on**. The floating ground had masked
> this: without a solid reference, GPIO11 "LOW" was not a true low at the
> transistor, so 5 V stayed up by accident. Repairing the ground made LOW real and
> switched 5 V properly off - which read as the CM1106 dying overnight.

### Net-naming inconsistency

The schematic names LoRa nets from the **MCU's** point of view but CM1106 and
Calypso nets from the **sensor's**. That inconsistency has already produced one
wrong pin map (the Calypso's RX/TX were swapped for weeks). When a new peripheral
is silent, sweep both orientations rather than reasoning about which convention
applies.

---

## 3. LoRa link parameters

| Parameter | Module C sets | Note |
|---|---|---|
| Band | `868000000` | EU868, Spain. Modules ship on 915 MHz (US) |
| Network ID | `5` | Factory default is 18 |
| Address | `1` | Module B is `2` |
| Parameters | `7,7,1,12` | SF7 / BW / CR / preamble |
| TX power | `14 dBm` | Lowered from 22 while chasing the brownout |
| UART | `115200` 8N1 | Matches module default |
| Payload cap | 240 bytes | Hard limit per packet |

The module arrives on the **US 915 MHz band**. `radio.begin()` rewrites band,
address, network ID and parameters at init, so a healthy boot corrects it - but
anything reaching the air before that init runs is off-band for Spain.

---

## 4. Open faults

### 4.1 Board loses power inside `radio.send()` — BLOCKING

```
[tx] payload 73 bytes, calling radio.send ...
ESP-ROM:esp32s3-20210327     <- never reaches "radio.send returned"
```

Three transmits attempted, three resets. Everything before the transmit completes
normally.

**Evidence.** A boot counter in `RTC_NOINIT_ATTR` re-initialises on every one of
these resets, meaning the RTC power domain itself drops out. That rules out a soft
reset and a firmware panic - a panic retains the RTC domain and prints a backtrace.
Cutting TX power from 22 to 14 dBm was accepted by the module and changed nothing,
so the supply is not marginally short; it collapses on any transmit at all.

**Diagnosis.** A genuine brownout. Prime suspect is the **RYLR998 VDD bypass wire** -
added to rule out the GPIO13 gate during the floating-ground period, it hangs a
~120 mA pulsed load directly on the ESP32's own 3.3 V node, skipping the PCB's
gating and local decoupling.

**Next.** Remove the bypass and let the radio run on its GPIO13-gated rail (the
firmware now drives that gate). If it still browns out, the fix is bulk capacitance
at the module's VDD. Do **not** add retry or backoff - that hides it.

### 4.2 CM1106 — RESOLVED, was never faulty

Two separate misreadings stacked on top of each other here, and both are worth
remembering.

**The value froze because the sensor is SINGLE-SHOT.** `0x11 0x01 0x01` only reads
back the last measurement; it never triggers a new one. The part measures on
power-up, so the value is fixed for a whole boot and fresh after a power cycle -
precisely what was seen and misread as a fault. Proven by cycling EN:

```
before cycle: CO2=793
after cycle:  CO2=671    -> power-up triggers a measurement
```

`power_budget.md` had recorded this all along ("BASE_ESCALADA - single-shot"), but
only one command had ever been sent to this part.

**It then went silent on 09-07 because its 5 V rail was switched off** - see the
gate-polarity note in section 2. Nothing was wrong with the sensor at any point.

**To read it correctly:** power-cycle EN (GPIO3), wait the warm-up, then read.
`Cm1106Sensor::powerCycle()` does this. Reading without cycling returns a stale
value that looks perfectly healthy - checksum passes, counter advances, nothing
signals staleness. `frozenValueRun()` compares the value bytes (never whole frames,
which the counter makes always differ) and is a staleness detector, not a fault
detector.

**Command set**, discovered by sweeping `0x00-0x2F` with computed checksums:

| Command | Response | Meaning |
|---|---|---|
| `0x01` | `16 05 01 <hi> <lo> 00 <ctr> <cs>` | CO2 read, 8 bytes |
| `0x04` | 14 bytes incl. the CO2 | Extended measurement frame |
| `0x0D` | 11 bytes carrying `0x1388` = 5000 | Range ceiling |
| `0x06`, `0x0F` | short frames | Status |
| everything else | `06 01 02 F7` | NAK |

---

## 5. Solved faults

### Floating ground — root cause of two apparently unrelated failures

The RYLR998 was silent to a bare `AT` at every baud and in both orientations, and
the BMV080 initialised only intermittently. Fixing the ground repaired both.

**Why it hid so well:** it made every meter reading misleading. Continuity passed,
because probing happens within a net; voltages looked correct, because they were
measured against the same floating reference. A GPIO probe reported *both* UART
pins as driven high - impossible for a pin wired to a module *input*. That
impossibility was the tell, and it was misread as external pull-ups at the time.

### LoRa rail gate never driven (GPIO13)

Module C's firmware never touched GPIO13, so the radio had no power. It appeared to
work only because of the hand-added VDD bypass wire. `PIN_LORA_EN` is now asserted
in `setup()` alongside the sensor rails - which matters when removing the bypass,
since without it the radio would go dead again and look like a fresh fault.

### BMV080 running continuously — ~90x its energy allowance

The laser draws ~68 mA, the largest load in the system by an order of magnitude,
and the budget allows it 20 s per 30 min. The firmware left it in continuous
measurement, and the board reset on every boot where it came up. Now duty-cycled:
started only around a sample, stopped as soon as a frame lands. Laser on-time fell
to ~3.6 s per reading and the setup-time reset disappeared.

### RYLR998 suspected dead — module exonerated, PCB was at fault

Moved to a breadboard ESP32-S3 on identical pins, so the PCB was the only variable
removed. It answered `+OK` immediately at 115200. The probe (`env:rylr998-bench`)
self-checks its own pads before judging the module - a verdict from an unverified
rig is worthless.

---

## 6. Firmware

### Build environments

| Environment | Purpose |
|---|---|
| `module-c-lora-tx` | Production firmware - sensors -> LoRa -> Module B |
| `module-c-bringup` | BMV080 raw-SDK probe with I2C scan and real status codes |
| `module-c-cm1106-probe` | CM1106 diagnosis: line state, orientation x baud sweep, command discovery |
| `rylr998-bench` | Interrogates a radio on a breadboard; self-checks its own pads first |
| `module-c-rylr998-bridge` | AT liveness, dual-orientation baud sweep, transparent bridge |
| `module-c-main` | **Do not flash** - puts I2C on GPIO10/11, which are now the rail gates |

### Downlink commands

Arriving from Module A via Module B, parsed as `CFG,<key>=<value>[,...]`. These land
only in the ~2 s listen window after each transmit, so a change takes effect after
the *next* packet, not immediately.

| Key | Effect |
|---|---|
| `SENSOR_READ` | Ticks between sensor reads |
| `LORA_TRANS` | Ticks between transmits |
| `ALARM=0` | Force-clears pre-alarm. Setting an alarm remotely is not supported |
| `INTERVAL` | Fallback transmit deadline, seconds |
| `BME690`, `SEN0466`, `BMV080`, `CM1106`, `CALYPSO` | Per-sensor enable/disable |

---

## 7. Sleep cycle

Written and building; **not yet flashed**. The ESP keeps its own 3.3 V rail and
wakes itself on a timer - the power MCU is not involved.

### One tick is 10 seconds

Both counters advance every wake and each resets on reaching *its own* threshold, so
the two schedules are independent:

```
sensor_read == 3   -> read fast sensors, store the reading
lora_trans  == 3   -> transmit everything stored, clear the store
neither due        -> no sensor or radio init at all, straight back to sleep
```

That last line is what makes a 10 s tick affordable - the BMV080's startup alone
would otherwise consume 5 s of every 10 s tick.

### The packet limit constrains the ratio, not one counter

Readings per packet is `ceil(LORA_TRANS / SENSOR_READ)`, so *lowering* `SENSOR_READ`
overflows a packet just as surely as raising `LORA_TRANS`. Both keys run the same
check. At ~70 bytes worst case per reading against a 240-byte cap, the limit is
**3 readings per packet**.

**Payload compatibility:** a single stored reading emits the exact legacy format, so
the default 3/3 configuration needs **no change on Module B**. The `B,<n>,...` batch
format appears only once `LORA_TRANS` exceeds `SENSOR_READ` - and that is when
Module B's parser must be updated. Each batched reading carries its age in ticks,
since Module C has no clock.

### Gates are latched through sleep

`gpio_hold_en()` on GPIO10/11/13 plus `gpio_deep_sleep_hold_en()`. Un-held, those
pads go Hi-Z and a floating P-FET gate can drift back to conducting - leaving the
rails powered for the whole sleep and wasting exactly what sleeping was meant to
save. **The saving comes from that gating, not from the MCU's ~10 uA.** The
corollary: nothing can drive those pins after a wake until the hold is released, so
`gpio_hold_dis()` must run before any `pinMode`.

### Cold-boot reflash window

Deep sleep drops the USB-Serial/JTAG. At a 10 s tick the port would appear for a
second or two at a time, impractical to catch. So after a power-on or reset - as
opposed to a timer wake - the node refuses to sleep for 30 s. Power-cycle the board
and you always get a window to flash in. This is the same trap that caused light
sleep to be removed from this firmware once before.

### Pre-alarm

On a trip the node stops sleeping entirely and reads continuously. It clears after
5 consecutive clean reads - more than one, so a dip mid-fire cannot end it - or
immediately on `CFG,ALARM=0`.

> **Thresholds are placeholders.** PM2.5 >= 50 ug/m3, temperature >= 50 C, gas
> resistance 50% below a learned clean-air baseline, CO >= 50 ppm. These are **not**
> from the project's `alarma` / `prealarma` spreadsheet tabs, which were never
> transcribed - only `parametros` reached the repo. Replace them before this means
> anything in the field.
>
> The CM1106 is deliberately excluded from fire detection: a trigger hung on a
> sensor whose value freezes would be blind to a real fire *and* liable to latch on
> a stale number. The gas baseline learns only while nothing looks wrong, or a slow
> fire would teach the node that smoke is normal.

---

## 8. Power budget

From the project spreadsheet. Autonomy assumes a 600 F supercap bank (3.8 V max /
2.5 V min, 464 mWh usable) - confirm these still hold if the design has moved to a
battery with two step-ups.

| Sensor | Rail | Active | Duration | Period |
|---|---|---|---|---|
| BMV080 PM | 3V3 | 68 mA | 20 s | 30 min |
| SEN0466 CO | 5V | 5 mA | 210 s | 30 min |
| BME690 VOC | 3V3 | 3.1 mA | 10.8 s | 20 min |
| BME690 T/H/P | 3V3 | 2.2 uA | 1 s | 5 min |
| CM1106 CO2 | 5V | 3 uA | 0.7 s | 30 min |
| Calypso wind | 5V | 0.15 mA | 1 s | 5 min |
| ESP32 + LoRa | 3V3 | 40 / 25 mA | 2 s / 0.1 s | per send |

Autonomy with no sun: **79.7 h** Normal (30 min sends), **27.5 h** Prealarma
(15 min), **13.4 h** Alarma (3 min).

The two step-ups share one source, so the rails are **coupled through the battery**:
a 5 V load spike pulls a magnified current through the source impedance, sagging the
input to the 3.3 V converter. A load on one rail can reset the MCU on the other.

---

## 9. Method and traps

### Techniques that resolved real faults here

- **Internal self-loopback.** Mux a UART's RX and TX onto the same pad; the ESP
  reads back what it drove. Proves the peripheral, pin mux and pad with no jumper.
  Caution: on a pin the peripheral also drives, this is output-against-output
  contention.
- **Verify the rig before judging the device.** Every probe self-checks first and
  declares itself inconclusive rather than blaming the part.
- **Single-variable substitution.** Moving the radio to a breadboard on identical
  pins removed only the PCB, and answered the question in one move.
- **Re-flash the last known good commit.** This exonerated the firmware for the
  CM1106 and converted an argument into a measurement.
- **Discover protocols from the device.** Sweep command bytes with correct checksums
  rather than guessing against an un-vendored datasheet.

### Traps already hit — do not repeat

- **`RTC_DATA_ATTR` does not survive a chip reset.** `.rtc.data` is reloaded from
  the image every boot, so a counter there reads 1 forever. Use `RTC_NOINIT_ATTR`
  with a magic word. (It *does* survive deep sleep, which is what it is for.)
- **A serial capture loop must catch `TimeoutException` inside its read loop.**
  Letting it escape reopened the port constantly and produced a reported "127 USB
  re-enumerations" that never happened.
- **A pin reading "driven high" may just be a pull-up.** Both radio UART pins read
  driven high during the floating-ground period, including one wired to a module
  *input*, which cannot drive anything.
- **Continuity and voltage both pass with a floating ground.** Continuity probes
  within a net; voltage is measured against the same bad reference.

---

## 10. Unknowns

- **Fire thresholds.** The `alarma` and `prealarma` spreadsheet tabs were never
  transcribed into the repo.
- **Battery or supercaps.** `power_budget.md` specifies a 600 F bank; the current
  design is described as a battery with two step-ups. Autonomy depends on which.
- **The power MCU.** A second microcontroller owns the boost converters. Nothing in
  the repo documents it, and it shapes any future sleep work.
- **Whether a command can trigger a CM1106 measurement** without a full EN power
  cycle. `0x06` and `0x0F` return status frames and are the candidates; cycling EN
  works today but costs the warm-up on every reading.
