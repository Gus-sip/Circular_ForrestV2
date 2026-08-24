# CHIP FOREST — Sensor Bring-Up

Solar-harvesting environmental monitoring node for forest deployment. This milestone
covers **bench bring-up only**: get all five sensors producing trustworthy numbers
over USB serial on an ESP32-S3-Zero (ESP32-S3FH4R2). The custom PCB (Altium
`PCB_Nodo`) isn't fabbed yet — everything here runs on a bench harness wired to the
same GPIOs the PCB uses, on one always-on 3.3V rail, with no power-gating MOSFETs in
the loop.

**Not in scope for this milestone:** deep sleep, RTC wake, LoRa, telemetry packing.
Those come later, once the gated-power hardware exists.

## Build / flash

```
pio run -e chip-forest-v1 -t upload
pio device monitor -e chip-forest-v1
```

`ARDUINO_USB_CDC_ON_BOOT=1` is required in `platformio.ini` for this board — without
it, `Serial` goes nowhere and the board looks dead over USB. It's already set for the
`chip-forest-v1` environment.

**One manual step after a clean checkout or `.pio` wipe:** the BMV080's Bosch SDK is a
closed-source static library that PlatformIO's library manager doesn't fetch. After
`pio run -e chip-forest-v1` creates `.pio/libdeps/chip-forest-v1/SparkFun BMV080 Arduino
Library/`, copy the vendored blobs in:

```powershell
$libDir = ".pio\libdeps\chip-forest-v1\SparkFun BMV080 Arduino Library\src"
Copy-Item "bosch_bmv080_sdk\bmv080.h" "$libDir\sfTk\bmv080.h" -Force
Copy-Item "bosch_bmv080_sdk\bmv080_defs.h" "$libDir\sfTk\bmv080_defs.h" -Force
Copy-Item "bosch_bmv080_sdk\lib_bmv080.a" "$libDir\esp32s3\lib_bmv080.a" -Force
Copy-Item "bosch_bmv080_sdk\lib_postProcessor.a" "$libDir\esp32s3\lib_postProcessor.a" -Force
```

Without this, the build fails with `bmv080.h not found`.

## Wiring table

| Sensor | Bus | Pins | Address / notes |
|---|---|---|---|
| BME690 | I2C (shared bus) | SDA=GPIO1, SCL=GPIO2 | `0x76` (SDO low) confirmed; `0x77` tried as fallback since SDO floats on this board |
| SEN0466 | I2C (shared bus) | SDA=GPIO1, SCL=GPIO2 | `0x74`, confirmed |
| BMV080 | I2C (shared bus) | SDA=GPIO1, SCL=GPIO2 | `0x57` (CS=high, SDO=high strap on shuttle board P4) |
| CM1106SL-NS | UART2 | TX=GPIO6, RX=GPIO7, EN=GPIO3 (active high) | 9600 baud; runs fine at 3.3V on this board |
| Calypso ULP PRO | UART1 | TX=GPIO8, RX=GPIO9 | 38400 baud; confirmed TTL-UART/NMEA variant (not RS485) |

GPIO12 is spare, reserved for a future CM1106 RDY line.

### BMV080 shuttle board 3.1 — full pin detail

The BMV080 needed more than SDA/SCL/power to come up. Two headers, both required:

**P4 (signal):**

| Pin | Connect to |
|---|---|
| SDI/SDA | GPIO1 |
| SCK/SCL | GPIO2 |
| GND | GND |
| VDD | 3.3V |
| CS | 3.3V (I2C address strap) |
| SDO | 3.3V (I2C address strap) |
| PS | 3.3V (protocol select — I2C vs SPI) |
| IRQ | unconnected (polling `bmv080_serve_interrupt` instead) |

CS + SDO set the I2C address: both high → `0x57`, CS high/SDO low → `0x56`, CS
low/SDO high → `0x55`, both low → `0x54`.

**P3 (power) — each row must be *bridged*, not just individually tied to 3.3V:**

| Row | Left | Right | Wiring |
|---|---|---|---|
| 1 | VDDIO | VDDIO_S | jumper together → 3.3V |
| 2 | VDD | VDDL | jumper together → 3.3V |
| 3 | VDD | VDDD | jumper together → 3.3V |
| 4 | VDD | VDDA | jumper together → 3.3V |

This was the actual root cause of the BMV080 not responding at all during bring-up:
VDDL/VDDD/VDDA (the sensor's internal digital/analog core rails) are distinct from
VDD/VDDIO and were left unpowered. The symptom was `bmv080_open()` failing with
`E_BMV080_ERROR_HW_WRITE` (no I2C ACK whatsoever) — a transport-level failure that
would occur regardless of any firmware fix, because the digital core running the I2C
peripheral itself was off.

## Bring-up order

1. **I2C sensors first** (BME690, SEN0466, BMV080) — shared bus, so a scan at boot
   catches wiring/address problems for all three at once.
2. **BMV080 needs a startup delay before `begin()`** (`BMV080_STARTUP_DELAY_MS` in
   `include/Config.h`, currently 5000ms) for its hardware init, laser preheat, and
   optical self-test. This value is **not** sourced from a confirmed datasheet page —
   it's a conservative placeholder that worked empirically.
3. **CM1106** — EN driven high, then a warm-up delay (`CM1106_WARMUP_MS`, 3000ms,
   also unconfirmed against a datasheet) before the first read request.
4. **Calypso** — just opens the UART; it streams NMEA sentences unprompted, no
   handshake needed.
5. Sampler prints one line per sensor, once a second, for whichever sensors made it
   through `begin()`.

## What a passing run looks like

```
=== CHIP FOREST bring-up: BME690 + SEN0466 + BMV080 + CM1106 + Calypso ===
Waiting 5000ms for BMV080 startup...
BME690   OK
SEN0466  OK
BMV080   OK
CM1106   OK
Calypso  OK

BME690   27.48 29.72 938.63 80629.92
SEN0466  0.00 31.00
BMV080   2.00 4.00 6.00 0.00
CM1106   815.00
Calypso  0.00 0.00 1.00
```

Column meanings (see each driver's header comment for the authoritative version):

- **BME690**: temperature °C, humidity %, pressure hPa, gas resistance Ω (climbs
  steadily as the heater warms up — that's normal, not an error)
- **SEN0466**: CO ppm, onboard temperature °C
- **BMV080**: PM1, PM2.5, PM10 (µg/m³), obstructed (0/1) — values fluctuate and may
  read low/zero for the first several seconds while the sensor stabilizes
- **CM1106**: CO2 ppm
- **Calypso**: wind angle (deg), wind speed, status valid (0/1) — `0 0 1` on a
  stationary indoor bench is expected (no wind, but a valid sentence)

A sensor that fails `begin()` simply doesn't get a row printed and doesn't block the
others. A sensor that's ready but has nothing new prints `(not ready)`,
`(timeout)`, `(no ack)`, or `(invalid frame)` instead of numbers — see
`src/module_c/sensor/sensors/Reading.h` for what each means.

## Architecture

Diagram below reflects this doc's original standalone project layout; in
`circular_forest_v2` these same files live under `src/module_c/sensor/` (see
`../circular_forest_v2_README.md` for the current layout).

```
include/
  pins.h        - pin map
  Config.h      - addresses, timing constants, baud rates
src/
  chip_forest_v1.cpp  - entry point: wires up all 5 sensors, runs the Sampler
  Sampler.{h,cpp}     - walks registered ISensors, prints the table
  PowerManager.{h,cpp} - stub; bench has no gating FETs yet, but the API shape
                          (enable3V3Sensors/disable3V3Sensors) is what the real
                          power-gated version will expose
  sensors/
    ISensor.h         - common interface: begin() / read() / sleep()
    Reading.h         - typed result with explicit validity (not sentinel floats)
    Bme690Sensor.*
    Sen0466Sensor.*
    Cm1106Sensor.*
    CalypsoSensor.*
    Bmv080Sensor.*    - uses the raw Bosch SDK directly, not the SparkFun Arduino
                          wrapper - the wrapper collapses every bmv080_status_code_t
                          into a bare bool, which is what made the wiring failure
                          above hard to diagnose in the first place
```

`src/bringup.cpp` and the various single-sensor test files (`bme690.cpp`,
`bmv080.cpp`, `cm1106.cpp`, `cm1106_read.cpp`, `sen0466.cpp`,
`ulp_pro_uart_test.cpp`) are earlier breadboard/isolation sketches, each with its own
PlatformIO environment. They're kept around as reference and diagnostic tools, not
part of the `chip-forest-v1` build.

## Known open questions / deferred work

- **BSEC vs. duty cycling**: BME690's gas resistance is logged raw. A real IAQ index
  needs BSEC2 and sustained runtime to converge, which conflicts with the
  duty-cycled design planned for the powered version. Not resolved yet — this bench
  milestone runs continuously powered, so the conflict doesn't bite here, but it will
  need a decision (log raw only vs. persist BSEC state across sleep) before deep
  sleep is added.
- **CM1106 / BMV080 warm-up durations** are placeholders, not datasheet-confirmed
  values.
- **Calypso poll mode**: the sensor supports being polled instead of streaming
  continuously (lower power for the duty-cycled version), not implemented yet -
  streaming is fine for a continuously-powered bench.
- **CM1106 COMSEL** (UART vs I2C mode select) is unstrapped on this board revision,
  but the part has consistently returned valid UART frames, so it's evidently
  defaulting into UART mode reliably on this specific unit.
