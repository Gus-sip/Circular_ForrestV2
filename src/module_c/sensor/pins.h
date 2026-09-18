#pragma once

// CHIP FOREST bench harness pin map. Board isn't fabbed yet - this mirrors the
// intended PCB wiring on a bench harness, one always-on 3V3 rail, no power gating.
// Every value below was confirmed empirically this session (I2C scans, working
// reads), not assumed from a schematic.

// Shared I2C bus: BME690, SEN0466, BMV080 all share this bus.
#define PIN_I2C_SDA 1
#define PIN_I2C_SCL 2

// Cubic CM1106SL-NS CO2 sensor: UART2. Confirmed running fine at 3.3V on this board
// (an earlier unrelated project's CM1106 unit needed 5V - that doesn't apply here).
#define PIN_CM1106_TX 6  // ESP TX -> sensor RX
#define PIN_CM1106_RX 7  // ESP RX <- sensor TX
#define PIN_CM1106_EN 3  // active high, through a 1k series resistor on the board

// Calypso ULP PRO wind sensor: UART1. Confirmed TTL-UART/NMEA variant (not RS485).
// 2026-09-01: RX moved 9->8 and TX 8->9. The sensor's TX (green) is physically
// on GPIO8 - a bench sweep across GPIO5/6/8/9 x baud 4800-115200 got a valid
// $IIMWV sentence ONLY on RX=GPIO8 @ 38400. Also: this unit needs 5V VCC, not
// 3.3V (the old "confirmed fine at 3.3V" note was wrong for it - it was silent
// on 3.3V, streamed the moment it moved to 5V).
#define PIN_CALYPSO_TX 9  // ESP TX -> sensor RX/yellow (unused - sensor streams unprompted)
#define PIN_CALYPSO_RX 8  // ESP RX <- sensor TX/green

// Spare pin, unused for now (reserved for a future CM1106 RDY line).
#define PIN_SPARE 12

// BMV080 enable line, added to the board after this file was first written.
//
// Must be driven LOW for the sensor to run. Left undriven the BMV080 opens over
// I2C perfectly - it answers at its strap address and bmv080_open() succeeds - but
// refuses to start its laser, returning SDK status 114
// (E_BMV080_ERROR_OPERATION_MODE_CHANNELS_OUT_OF_SYNC), whose Bosch hint is
// "check that power provided to BMV080 is sufficient & stable". That hint points
// at the supply, which is exactly what an unasserted enable line looks like from
// the sensor's side.
//
// Driven with the rail gates, before any sensor is opened, so it is settled by the
// time the SDK talks to the part.
#define PIN_BMV080_EN 14
#define PIN_BMV080_EN_ACTIVE LOW

// ---------- Module C PCB enable lines (GPIO10/11) ----------
// The real PCB (as opposed to the bench harness this file originally described)
// gates BOTH sensor supply rails behind these: each drives a high-side switching
// transistor that closes the circuit into the 5V rail and the 3V3 rail
// respectively (confirmed 2026-09-04). LOW = transistor on = rail up, the same
// convention as Module B's LORA_EN_PIN. They were unused by every other pin
// above, so nothing on this board contends for them.
//
// These MUST be asserted at the very top of setup(), before Wire.begin() and
// every sensor begin(): with them deasserted the sensors have no power at all,
// so an I2C scan or a UART probe would find nothing and look like dead hardware.
// The Calypso wind sensor and the CM1106 both need the 5V rail specifically.
//
// THE TWO GATES HAVE OPPOSITE POLARITY. Established 2026-09-08 by sweeping all
// four GPIO10/11 combinations and reading the CM1106 (the only connected 5V device)
// in each:
//
//     GPIO10=LOW  GPIO11=LOW   -> silent
//     GPIO10=LOW  GPIO11=HIGH  -> answers      <- 5V rail up
//     GPIO10=HIGH GPIO11=LOW   -> silent
//     GPIO10=HIGH GPIO11=HIGH  -> answers      <- 5V rail up
//
// So GPIO11 gates the 5V rail and is ACTIVE-HIGH, while GPIO10 (3V3) is active-low
// as originally assumed - the 3V3 sensors have always worked with it LOW.
//
// This was masked for days by the floating-ground fault: with no solid reference,
// driving GPIO11 "LOW" was not a true low at the transistor, so the 5V rail stayed
// up by accident. Repairing the ground made LOW a real low and switched the 5V rail
// properly OFF, which read as the CM1106 dying. It had not; it was unpowered.
//
// Anything on the 5V rail - CM1106 and the Calypso wind sensor - depends on this.
#define PIN_PCB_EN_A 10          // 3V3 sensor rail
#define PIN_PCB_EN_A_ACTIVE LOW
#define PIN_PCB_EN_B 11          // 5V sensor rail
#define PIN_PCB_EN_B_ACTIVE HIGH
#define PIN_PCB_EN_SETTLE_MS 300

// Retained so older sketches still compile, but it is only correct for gate A.
// Prefer the per-gate macros above.
#define PIN_PCB_EN_ACTIVE PIN_PCB_EN_A_ACTIVE

// ---------- RYLR998 LoRa power gate (GPIO13) ----------
// Third rail gate, separate from the two above: a P-FET on the LoRa supply,
// LOW = on. Same pin and same convention as Module B's LORA_EN_PIN (Q4 gate,
// found by pin-scan 2026-08-19) - this PCB reuses that design. Module C's
// firmware never drove it, which is why the RYLR998 was silent to a bare AT at
// every baud: the module simply had no power. Confirmed by the user 2026-09-04.
// The radio's UART is RXD=GPIO5 (ESP TX) / TXD=GPIO4 (ESP RX) - nets are named
// from the MCU's point of view, so that crossover is correct as written.
#define PIN_LORA_EN 13
#define PIN_LORA_EN_ACTIVE LOW
#define PIN_LORA_EN_SETTLE_MS 200

// ---------- Status LED (GPIO21) ----------
// Addressable WS2812 on the module, driven with neopixelWrite(). Confirmed 21 by
// sweeping every safe GPIO - it is NOT the esp32s3 variant's PIN_NEOPIXEL 48, and
// not the GPIO38 some boards use.
//
// Never drive it via RGB_BUILTIN: the variant defines that as
// SOC_GPIO_PIN_COUNT + PIN_NEOPIXEL = 97, a sentinel digitalWrite() special-cases.
// neopixelWrite() wants a real GPIO, so 97 addresses a pin that does not exist and
// lights nothing, silently.
#define PIN_STATUS_LED 21
