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
#define PIN_CALYPSO_TX 8  // ESP TX -> sensor RX (unused - sensor streams unprompted)
#define PIN_CALYPSO_RX 9  // ESP RX <- sensor TX

// Spare pin, unused for now (reserved for a future CM1106 RDY line).
#define PIN_SPARE 12
