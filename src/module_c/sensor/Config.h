#pragma once

// CHIP FOREST bring-up configuration. No magic numbers in the drivers - everything
// tunable lives here.

// Sensor I2C addresses - confirmed on this specific bench harness via I2C scan and
// successful reads. Address-strap pins float on more than one of these boards, so a
// different unit could legitimately need a different value; every driver still takes
// its address as a constructor argument rather than hardcoding it internally.
#define BME690_I2C_ADDR 0x76   // SDO strap; alternate 0x77 tried automatically as fallback
#define SEN0466_I2C_ADDR 0x74
#define BMV080_I2C_ADDR 0x57   // CS=high, SDO=high strap on the shuttle board's P4 header

// Warm-up / startup timing. Values marked UNCONFIRMED are not sourced from a
// datasheet page - they're conservative placeholders that worked empirically during
// bring-up, kept as named constants so they're easy to find and correct later.
#define CM1106_WARMUP_MS 3000          // UNCONFIRMED vs datasheet
#define BMV080_STARTUP_DELAY_MS 5000   // UNCONFIRMED vs datasheet - HW init/laser preheat/self-test

// UART baud rates
#define CM1106_BAUD 9600
#define CALYPSO_BAUD 38400

// How often the bring-up Sampler prints a reading table
#define SAMPLE_INTERVAL_MS 1000
