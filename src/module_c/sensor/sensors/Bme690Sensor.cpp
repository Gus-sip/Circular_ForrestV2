#include "Bme690Sensor.h"

#include <Wire.h>

// Raw read of the chip-ID register, bypassing the Bosch driver entirely.
//
// Needed because the library's begin() collapses several very different failures
// into one `false`: the I2C ACK probe, bme69x_init()'s soft reset, its chip-ID
// check, the variant read and the calibration load. Node C2 showed `ACK at 0x76`
// in the bus scan and `BME690: NOT FOUND` from begin() on the SAME boot, which
// means the device answers its address but something after that is failing - and
// without the raw value there is no way to tell a wrong part from a corrupt read.
//
// The expected value is 0x61 (shared by BME680/688/690). Reports on the forums
// show this register coming back as 0x00 or 0xF1 on buses that otherwise ACK,
// which is bus corruption rather than a mismatched part - a one-byte address probe
// is short and survives, while a register read is a longer transaction and does
// not.
// Two access patterns, because they fail for different reasons and a device that
// refuses one may answer the other:
//
//   mode 0  write register, REPEATED START, read   - what the Bosch driver does
//   mode 1  write register, STOP, then a fresh read transaction
//
// Some I2C devices and some bus conditions tolerate only one. Reporting which (if
// either) works turns "does not respond" into something actionable: if mode 1
// succeeds where mode 0 fails, the part is alive and the repeated start is the
// problem; if both fail while the bare address still ACKs, the device is powered
// enough to pull the line but not to run.
static bool readChipIdMode(uint8_t addr, uint8_t &out, int mode) {
  Wire.beginTransmission(addr);
  Wire.write(0xD0);  // BME69X_REG_CHIP_ID
  if (Wire.endTransmission(mode == 1) != 0) return false;
  if (mode == 1) delay(2);
  if (Wire.requestFrom((int)addr, 1) != 1) return false;
  out = (uint8_t)Wire.read();
  return true;
}

static bool readChipId(uint8_t addr, uint8_t &out, int &modeUsed) {
  for (int mode = 0; mode < 2; mode++) {
    if (readChipIdMode(addr, out, mode)) {
      modeUsed = mode;
      return true;
    }
    delay(5);
  }
  modeUsed = -1;
  return false;
}

// The first forced measurement after begin() ALWAYS fails on this part.
//
// Measured on node C2: read 1 -> Timeout, read 2 -> OK (gas 41451), read 3 -> OK,
// on every single boot. The library's getData() triggers a forced measurement,
// waits the calculated duration, then treats any non-OK status from
// bme69x_get_data() as failure - and Bosch warnings such as NO_NEW_DATA are
// POSITIVE status codes, so a first-measurement warning is indistinguishable from
// a hard error there. The gas heater also needs one run before its reading is
// stable, which is why the first measurement is the one that warns.
//
// This cost one reading in a hundred while the node sampled continuously, so it
// went unnoticed. Under the sleep cycle it is fatal: every wake is a fresh begin()
// followed by exactly ONE read, so the only read taken is the one that always
// fails, and the BME690 would never contribute a single value to a packet.
//
// Hence a discarded priming measurement here, and one retry in read().

bool Bme690Sensor::begin() {
  _chipId = 0;
  _chipIdValid = false;

  // Retry the whole bring-up rather than failing on one bad attempt.
  //
  // bme69x_init() issues a SOFT RESET and then immediately reads the chip ID. A
  // part that has only just had its rail switched on - which is every wake under
  // the sleep cycle - can still be settling when that read lands, and the driver
  // reports the same `false` as a genuinely absent sensor. Three tries with a gap
  // costs ~150ms on the failing path and nothing at all on the healthy one.
  for (int attempt = 0; attempt < 3; attempt++) {
    if (_sensor.begin(_addr)) {
      primeAndMark();
      return true;
    }

    uint8_t altAddr = (_addr == 0x76) ? 0x77 : 0x76;
    if (_sensor.begin(altAddr)) {
      _addr = altAddr;
      primeAndMark();
      return true;
    }

    delay(50);
  }

  // Failed. Capture the raw chip ID so the caller can say WHY rather than just
  // "NOT FOUND" - 0x61 means the part is fine and the driver is at fault, 0x00 or
  // garbage means the bus is corrupting reads, and no response at all means it
  // really is absent.
  _chipIdValid = readChipId(_addr, _chipId, _chipIdMode);
  return false;
}

void Bme690Sensor::primeAndMark() {
  // Priming measurement, deliberately discarded. Costs ~200ms, and on the sleep
  // path it lands inside the BMV080's 5s startup wait anyway, so it is free.
  float t, h, pr, g;
  _sensor.getData(t, h, pr, g);
  _primed = true;
  _chipId = 0x61;  // it initialised, so the ID matched by definition
  _chipIdValid = true;
  _chipIdMode = 0;
}

Reading Bme690Sensor::read() {
  Reading r;
  float temperature, humidity, pressure, gas;

  // One retry. The priming read in begin() handles the cold case; this covers a
  // measurement that warns mid-run, which would otherwise discard a whole wake's
  // environmental data. Two failures in a row is a real fault worth reporting.
  for (int attempt = 0; attempt < 2; attempt++) {
    if (_sensor.getData(temperature, humidity, pressure, gas)) {
      r.status = ReadingStatus::Ok;
      r.values[0] = temperature;
      r.values[1] = humidity;
      r.values[2] = pressure;
      r.values[3] = gas;
      r.count = 4;
      return r;
    }
    delay(20);
  }

  r.status = ReadingStatus::Timeout;
  return r;
}
