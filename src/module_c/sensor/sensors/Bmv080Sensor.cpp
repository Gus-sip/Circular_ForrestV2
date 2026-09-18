#include "Bmv080Sensor.h"

// Settling after a reset before the sensor accepts a measurement command.
#define BMV080_POST_RESET_MS 100
// Pause between attempts to start the laser.
#define BMV080_START_RETRY_MS 150

#if defined(ESP32)
SET_LOOP_TASK_STACK_SIZE(60 * 1024);
#endif

int8_t Bmv080Sensor::readCB(bmv080_sercom_handle_t handle, uint16_t header, uint16_t *payload,
                             uint16_t payloadLength) {
  if (handle == nullptr) return -5;
  sfTkIBus *bus = (sfTkIBus *)handle;
  if (bus->type() == ksfTkBusTypeI2C) header = header << 1;
  size_t nRead = 0;
  sfTkError_t rc = bus->readRegister(header, payload, payloadLength, nRead);
  return (rc == ksfTkErrOk && nRead == payloadLength) ? 0 : -2;
}

int8_t Bmv080Sensor::writeCB(bmv080_sercom_handle_t handle, uint16_t header, const uint16_t *payload,
                              uint16_t payloadLength) {
  if (handle == nullptr) return -5;
  sfTkIBus *bus = (sfTkIBus *)handle;
  if (bus->type() == ksfTkBusTypeI2C) header = header << 1;
  sfTkError_t rc = bus->writeRegister(header, payload, payloadLength);
  return (rc == ksfTkErrOk) ? 0 : -3;
}

int8_t Bmv080Sensor::delayCB(uint32_t durationMs) {
  delay(durationMs);
  return 0;
}

void Bmv080Sensor::onDataReady(bmv080_output_t output, void *callbackParameters) {
  Bmv080Sensor *self = (Bmv080Sensor *)callbackParameters;
  self->_lastOutput = output;
  self->_dataReady = true;
}

bool Bmv080Sensor::begin() {
  // The I2C address is set by the CS/SDO straps, and it genuinely differs between
  // boards: node C1 answers at 0x57, node C2 at 0x56. Rather than require a
  // per-board build - unworkable across a 200-node deployment - try the configured
  // address first and then the rest of the strap range, exactly as Bme690Sensor
  // already does for its 0x76/0x77 pair.
  //
  //   CS high, SDO high -> 0x57      CS low,  SDO high -> 0x55
  //   CS high, SDO low  -> 0x56      CS low,  SDO low  -> 0x54
  static const uint8_t kStrapAddrs[4] = {0x57, 0x56, 0x55, 0x54};

  bmv080_status_code_t rc = E_BMV080_ERROR_HW_WRITE;
  for (uint8_t attempt = 0; attempt < 5; attempt++) {
    // Attempt 0 uses whatever was configured; later attempts walk the strap range
    // and skip the one already tried.
    uint8_t tryAddr = (attempt == 0) ? _addr : kStrapAddrs[attempt - 1];
    if (attempt > 0 && tryAddr == _addr) continue;

    _bus.init(_wire, tryAddr);
    _bus.setByteOrder(SFTK_MSBFIRST);

    rc = bmv080_open(&_handle, (bmv080_sercom_handle_t)&_bus, (bmv080_callback_read_t)readCB,
                     (bmv080_callback_write_t)writeCB, (bmv080_callback_delay_t)delayCB);
    if (rc == E_BMV080_OK) {
      _addr = tryAddr;  // remember it, so read() and any retry use the live address
      break;
    }
    // bmv080_open leaves no handle behind on failure, so nothing to close here.
    _handle = nullptr;
  }
  _lastOpenStatus = (int)rc;
  if (rc != E_BMV080_OK) {
    _failStage = FailStage::Open;
    return false;
  }

  bmv080_reset(_handle);
  // A reset needs settling before the sensor will accept a measurement command.
  // Without it the first bmv080_start_continuous_measurement() after begin() fails,
  // which used to be reported as "NOT FOUND" and is now reported as a read NoAck.
  delay(BMV080_POST_RESET_MS);

  bool doObstructionDetection = true;
  bmv080_set_parameter(_handle, "do_obstruction_detection", (void *)&doObstructionDetection);

  // DO NOT start a measurement here.
  //
  // begin() used to finish with bmv080_start_continuous_measurement() and return
  // its result, which conflated two entirely different failures under one "NOT
  // FOUND": the sensor being absent, and the sensor being present but unable to
  // START ITS LASER. The laser is the 68mA load that has browned this board out
  // repeatedly, so the second case is the likely one - and it was being reported
  // as a missing sensor, which sent the diagnosis toward the I2C bus and the
  // hardware instead of the supply.
  //
  // It was also wasted work: the caller stops the measurement again immediately,
  // because the sequential-slot design starts the laser only inside the BMV080's
  // own slot, with every other load quiesced. Opening the sensor is what begin()
  // is for; measuring is the slot's job.
  //
  // So begin() now succeeds when the sensor OPENS. A laser that will not start is
  // reported later, by the slot, as a read failure - which is what it actually is.
  _measuring = false;
  _failStage = FailStage::None;
  return true;
}

bool Bmv080Sensor::startMeasurement() {
  if (_handle == nullptr) return false;
  if (_measuring) return true;

  // Retry the start. This is where the laser actually fires - the 68mA load that
  // has browned this board out repeatedly - so a single refusal is not proof the
  // sensor is broken, and one retry after a short pause costs nothing on the
  // healthy path. The SDK status is kept so a persistent failure can be diagnosed
  // rather than guessed at.
  bmv080_status_code_t rc = E_BMV080_ERROR_HW_WRITE;
  for (int attempt = 0; attempt < 3; attempt++) {
    rc = bmv080_start_continuous_measurement(_handle);
    if (rc == E_BMV080_OK) break;
    delay(BMV080_START_RETRY_MS);
  }

  _lastStartStatus = (int)rc;
  _measuring = (rc == E_BMV080_OK);
  _dataReady = false;
  return _measuring;
}

bool Bmv080Sensor::stopMeasurement() {
  if (_handle == nullptr || !_measuring) return true;
  bmv080_status_code_t rc = bmv080_stop_measurement(_handle);
  _measuring = false;  // treat as off regardless - never leave the laser believed-on
  _dataReady = false;
  return rc == E_BMV080_OK;
}

Reading Bmv080Sensor::read() {
  Reading r;
  _dataReady = false;
  bmv080_status_code_t rc = bmv080_serve_interrupt(_handle, onDataReady, this);

  if (rc != E_BMV080_OK) {
    r.status = (rc == E_BMV080_ERROR_HW_WRITE || rc == E_BMV080_ERROR_HW_READ) ? ReadingStatus::NoAck
                                                                                : ReadingStatus::InvalidFrame;
    return r;
  }

  if (!_dataReady) {
    r.status = ReadingStatus::NotReady;
    return r;
  }

  r.status = ReadingStatus::Ok;
  r.values[0] = _lastOutput.pm1_mass_concentration;
  r.values[1] = _lastOutput.pm2_5_mass_concentration;
  r.values[2] = _lastOutput.pm10_mass_concentration;
  r.values[3] = _lastOutput.is_obstructed ? 1.0f : 0.0f;
  r.count = 4;
  return r;
}

void Bmv080Sensor::sleep() {
  stopMeasurement();
}
