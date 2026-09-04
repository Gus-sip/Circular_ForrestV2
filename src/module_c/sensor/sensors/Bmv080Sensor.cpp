#include "Bmv080Sensor.h"

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
  _bus.init(_wire, _addr);
  _bus.setByteOrder(SFTK_MSBFIRST);

  bmv080_status_code_t rc =
      bmv080_open(&_handle, (bmv080_sercom_handle_t)&_bus, (bmv080_callback_read_t)readCB,
                  (bmv080_callback_write_t)writeCB, (bmv080_callback_delay_t)delayCB);
  if (rc != E_BMV080_OK) return false;

  bmv080_reset(_handle);

  bool doObstructionDetection = true;
  bmv080_set_parameter(_handle, "do_obstruction_detection", (void *)&doObstructionDetection);

  rc = bmv080_start_continuous_measurement(_handle);
  _measuring = (rc == E_BMV080_OK);
  return _measuring;
}

bool Bmv080Sensor::startMeasurement() {
  if (_handle == nullptr) return false;
  if (_measuring) return true;
  bmv080_status_code_t rc = bmv080_start_continuous_measurement(_handle);
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
