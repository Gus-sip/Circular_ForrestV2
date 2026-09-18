#pragma once

#include "ISensor.h"
#include <Wire.h>
#include <sfTkArdI2C.h>
#include <sfTk/bmv080.h>
#include <sfTk/bmv080_defs.h>

// I2C, shared bus. Uses the raw Bosch SDK directly rather than the SparkFun Arduino
// wrapper (SparkFunBMV080/sfDevBMV080) - that wrapper collapses every
// bmv080_status_code_t into a bare bool, which made the original wiring failure
// (bmv080_open failing with E_BMV080_ERROR_HW_WRITE) impossible to see. The read/write
// callbacks below are copied from the wrapper's own private implementation, not
// reinvented: left-shift the 16-bit header by 1 bit for I2C, then delegate to
// sfTkIBus::readRegister/writeRegister, with MSB-first byte order.
//
// SET_LOOP_TASK_STACK_SIZE(60*1024) in the .cpp is required - bmv080_serve_interrupt
// needs more stack than the ESP32 Arduino default loop task gets (per SparkFun's own
// library header comment).
//
// values[]: [0]=PM1, [1]=PM2.5, [2]=PM10 (ug/m3), [3]=obstructed (1/0).
// Needs several seconds of continuous operation before a reading is valid (BMV080_STARTUP_DELAY_MS
// in Config.h covers HW init/laser preheat/self-test) - readings can arrive before the
// value has fully stabilized, since is_outside_measurement_range is the only validity
// flag the SDK itself exposes.
class Bmv080Sensor : public ISensor {
public:
  explicit Bmv080Sensor(uint8_t addr, TwoWire &wire = Wire) : _addr(addr), _wire(wire) {}

  bool begin() override;
  Reading read() override;
  void sleep() override;
  const char *name() const override { return "BMV080"; }

  // Duty-cycling. This part draws ~68mA with the laser running - by an order of
  // magnitude the largest load in the system, and notes/power_budget.md budgets it
  // for 20s every 30 min, not continuous operation. begin() leaves it measuring so
  // presence is proven; the caller is expected to stopMeasurement() straight after
  // and only start it again around an actual sample. Start/stop reuse the open
  // handle, so neither pays the open/reset/self-test cost again.
  bool startMeasurement();
  bool stopMeasurement();
  bool isMeasuring() const { return _measuring; }

  // Why begin() failed, and the SDK status code behind it. begin() otherwise
  // collapses "not on the bus at any strap address" and "present but the laser
  // would not start" into one bool - two faults with completely different causes
  // and fixes.
  enum class FailStage { None, Open };
  FailStage failStage() const { return _failStage; }
  int lastOpenStatus() const { return _lastOpenStatus; }
  // SDK status from the last attempt to start the laser.
  int lastStartStatus() const { return _lastStartStatus; }

private:
  uint8_t _addr;
  TwoWire &_wire;
  sfTkArdI2C _bus;
  bmv080_handle_t _handle = nullptr;
  FailStage _failStage = FailStage::None;
  int _lastOpenStatus = 0;
  int _lastStartStatus = 0;
  bool _measuring = false;

  volatile bool _dataReady = false;
  bmv080_output_t _lastOutput{};

  static int8_t readCB(bmv080_sercom_handle_t handle, uint16_t header, uint16_t *payload, uint16_t payloadLength);
  static int8_t writeCB(bmv080_sercom_handle_t handle, uint16_t header, const uint16_t *payload,
                         uint16_t payloadLength);
  static int8_t delayCB(uint32_t durationMs);
  static void onDataReady(bmv080_output_t output, void *callbackParameters);
};
