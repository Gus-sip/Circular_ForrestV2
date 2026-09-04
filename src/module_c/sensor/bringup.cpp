#include <Arduino.h>
#include <Wire.h>
#include <sfTkArdI2C.h>
#include <sfTk/bmv080.h>
#include <sfTk/bmv080_defs.h>

// BMV080 raw-SDK isolation probe. Bypasses the SparkFunBMV080 Arduino wrapper on
// purpose: that wrapper collapses bmv080_open/bmv080_reset/bmv080_start_continuous_measurement
// into bare booleans, so a failure in any one of them is indistinguishable from any other.
// This prints the real bmv080_status_code_t from each call instead.
//
// Reused verbatim from the vendored SparkFun library (not reinvented):
//   - The read/write callback shape: left-shift the 16-bit header by 1 bit for I2C,
//     then delegate to sfTkIBus::readRegister/writeRegister. Copied from
//     sfDevBMV080::device_read_16bit_CB / device_write_16bit_CB (SparkFun BMV080
//     Arduino Library/src/sfTk/sfDevBMV080.cpp) - those are private, so re-implemented
//     here as free functions rather than guessed at.
//   - sfTkArdI2C::setByteOrder(SFTK_MSBFIRST) - SparkFunBMV080::begin() calls this
//     before use; the Bosch protocol is documented as MSB-first 16-bit words
//     (bmv080_defs.h), and skipping this call would leave byte order unset/mismatched.
//   - SET_LOOP_TASK_STACK_SIZE(60*1024) - SparkFun_BMV080_Arduino_Library.h calls this
//     unconditionally on ESP32 with the comment "bmv080_serve_interrupt is the culprit"
//     (needs more stack than the Arduino loop task's default). Skipping this risks a
//     stack overflow inside bmv080_serve_interrupt that has nothing to do with wiring.
#if defined(ESP32)
SET_LOOP_TASK_STACK_SIZE(60 * 1024);
#endif

#define I2C_SDA_PIN 1
#define I2C_SCL_PIN 2

// I2C address: tied to the CS/SDO strap combination on the shuttle board's P4 header.
//   CS high, SDO high -> 0x57
//   CS high, SDO low  -> 0x56
//   CS low,  SDO high -> 0x55
//   CS low,  SDO low  -> 0x54
// One-line edit when the jumpers change.
// The fabbed PCB straps CS high / SDO high -> 0x57 (I2C scan, 6/6 boots, 2026-09-04).
#define BMV080_I2C_ADDR 0x57

// Sensor runs HW init, laser preheat, and an optical self-test after power-up.
// Not yet confirmed against the datasheet's exact spec (open question) - this is a
// conservative starting point, not a magic number.
#define BMV080_STARTUP_DELAY_MS 5000

// PCB power-rail enables. Each drives a high-side transistor closing the circuit
// into the 5V / 3V3 rail; LOW = rail on. Kept in sync with pins.h (this probe
// deliberately declares its own pins rather than including the production map).
// Without these the whole I2C bus is unpowered and the scan below finds nothing.
#define PCB_EN_A_PIN 10
#define PCB_EN_B_PIN 11
#define PCB_EN_SETTLE_MS 300

sfTkArdI2C bmvBus;
bmv080_handle_t bmvHandle = nullptr;
bool measuring = false;

// ---------- Bosch SDK callbacks (I2C register bridge, copied from the vendored
// SparkFun implementation's private methods since those aren't externally callable) ----------
static int8_t bmvReadCB(bmv080_sercom_handle_t handle, uint16_t header, uint16_t *payload, uint16_t payload_length) {
  if (handle == nullptr) return -5;
  sfTkIBus *bus = (sfTkIBus *)handle;
  if (bus->type() == ksfTkBusTypeI2C) header = header << 1;
  size_t nRead = 0;
  sfTkError_t rc = bus->readRegister(header, payload, payload_length, nRead);
  return (rc == ksfTkErrOk && nRead == payload_length) ? 0 : -2;
}

static int8_t bmvWriteCB(bmv080_sercom_handle_t handle, uint16_t header, const uint16_t *payload, uint16_t payload_length) {
  if (handle == nullptr) return -5;
  sfTkIBus *bus = (sfTkIBus *)handle;
  if (bus->type() == ksfTkBusTypeI2C) header = header << 1;
  sfTkError_t rc = bus->writeRegister(header, payload, payload_length);
  return (rc == ksfTkErrOk) ? 0 : -3;
}

static int8_t bmvDelayCB(uint32_t duration_in_ms) {
  delay(duration_in_ms);
  return 0;
}

static void onDataReady(bmv080_output_t output, void *callback_parameters) {
  Serial.printf("  DATA READY: PM1=%.2f PM2.5=%.2f PM10=%.2f obstructed=%d outside_range=%d runtime=%.1fs\n",
                output.pm1_mass_concentration, output.pm2_5_mass_concentration, output.pm10_mass_concentration,
                output.is_obstructed, output.is_outside_measurement_range, output.runtime_in_sec);
}

static void scanI2C() {
  Serial.println("--- I2C scan 0x08-0x77 ---");
  byte found = 0;
  for (byte addr = 0x08; addr <= 0x77; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  ACK at 0x%02X\n", addr);
      found++;
    }
  }
  if (found == 0) Serial.println("  (nothing acked)");
}

// Runs the full open->reset->get_id->set_parameter->start_measurement handshake once,
// printing every status code. Callable repeatedly from loop() while not yet measuring,
// so a monitor attaching at any time - not just at reset - still sees a full attempt,
// instead of racing a one-shot setup() print.
static void attemptBringup() {
  if (bmvHandle != nullptr) {
    bmv080_close(&bmvHandle);
    bmvHandle = nullptr;
  }

  bmv080_status_code_t rc;

  rc = bmv080_open(&bmvHandle, (bmv080_sercom_handle_t)&bmvBus, (bmv080_callback_read_t)bmvReadCB,
                    (bmv080_callback_write_t)bmvWriteCB, (bmv080_callback_delay_t)bmvDelayCB);
  Serial.printf("bmv080_open: %d%s\n", (int)rc, rc == E_BMV080_OK ? " (OK)" : "");
  if (rc != E_BMV080_OK) {
    Serial.println("Stopping here - open failed, nothing downstream will work.");
    return;
  }

  rc = bmv080_reset(bmvHandle);
  Serial.printf("bmv080_reset: %d%s\n", (int)rc, rc == E_BMV080_OK ? " (OK)" : "");

  char id[13];
  rc = bmv080_get_sensor_id(bmvHandle, id);
  if (rc == E_BMV080_OK) {
    Serial.printf("bmv080_get_sensor_id: %d (OK) id=%s\n", (int)rc, id);
  } else {
    Serial.printf("bmv080_get_sensor_id: %d\n", (int)rc);
  }

  bool doObstructionDetection = true;
  rc = bmv080_set_parameter(bmvHandle, "do_obstruction_detection", (void *)&doObstructionDetection);
  Serial.printf("bmv080_set_parameter(do_obstruction_detection): %d%s\n", (int)rc, rc == E_BMV080_OK ? " (OK)" : "");

  rc = bmv080_start_continuous_measurement(bmvHandle);
  Serial.printf("bmv080_start_continuous_measurement: %d%s\n", (int)rc, rc == E_BMV080_OK ? " (OK)" : "");
  measuring = (rc == E_BMV080_OK);
}

void setup() {
  Serial.begin(115200);
  uint32_t waitStart = millis();
  while (!Serial && millis() - waitStart < 3000) delay(10);
  delay(500);

  Serial.println();
  Serial.println("=== BMV080 raw-SDK isolation probe ===");

  // Rails first - nothing on the bus is powered until these are asserted.
  pinMode(PCB_EN_A_PIN, OUTPUT);
  digitalWrite(PCB_EN_A_PIN, LOW);
  pinMode(PCB_EN_B_PIN, OUTPUT);
  digitalWrite(PCB_EN_B_PIN, LOW);
  Serial.printf("PCB rails: GPIO%d + GPIO%d LOW, settling %dms\n", PCB_EN_A_PIN,
                PCB_EN_B_PIN, PCB_EN_SETTLE_MS);
  delay(PCB_EN_SETTLE_MS);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  scanI2C();

  Serial.printf("Waiting %dms for sensor startup (HW init / laser preheat / self-test)...\n",
                BMV080_STARTUP_DELAY_MS);
  delay(BMV080_STARTUP_DELAY_MS);

  bmvBus.init(Wire, BMV080_I2C_ADDR);
  bmvBus.setByteOrder(SFTK_MSBFIRST);

  attemptBringup();
  Serial.println();
}

void loop() {
  static uint32_t lastHeartbeat = 0;
  static uint32_t lastRetry = 0;

  if (measuring) {
    bmv080_status_code_t rc = bmv080_serve_interrupt(bmvHandle, onDataReady, nullptr);
    if (rc != E_BMV080_OK) {
      Serial.printf("  bmv080_serve_interrupt: %d (error)\n", (int)rc);
    }
  } else if (millis() - lastRetry >= 3000) {
    lastRetry = millis();
    Serial.println("--- retrying bring-up ---");
    attemptBringup();
    Serial.println();
  }

  if (millis() - lastHeartbeat >= 1000) {
    lastHeartbeat = millis();
    Serial.printf("heartbeat t=%lus measuring=%d\n", (unsigned long)(millis() / 1000), measuring);
  }

  delay(50);
}
