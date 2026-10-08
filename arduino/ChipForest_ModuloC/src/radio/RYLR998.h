#pragma once

#include <Arduino.h>

// Shared by both Module C (TX) and Module B (RX) - see radio/README.md
// (carried over in src/sensor_node/radio/) for the starting link-parameter
// values and how they trade range against airtime. Previously duplicated
// per-module with only comment differences; deduped here since the driver
// itself was already identical.

// One inbound packet, parsed from a "+RCV=<addr>,<len>,<data>,<rssi>,<snr>" line.
// Payload cap (240 bytes) is the commonly documented RYLR998 AT+SEND limit - not
// re-derived from a vendored datasheet (none is in this repo), so treat it as a
// starting assumption to verify against this module's actual firmware/datasheet.
struct LoRaMessage {
  uint16_t senderAddr = 0;
  char payload[241] = {0};
  uint8_t length = 0;
  int16_t rssi = 0;
  int8_t snr = 0;
};

// AT+PARAMETER=<spreadingFactor>,<bandwidth>,<codingRate>,<preamble>. Bandwidth
// is the module's index/code, not a raw kHz value - see radio/README.md for the
// starting values used here and how they trade range against airtime.
struct RYLR998Params {
  uint8_t spreadingFactor;
  uint8_t bandwidth;
  uint8_t codingRate;
  uint16_t preamble;
};

// Minimal REYAX RYLR998 AT-command driver. begin()/send() are blocking (both
// wait on a quick +OK/+ERR from the module); poll() is not - it only drains
// bytes already sitting in the UART's RX buffer, so it's safe to call every
// loop() iteration without ever stalling on the radio (on Module B in
// particular, it must never stall the web server's handleClient()).
class RYLR998 {
public:
  RYLR998(HardwareSerial &serial, uint8_t rxPin, uint8_t txPin, uint32_t baud = 115200)
      : _serial(serial), _rxPin(rxPin), _txPin(txPin), _baud(baud) {}

  // Blocking: runs AT, AT+ADDRESS, AT+NETWORKID, AT+BAND, AT+PARAMETER in turn,
  // bailing out on the first one that doesn't come back +OK. False here means
  // check wiring/baud (rylr998_bridge.cpp) before assuming a parameter is bad.
  // Pass a `debug` stream (e.g. &Serial) to print each command and its reply
  // as it happens, so you can confirm the module actually accepted each
  // setting instead of just trusting one pass/fail bool at the end.
  bool begin(uint16_t addr, uint16_t networkId, uint32_t bandHz, const RYLR998Params &params,
             Print *debug = nullptr);

  // Blocking: writes AT+SEND=<addr>,<len>,<data> and waits for +OK, bounded by
  // kSendTimeoutMs. Actual on-air time depends on payload length and the
  // SF/BW/CR in effect, but this call never waits indefinitely. Not used by
  // Module B's dashboard path today, but kept available for a future LoRa ACK
  // back to the sensor node.
  bool send(uint16_t destAddr, const char *data, uint8_t len);

  // Blocking: AT+CRFOP=<dbm>, 0..22. The module defaults to 22 dBm, whose TX
  // burst pulls ~120mA - enough to brown out a supply that cannot source a step
  // load, which is exactly what Module C's PCB does (it reset on every transmit).
  // Lower power trades range for a smaller burst. Deliberately NOT folded into
  // begin(): this file is shared with Module B, which is mains-adjacent and has
  // no reason to give up range.
  bool setTxPower(uint8_t dbm, Print *debug = nullptr);

  // Non-blocking: drains whatever's already buffered, returns true (with
  // outMsg populated) the moment a complete, well-formed +RCV= line is found.
  // Anything else seen along the way (bare +OK echoes, garbage) is dropped.
  bool poll(LoRaMessage &outMsg);

private:
  static constexpr uint16_t kLineMax = 260;  // "+RCV=" + addr + len + 240B payload + rssi + snr
  static constexpr uint32_t kSendTimeoutMs = 2000;

  HardwareSerial &_serial;
  uint8_t _rxPin, _txPin;
  uint32_t _baud;

  char _line[kLineMax + 1] = {0};
  uint16_t _lineLen = 0;

  LoRaMessage _pending;
  bool _hasPending = false;

  bool sendATCommand(const char *cmd, Print *debug, uint32_t timeoutMs = 1000);
  bool waitForOK(uint32_t timeoutMs, Print *debug);
  static bool tryParseLine(char *line, uint16_t len, LoRaMessage &outMsg);
};
