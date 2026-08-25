#pragma once

#include <stdint.h>
#include <stddef.h>

#include "Config.h"
#include "telemetry/SensorSnapshot.h"

// Everything in this file is deliberately Arduino-free (no Arduino.h, no
// HardwareSerial) so it can be exercised by a native/off-target unit test
// (see test/test_nbiot_protocol) without pulling in the ESP32 toolchain.
// ModemNBIoT is the only thing that talks to the UART; it hands this module
// plain buffers/lines and gets structured results back.

namespace NbiotProtocol {

// ---------- AT response line parsers ----------
// Each takes one already-line-buffered, NUL-terminated response line (CRLF
// stripped, exactly what ModemNBIoT's byte-at-a-time reader hands over) and
// returns false if the line doesn't match at all, so the caller can try the
// next parser or treat it as an unrecognized URC rather than guessing.

enum class CeregState : uint8_t {
  NOT_REGISTERED = 0,
  REGISTERED_HOME = 1,
  SEARCHING = 2,
  DENIED = 3,
  UNKNOWN = 4,
  REGISTERED_ROAMING = 5,
};

inline bool cieregIsAttached(CeregState s) {
  return s == CeregState::REGISTERED_HOME || s == CeregState::REGISTERED_ROAMING;
}

// "+CEREG: <n>,<stat>[,...]" (unsolicited) or "+CEREG: <stat>" (query reply
// when <n>=0). Only <stat> is needed here, so both shapes are accepted.
bool parseCereg(const char *line, CeregState &out);

// "+CSQ: <rssi>,<ber>". <rssi>=99 means "not known/not detectable".
// Returns false if the line doesn't parse; rssiDbm is only valid when true.
bool parseCsq(const char *line, int &rssiDbm);

// "+QIOPEN: <connectID>,<err>" - the async URC that follows the OK ack of
// AT+QIOPEN. err==0 means the socket opened successfully.
bool parseQiopen(const char *line, int &connectId, int &err);

// "+QIURC: \"recv\",<connectID>,<dataLen>" - notifies that <dataLen> bytes
// are waiting to be pulled with AT+QIRD=<connectID>,<dataLen>.
bool parseQiurcRecv(const char *line, int &connectId, int &dataLen);

// "+QIRD: <len>" - the header line AT+QIRD's response starts with, followed
// by <len> raw bytes on the next line(s). Only the header is parsed here;
// the raw bytes are whatever the caller already captured after this line.
bool parseQirdHeader(const char *line, int &len);

// ---------- MQTT URCs (BC660K-GL MQTT AT command set) ----------
// Syntax below follows Quectel's standard QMT command shape shared across
// the BC66/BC660K/EC-series family - cross-check against the actual
// "BC660K-GL MQTT Application Note" if any of these ever come back
// unparsed, rather than assuming the field layout is wrong.

// "+QMTOPEN: <client_idx>,<result>" - async URC after AT+QMTOPEN's OK ack.
// result==0 means the TCP link to the broker opened successfully.
bool parseQmtopen(const char *line, int &clientIdx, int &result);

// "+QMTCONN: <client_idx>,<result>,<retCode>" - async URC after AT+QMTCONN's
// OK ack. result==0 means the CONNECT packet was sent and a response
// received; retCode==0 means the broker accepted the connection.
bool parseQmtconn(const char *line, int &clientIdx, int &result, int &retCode);

// "+QMTPUB: <client_idx>,<msgId>,<result>[,<value>]" - async URC after
// AT+QMTPUB. result==0 means publish succeeded (value is only meaningful
// for QoS>0, not used here since telemetry publishes at QoS 0).
bool parseQmtpub(const char *line, int &clientIdx, int &msgId, int &result);

// "+QMTSTAT: <client_idx>,<errCode>" - unsolicited, reports the MQTT client
// dropped (broker/network closed it, keepalive timeout, etc). Not tied to
// any in-flight command - always routed through the URC path.
bool parseQmtstat(const char *line, int &clientIdx, int &errCode);

// ---------- Downlink application grammar ----------
// Server replies to an uplink datagram with either:
//   ACK,<seq>
//   ACK,<seq>,CFG,<version>,<key>=<value>,<key>=<value>,...
// This is plain text (unlike the uplink, which is binary - see below): it's
// rare, small, and worth keeping human-debuggable (matches the existing
// nbiot_server.py prototype's `ACK,{seq}` reply and its `nc -u` test recipe).
//
// Parsing here only extracts structure - it does NOT know which keys are
// valid or what range they're allowed to take. That semantic clamping lives
// in ModemNBIoT, using the NBIOT_CFG_*_MIN/MAX constants from Config.h, so a
// bad or unrecognized key/value can never be anything other than ignored.
constexpr uint8_t kMaxDownlinkKv = 4;
constexpr uint8_t kDownlinkKeyLen = 8;
constexpr uint8_t kDownlinkValueLen = 16;

struct DownlinkKv {
  char key[kDownlinkKeyLen] = {0};
  char value[kDownlinkValueLen] = {0};
};

struct DownlinkAck {
  bool valid = false;
  uint16_t seq = 0;
  bool hasCfg = false;
  uint8_t cfgVersion = 0;
  DownlinkKv kv[kMaxDownlinkKv];
  uint8_t kvCount = 0;
};

// text is the raw UDP payload (already NUL-terminated by the caller).
// Returns false only if it doesn't even start with "ACK,<number>" - anything
// beyond that which doesn't fit the CFG grammar is simply left unparsed
// (out.hasCfg stays false) rather than rejecting the whole ACK.
bool parseDownlink(const char *text, DownlinkAck &out);

// ---------- Uplink batch frame (binary) ----------
// Text CSV for a full 13-field reading (temp,hum,pres,gas,pm1,pm25,pm10,co2,
// co,coTemp,windAngle,windSpeed,windValid) plus a timestamp and link-quality
// pair comes to ~75-80 bytes/reading even at 1 decimal place - a default
// batch of 10 would blow well past the 500-byte budget on its own. Binary
// fixed-point fields get one reading down to kBatchRecordSize bytes, which
// is what actually makes "10 readings / 20 minutes" fit.
//
// All multi-byte fields are big-endian ("network order"), written manually
// byte-by-byte rather than memcpy'd from a struct - no reliance on the
// compiler's struct packing/padding or on ESP32 vs. server endianness
// matching by accident.
//
// Frame layout:
//   Header (kBatchHeaderSize = 5 bytes):
//     [0]   magic        0xF0
//     [1:2] seq          uint16 BE, increments once per batch (wraps)
//     [3]   count        uint8, number of records that follow
//     [4]   cfgVersion   uint8, version of the last downlink CFG actually
//                         applied (0 = none yet) - reported every uplink per
//                         the "report the applied config version" requirement
//   Record (kBatchRecordSize = 30 bytes), repeated `count` times:
//     [0:1]   dtSec        uint16 BE, seconds since the OLDEST reading in
//                           this batch (i.e. reading 0 always has dtSec=0)
//     [2:3]   rssi         int16 BE, LoRa RSSI (dBm) at time of reception
//     [4]     snr          int8,  LoRa SNR
//     [5:6]   temp_x10     int16 BE, deg C * 10
//     [7:8]   hum_x10      uint16 BE, %RH * 10
//     [9:10]  pres_x10     uint16 BE, hPa * 10
//     [11:12] gas_x100ohm  uint16 BE, sensor resistance in units of 100 ohm
//     [13:14] pm1_x10      uint16 BE, ug/m3 * 10
//     [15:16] pm25_x10     uint16 BE, ug/m3 * 10
//     [17:18] pm10_x10     uint16 BE, ug/m3 * 10
//     [19:20] co2_ppm      uint16 BE
//     [21:22] co_x100      uint16 BE, ppm * 100
//     [23:24] coTemp_x10   int16 BE, deg C * 10
//     [25:26] windAngle_x10 uint16 BE, deg * 10 (0-3599)
//     [27:28] windSpeed_x10 uint16 BE
//     [29]    flags        uint8, bit0 = windValid, bits 1-7 reserved (0)
//
// NOTE for whoever writes the server-side decoder (nbiot_server.py's current
// FIELDS/parse_payload is a CSV placeholder and does not match this format -
// it needs a binary decoder using this same layout before it can read real
// uplinks).
constexpr uint8_t kBatchMagic = 0xF0;
constexpr size_t kBatchHeaderSize = 5;
constexpr size_t kBatchRecordSize = 30;

static_assert(kBatchHeaderSize + (size_t)NBIOT_CFG_READINGS_MAX * kBatchRecordSize <=
                  NBIOT_PAYLOAD_MAX_BYTES,
              "NBIOT_CFG_READINGS_MAX * kBatchRecordSize + header must fit under "
              "NBIOT_PAYLOAD_MAX_BYTES - lower NBIOT_CFG_READINGS_MAX or shrink the record");

// Appends kBatchHeaderSize bytes to out (caller-owned buffer, must have room).
void encodeBatchHeader(uint16_t seq, uint8_t count, uint8_t cfgVersion, uint8_t *out);

// Appends kBatchRecordSize bytes to out (caller-owned buffer, must have room).
void encodeBatchRecord(const SensorSnapshot &snap, uint16_t dtSec, uint8_t *out);

}  // namespace NbiotProtocol
