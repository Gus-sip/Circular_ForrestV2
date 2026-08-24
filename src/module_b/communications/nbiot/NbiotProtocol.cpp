#include "NbiotProtocol.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

namespace NbiotProtocol {

namespace {

// Returns a pointer just past the Nth comma (0-indexed count) found starting
// at p, or nullptr if there aren't that many. Doesn't mutate the string -
// strtol() naturally stops at the next non-numeric character, so callers
// never need a NUL-terminated sub-string to read a numeric field out of one.
const char *skipCommas(const char *p, int count) {
  for (int i = 0; i < count; i++) {
    p = strchr(p, ',');
    if (!p) return nullptr;
    p++;
  }
  return p;
}

void putU16BE(uint8_t *out, uint16_t v) {
  out[0] = (uint8_t)(v >> 8);
  out[1] = (uint8_t)(v & 0xFF);
}

int16_t clampToInt16(long v) {
  if (v > 32767) return 32767;
  if (v < -32768) return -32768;
  return (int16_t)v;
}

uint16_t clampToUint16(long v) {
  if (v < 0) return 0;
  if (v > 65535) return 65535;
  return (uint16_t)v;
}

}  // namespace

bool parseCereg(const char *line, CeregState &out) {
  if (strncmp(line, "+CEREG:", 7) != 0) return false;
  const char *p = line + 7;
  while (*p == ' ') p++;
  // Only the "AT+CEREG?" query-reply shape "+CEREG: <n>,<stat>[,tac,ci,act]"
  // is handled - ModemNBIoT always polls rather than enabling unsolicited
  // +CEREG URCs, so that's the only shape it ever needs to read. (The
  // unsolicited URC shape is ambiguous without that context: with +CEREG=2
  // its first field IS <stat>, not <n> - ambiguity queries don't have.)
  const char *statField = skipCommas(p, 1);
  if (!statField) return false;
  long stat = strtol(statField, nullptr, 10);
  if (stat < 0 || stat > 5) return false;
  out = static_cast<CeregState>(stat);
  return true;
}

bool parseCsq(const char *line, int &rssiDbm) {
  if (strncmp(line, "+CSQ:", 5) != 0) return false;
  const char *p = line + 5;
  while (*p == ' ') p++;
  long raw = strtol(p, nullptr, 10);
  if (raw < 0 || raw > 31) return false;  // 99 (and anything else out of range) = not known
  rssiDbm = (int)(-113 + raw * 2);
  return true;
}

bool parseQiopen(const char *line, int &connectId, int &err) {
  if (strncmp(line, "+QIOPEN:", 8) != 0) return false;
  const char *p = line + 8;
  while (*p == ' ') p++;
  const char *errField = skipCommas(p, 1);
  if (!errField) return false;
  connectId = (int)strtol(p, nullptr, 10);
  err = (int)strtol(errField, nullptr, 10);
  return true;
}

bool parseQiurcRecv(const char *line, int &connectId, int &dataLen) {
  if (strncmp(line, "+QIURC:", 7) != 0) return false;
  const char *quoted = strstr(line, "\"recv\"");
  if (!quoted) return false;
  const char *idField = skipCommas(quoted + 6, 1);
  if (!idField) return false;
  const char *lenField = skipCommas(idField, 1);
  if (!lenField) return false;
  connectId = (int)strtol(idField, nullptr, 10);
  dataLen = (int)strtol(lenField, nullptr, 10);
  return true;
}

bool parseQirdHeader(const char *line, int &len) {
  if (strncmp(line, "+QIRD:", 6) != 0) return false;
  const char *p = line + 6;
  while (*p == ' ') p++;
  len = (int)strtol(p, nullptr, 10);
  return true;
}

bool parseDownlink(const char *text, DownlinkAck &out) {
  out = DownlinkAck{};
  if (strncmp(text, "ACK,", 4) != 0) return false;

  const char *p = text + 4;
  char *end = nullptr;
  long seq = strtol(p, &end, 10);
  if (end == p || seq < 0) return false;
  out.seq = (uint16_t)seq;
  out.valid = true;

  p = end;
  if (*p != ',') return true;  // bare "ACK,<seq>" - valid, no CFG attached
  p++;
  if (strncmp(p, "CFG,", 4) != 0) return true;  // unrecognized suffix - ACK itself still stands
  p += 4;

  long version = strtol(p, &end, 10);
  if (end == p || version < 0) return true;  // malformed CFG header - ignore just the CFG part
  out.cfgVersion = (uint8_t)version;
  out.hasCfg = true;
  p = end;

  while (*p == ',' && out.kvCount < kMaxDownlinkKv) {
    p++;
    const char *eq = strchr(p, '=');
    if (!eq) break;
    const char *nextComma = strchr(eq, ',');
    size_t keyLen = (size_t)(eq - p);
    size_t valLen = nextComma ? (size_t)(nextComma - eq - 1) : strlen(eq + 1);

    if (keyLen > 0 && keyLen < kDownlinkKeyLen && valLen > 0 && valLen < kDownlinkValueLen) {
      DownlinkKv &slot = out.kv[out.kvCount++];
      memcpy(slot.key, p, keyLen);
      slot.key[keyLen] = '\0';
      memcpy(slot.value, eq + 1, valLen);
      slot.value[valLen] = '\0';
    }
    // A malformed/oversized token just doesn't get stored - the rest of the
    // line is still worth parsing, so fall through to advance p either way.
    p = nextComma ? nextComma : eq + 1 + valLen;
  }
  return true;
}

void encodeBatchHeader(uint16_t seq, uint8_t count, uint8_t cfgVersion, uint8_t *out) {
  out[0] = kBatchMagic;
  putU16BE(out + 1, seq);
  out[3] = count;
  out[4] = cfgVersion;
}

void encodeBatchRecord(const SensorSnapshot &snap, uint16_t dtSec, uint8_t *out) {
  putU16BE(out + 0, dtSec);
  putU16BE(out + 2, (uint16_t)clampToInt16((long)snap.rssi));
  out[4] = (uint8_t)(int8_t)snap.snr;
  putU16BE(out + 5, (uint16_t)clampToInt16(lroundf(snap.temp * 10.0f)));
  putU16BE(out + 7, clampToUint16(lroundf(snap.hum * 10.0f)));
  putU16BE(out + 9, clampToUint16(lroundf(snap.pres * 10.0f)));
  putU16BE(out + 11, clampToUint16(lroundf(snap.gas / 100.0f)));
  putU16BE(out + 13, clampToUint16(lroundf(snap.pm1 * 10.0f)));
  putU16BE(out + 15, clampToUint16(lroundf(snap.pm25 * 10.0f)));
  putU16BE(out + 17, clampToUint16(lroundf(snap.pm10 * 10.0f)));
  putU16BE(out + 19, clampToUint16(lroundf(snap.co2)));
  putU16BE(out + 21, clampToUint16(lroundf(snap.co * 100.0f)));
  putU16BE(out + 23, (uint16_t)clampToInt16(lroundf(snap.coTemp * 10.0f)));
  putU16BE(out + 25, clampToUint16(lroundf(snap.windAngle * 10.0f)));
  putU16BE(out + 27, clampToUint16(lroundf(snap.windSpeed * 10.0f)));
  out[29] = snap.windValid ? 0x01 : 0x00;
}

}  // namespace NbiotProtocol
