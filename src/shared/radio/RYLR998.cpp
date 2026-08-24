#include "RYLR998.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

bool RYLR998::begin(uint16_t addr, uint16_t networkId, uint32_t bandHz, const RYLR998Params &params,
                     Print *debug) {
  _serial.begin(_baud, SERIAL_8N1, _rxPin, _txPin);

  char cmd[48];

  if (!sendATCommand("AT", debug)) return false;

  snprintf(cmd, sizeof(cmd), "AT+ADDRESS=%u", addr);
  if (!sendATCommand(cmd, debug)) return false;

  snprintf(cmd, sizeof(cmd), "AT+NETWORKID=%u", networkId);
  if (!sendATCommand(cmd, debug)) return false;

  snprintf(cmd, sizeof(cmd), "AT+BAND=%lu", (unsigned long)bandHz);
  if (!sendATCommand(cmd, debug)) return false;

  snprintf(cmd, sizeof(cmd), "AT+PARAMETER=%u,%u,%u,%u", params.spreadingFactor, params.bandwidth,
           params.codingRate, params.preamble);
  if (!sendATCommand(cmd, debug)) return false;

  return true;
}

bool RYLR998::send(uint16_t destAddr, const char *data, uint8_t len) {
  char header[24];
  snprintf(header, sizeof(header), "AT+SEND=%u,%u,", destAddr, len);
  _serial.print(header);
  _serial.write(reinterpret_cast<const uint8_t *>(data), len);
  _serial.print("\r\n");
  return waitForOK(kSendTimeoutMs, nullptr);
}

bool RYLR998::sendATCommand(const char *cmd, Print *debug, uint32_t timeoutMs) {
  if (debug) {
    debug->print("  -> ");
    debug->println(cmd);
  }
  _serial.print(cmd);
  _serial.print("\r\n");
  bool ok = waitForOK(timeoutMs, debug);
  if (debug && !ok) debug->println("     (no +OK - check wiring/baud or that value is accepted)");
  return ok;
}

bool RYLR998::waitForOK(uint32_t timeoutMs, Print *debug) {
  uint32_t start = millis();
  char line[kLineMax + 1];
  uint16_t lineLen = 0;

  while (millis() - start < timeoutMs) {
    while (_serial.available()) {
      char c = _serial.read();

      if (c == '\n') {
        if (lineLen > 0) {
          line[lineLen] = '\0';
          if (debug) {
            debug->print("  <- ");
            debug->println(line);
          }
          if (strncmp(line, "+OK", 3) == 0) return true;
          if (strncmp(line, "+ERR", 4) == 0) return false;
          // Could be an inbound +RCV= that raced this command's own ack - stash
          // it instead of silently swallowing a real packet.
          if (!_hasPending && tryParseLine(line, lineLen, _pending)) _hasPending = true;
        }
        lineLen = 0;
      } else if (c != '\r') {
        if (lineLen < kLineMax) {
          line[lineLen++] = c;
        } else {
          lineLen = 0;  // guard against garbage
        }
      }
    }
  }
  return false;
}

bool RYLR998::poll(LoRaMessage &outMsg) {
  if (_hasPending) {
    outMsg = _pending;
    _hasPending = false;
    return true;
  }

  while (_serial.available()) {
    char c = _serial.read();

    if (c == '\n') {
      bool matched = false;
      if (_lineLen > 0) {
        _line[_lineLen] = '\0';
        matched = tryParseLine(_line, _lineLen, outMsg);
      }
      _lineLen = 0;
      if (matched) return true;
    } else if (c != '\r') {
      if (_lineLen < kLineMax) {
        _line[_lineLen++] = c;
      } else {
        _lineLen = 0;
      }
    }
  }
  return false;
}

// Parses "+RCV=<addr>,<len>,<data>,<rssi>,<snr>". Length-bounded rather than a
// naive comma split, so payload bytes that happen to contain a comma don't
// desync the fields after them. A newline byte inside the payload would still
// break this (the line-buffering in poll()/waitForOK() splits on '\n' before
// this function ever sees the line) - neither Module C's outbound telemetry
// nor an inbound CFG/ACK command is expected to contain one, but it's not
// defended against.
bool RYLR998::tryParseLine(char *line, uint16_t len, LoRaMessage &outMsg) {
  if (len < 7 || memcmp(line, "+RCV=", 5) != 0) return false;

  char *p = line + 5;
  char *comma1 = strchr(p, ',');
  if (!comma1) return false;
  *comma1 = '\0';
  int addr = atoi(p);

  char *lenStart = comma1 + 1;
  char *comma2 = strchr(lenStart, ',');
  if (!comma2) return false;
  *comma2 = '\0';
  int dataLen = atoi(lenStart);
  if (dataLen < 0 || dataLen > (int)(sizeof(outMsg.payload) - 1)) return false;

  char *dataStart = comma2 + 1;
  if ((dataStart - line) + dataLen + 1 > len) return false;
  if (dataStart[dataLen] != ',') return false;

  char *rssiStart = dataStart + dataLen + 1;
  char *comma3 = strchr(rssiStart, ',');
  if (!comma3) return false;
  *comma3 = '\0';
  int rssi = atoi(rssiStart);

  char *snrStart = comma3 + 1;
  int snr = atoi(snrStart);

  memcpy(outMsg.payload, dataStart, dataLen);
  outMsg.payload[dataLen] = '\0';
  outMsg.length = (uint8_t)dataLen;
  outMsg.senderAddr = (uint16_t)addr;
  outMsg.rssi = (int16_t)rssi;
  outMsg.snr = (int8_t)snr;
  return true;
}
