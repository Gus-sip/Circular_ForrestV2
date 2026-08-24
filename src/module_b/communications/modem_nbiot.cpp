#include "modem_nbiot.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

ModemNBIoT::ModemNBIoT(HardwareSerial &serial, uint8_t rxPin, uint8_t txPin, uint8_t enPin, uint8_t rstPin,
                        uint32_t baud)
    : _serial(serial), _rxPin(rxPin), _txPin(txPin), _enPin(enPin), _rstPin(rstPin), _baud(baud) {}

void ModemNBIoT::begin() {
  pinMode(_enPin, OUTPUT);
  digitalWrite(_enPin, NBIOT_DISABLE);
  pinMode(_rstPin, OUTPUT);
  digitalWrite(_rstPin, !NBIOT_RST_ACTIVE);  // held de-asserted - not driven by the recovery ladder (yet)
  setState(State::OFF);
}

void ModemNBIoT::tick() {
  pumpSerial();
  checkCommandTimeout();

  // The downlink read is a self-contained mini-transaction that owns the
  // command slot for its duration - handled here, before any state handler
  // runs, so a state's own tick*() never mistakes AT+QIRD's outcome for its
  // own in-flight command's result.
  if (_downlinkReadInFlight) {
    if (_cmd.active) return;
    CmdOutcome outcome = consumeOutcome();
    _downlinkReadInFlight = false;
    if (outcome == CmdOutcome::OK && _cmd.hasInfoLine) applyDownlink(_cmd.infoLine);
    return;
  }
  if (!_cmd.active && _downlinkPending) {
    char cmd[24];
    snprintf(cmd, sizeof(cmd), "AT+QIRD=%d,%d", NBIOT_CONNECT_ID, _downlinkPendingLen);
    issueCommand(cmd, NBIOT_AT_CMD_TIMEOUT_MS);
    _downlinkPending = false;
    _downlinkReadInFlight = true;
    return;
  }

  switch (_state) {
    case State::OFF: tickOff(); break;
    case State::POWERING: tickPowering(); break;
    case State::WAIT_AT: tickWaitAt(); break;
    case State::CONFIG: tickConfig(); break;
    case State::ATTACHING: tickAttaching(); break;
    case State::SOCKET_OPEN: tickSocketOpen(); break;
    case State::IDLE: tickIdle(); break;
    case State::SENDING: tickSending(); break;
    case State::ERROR: tickError(); break;
  }
}

void ModemNBIoT::enqueue(const SensorSnapshot &snap) {
  if (_ringCount == 0) _oldestPendingMs = snap.lastHeardMs ? snap.lastHeardMs : millis();

  if (_ringCount >= NBIOT_RING_CAPACITY) {
    _ringHead = (uint8_t)((_ringHead + 1) % NBIOT_RING_CAPACITY);
    _ringCount--;
    _packetsDropped++;
    _oldestPendingMs = ringPeek(0).lastHeardMs;
  }

  uint8_t idx = (uint8_t)((_ringHead + _ringCount) % NBIOT_RING_CAPACITY);
  _ring[idx] = snap;
  _ringCount++;
}

const SensorSnapshot &ModemNBIoT::ringPeek(uint8_t i) const {
  return _ring[(_ringHead + i) % NBIOT_RING_CAPACITY];
}

void ModemNBIoT::popSentReadings() {
  uint8_t n = _lastSendCount;
  if (n > _ringCount) n = _ringCount;
  _ringHead = (uint8_t)((_ringHead + n) % NBIOT_RING_CAPACITY);
  _ringCount = (uint8_t)(_ringCount - n);
  if (_ringCount > 0) _oldestPendingMs = ringPeek(0).lastHeardMs;
}

// ---------- The one command path ----------

void ModemNBIoT::issueCommand(const char *cmd, uint32_t timeoutMs, CmdKind kind, const uint8_t *sendPayload,
                               size_t sendPayloadLen) {
  _cmd = PendingCmd{};
  _cmd.active = true;
  _cmd.kind = kind;
  _cmd.sentAtMs = millis();
  _cmd.timeoutMs = timeoutMs;
  _cmd.sendPayload = sendPayload;
  _cmd.sendPayloadLen = sendPayloadLen;
  _cmd.awaitingPrompt = (kind == CmdKind::QISEND);
  _serial.print(cmd);
  _serial.print("\r\n");
}

void ModemNBIoT::pumpSerial() {
  while (_serial.available()) {
    char c = (char)_serial.read();

    // BC66-family modules answer AT+QISEND with a bare '>' (no CRLF) asking
    // for the raw payload bytes - caught here, before line buffering, since
    // it isn't a line at all.
    if (_cmd.active && _cmd.kind == CmdKind::QISEND && _cmd.awaitingPrompt && c == '>') {
      _serial.write(_cmd.sendPayload, _cmd.sendPayloadLen);
      _cmd.awaitingPrompt = false;
      _lineLen = 0;
      continue;
    }

    if (c == '\n') {
      if (_lineLen > 0) {
        _lineBuf[_lineLen] = '\0';
        handleLine(_lineBuf, _lineLen);
      }
      _lineLen = 0;
    } else if (c != '\r') {
      if (_lineLen < sizeof(_lineBuf) - 1) {
        _lineBuf[_lineLen++] = c;
      } else {
        _lineLen = 0;  // guard against a garbage/oversized line
      }
    }
  }
}

void ModemNBIoT::handleLine(const char *line, size_t len) {
  if (!_cmd.active) {
    handleUrc(line, len);
    return;
  }

  if (_cmd.kind == CmdKind::QISEND) {
    if (strcmp(line, "SEND OK") == 0) { completeCommand(CmdOutcome::OK); return; }
    if (strcmp(line, "SEND FAIL") == 0 || strncmp(line, "ERROR", 5) == 0) {
      completeCommand(CmdOutcome::ERR);
      return;
    }
    return;  // noise while waiting on the send result (e.g. a stray URC)
  }

  if (strcmp(line, "OK") == 0) {
    if (_cmd.kind == CmdKind::QIOPEN) {
      _cmd.sawOk = true;  // ack received - the real result is the async +QIOPEN URC below
      return;
    }
    completeCommand(CmdOutcome::OK);
    return;
  }
  if (strncmp(line, "ERROR", 5) == 0 || strncmp(line, "+CME ERROR", 10) == 0 ||
      strncmp(line, "+CMS ERROR", 10) == 0) {
    completeCommand(CmdOutcome::ERR);
    return;
  }

  if (_cmd.kind == CmdKind::QIOPEN && _cmd.sawOk) {
    int connectId, err;
    if (NbiotProtocol::parseQiopen(line, connectId, err)) {
      strncpy(_cmd.infoLine, line, sizeof(_cmd.infoLine) - 1);
      _cmd.hasInfoLine = true;
      completeCommand(err == 0 ? CmdOutcome::OK : CmdOutcome::ERR);
      return;
    }
    // Not the +QIOPEN URC yet - could be an unrelated URC (e.g. +QIURC)
    // interleaved while we wait for it. Let the URC path see it instead of
    // swallowing it; this command is still pending either way.
    handleUrc(line, len);
    return;
  }

  // Any other line while a plain command is active is its informational
  // response body (e.g. the "+CSQ: ..." line that precedes OK).
  strncpy(_cmd.infoLine, line, sizeof(_cmd.infoLine) - 1);
  _cmd.hasInfoLine = true;
}

void ModemNBIoT::handleUrc(const char *line, size_t len) {
  (void)len;

  int connectId, dataLen;
  if (NbiotProtocol::parseQiurcRecv(line, connectId, dataLen)) {
    if (connectId == NBIOT_CONNECT_ID && dataLen > 0) {
      _downlinkPending = true;
      _downlinkPendingLen = dataLen;
    }
    return;
  }

  if (strstr(line, "RDY") || strstr(line, "QNBIOTEVENT")) {
    _sawBootUrc = true;
    return;
  }

  Serial.printf("[nbiot] urc: %s\n", line);
}

void ModemNBIoT::completeCommand(CmdOutcome outcome) {
  _cmd.active = false;
  _cmd.outcome = outcome;
}

void ModemNBIoT::checkCommandTimeout() {
  if (_cmd.active && millis() - _cmd.sentAtMs > _cmd.timeoutMs) completeCommand(CmdOutcome::TIMEOUT);
}

ModemNBIoT::CmdOutcome ModemNBIoT::consumeOutcome() {
  CmdOutcome o = _cmd.outcome;
  _cmd.outcome = CmdOutcome::NONE;
  return o;
}

// ---------- Downlink ----------

void ModemNBIoT::applyDownlink(const char *text) {
  NbiotProtocol::DownlinkAck ack;
  if (!NbiotProtocol::parseDownlink(text, ack) || !ack.valid || !ack.hasCfg) return;
  if (ack.cfgVersion == _appliedCfgVersion) return;  // already applied - UDP has no delivery guarantee either way

  uint16_t newReadings = _batchReadingsTarget;
  uint32_t newSeconds = _batchSecondsTarget;

  for (uint8_t i = 0; i < ack.kvCount; i++) {
    const NbiotProtocol::DownlinkKv &kv = ack.kv[i];
    if (strcmp(kv.key, "N") == 0) {
      long v = strtol(kv.value, nullptr, 10);
      if (v >= NBIOT_CFG_READINGS_MIN && v <= NBIOT_CFG_READINGS_MAX) newReadings = (uint16_t)v;
      // out of range -> ignored outright, current value kept
    } else if (strcmp(kv.key, "T") == 0) {
      long v = strtol(kv.value, nullptr, 10);
      if (v >= (long)NBIOT_CFG_SECONDS_MIN && v <= (long)NBIOT_CFG_SECONDS_MAX) newSeconds = (uint32_t)v;
    }
    // unrecognized keys are ignored outright
  }

  _batchReadingsTarget = newReadings;
  _batchSecondsTarget = newSeconds;
  _appliedCfgVersion = ack.cfgVersion;
  if (_configAppliedCb) _configAppliedCb(_appliedCfgVersion, _batchReadingsTarget, _batchSecondsTarget);
}

// ---------- State machine ----------

void ModemNBIoT::setState(State s) {
  _state = s;
  _stateEnteredMs = millis();
  switch (s) {
    case State::ATTACHING:
      _cgpaddrChecked = false;
      _lastCeregPollMs = 0;
      break;
    case State::IDLE:
      _idleSub = IdleSub::NORMAL;
      _idleSinceMs = millis();
      break;
    case State::SENDING:
      _sendInFlight = false;
      break;
    default:
      break;
  }
}

void ModemNBIoT::enterPowering() {
  if (!_uartStarted) {
    _serial.begin(_baud, SERIAL_8N1, _rxPin, _txPin);
    _uartStarted = true;
  }
  digitalWrite(_enPin, NBIOT_EN_ACTIVE);
  _sawBootUrc = false;
  setState(State::POWERING);
}

void ModemNBIoT::enterError(const char *reason) {
  setLastError(reason);
  _consecutiveFailures++;
  _backoffUntilMs = millis() + computeBackoff(_consecutiveFailures);
  setState(State::ERROR);
}

void ModemNBIoT::powerCycle() {
  setLastError("power cycling modem (last resort)");
  digitalWrite(_enPin, NBIOT_DISABLE);
  _attached = false;
  _powerCycleSettleUntilMs = millis() + NBIOT_POWER_OFF_SETTLE_MS;
  setState(State::OFF);
}

void ModemNBIoT::handleFailureAtLevel(RecoveryLevel level, const char *reason) {
  setLastError(reason);
  _consecutiveFailures++;

  switch (level) {
    case RecoveryLevel::SEND:
      _sendRetries++;
      if (_sendRetries <= NBIOT_MAX_SEND_RETRIES) {
        applyBackoffAndSetState(State::SENDING);
        return;
      }
      _sendRetries = 0;
      _socketSub = SocketSub::CLOSE_FIRST;
      applyBackoffAndSetState(State::SOCKET_OPEN);
      return;

    case RecoveryLevel::SOCKET:
      _socketRetries++;
      if (_socketRetries <= NBIOT_MAX_SOCKET_RETRIES) {
        _socketSub = SocketSub::CLOSE_FIRST;
        applyBackoffAndSetState(State::SOCKET_OPEN);
        return;
      }
      _socketRetries = 0;
      applyBackoffAndSetState(State::ATTACHING);
      return;

    case RecoveryLevel::ATTACH:
      _attachRetries++;
      if (_attachRetries <= NBIOT_MAX_ATTACH_RETRIES) {
        applyBackoffAndSetState(State::ATTACHING);
        return;
      }
      _attachRetries = 0;
      powerCycle();
      return;
  }
}

void ModemNBIoT::applyBackoffAndSetState(State s) {
  _backoffUntilMs = millis() + computeBackoff(_consecutiveFailures);
  setState(s);
}

uint32_t ModemNBIoT::computeBackoff(uint32_t attempt) const {
  if (attempt > 10) attempt = 10;  // keep the shift bounded - NBIOT_BACKOFF_MAX_MS caps it anyway
  uint32_t v = (uint32_t)NBIOT_BACKOFF_BASE_MS << attempt;
  if (v > NBIOT_BACKOFF_MAX_MS || v < NBIOT_BACKOFF_BASE_MS) v = NBIOT_BACKOFF_MAX_MS;
  return v;
}

void ModemNBIoT::setLastError(const char *msg) {
  strncpy(_lastError, msg, sizeof(_lastError) - 1);
  _lastError[sizeof(_lastError) - 1] = '\0';
}

const char *ModemNBIoT::stateName() const {
  switch (_state) {
    case State::OFF: return "OFF";
    case State::POWERING: return "POWERING";
    case State::WAIT_AT: return "WAIT_AT";
    case State::CONFIG: return "CONFIG";
    case State::ATTACHING: return "ATTACHING";
    case State::SOCKET_OPEN: return "SOCKET_OPEN";
    case State::IDLE: return "IDLE";
    case State::SENDING: return "SENDING";
    case State::ERROR: return "ERROR";
  }
  return "?";
}

uint32_t ModemNBIoT::attachedUptimeMs() const {
  return _attached ? (millis() - _attachedSinceMs) : 0;
}

void ModemNBIoT::tickOff() {
  if (millis() < _powerCycleSettleUntilMs) return;
  enterPowering();
}

void ModemNBIoT::tickPowering() {
  // Real hardware doesn't answer AT the instant power is applied - it prints
  // its own boot URC first (RDY / +QNBIOTEVENT: "EXIT DEEPSLEEP" seen on the
  // bench during the bc660k_bridge.cpp bring-up). Move on as soon as one
  // shows up, or after a flat timeout either way - WAIT_AT's own AT retries
  // are the real synchronization point, this is just a head start.
  if (_sawBootUrc || millis() - _stateEnteredMs > NBIOT_TIMEOUT_POWERING_MS) {
    _lastAtPingMs = 0;  // force an immediate first AT attempt
    setState(State::WAIT_AT);
  }
}

void ModemNBIoT::tickWaitAt() {
  if (millis() - _stateEnteredMs > NBIOT_TIMEOUT_WAIT_AT_MS) {
    enterError("modem never responded to AT");
    return;
  }
  if (_cmd.active) return;

  if (consumeOutcome() == CmdOutcome::OK) {
    _configStep = ConfigStep::ATE0;
    setState(State::CONFIG);
    return;
  }

  if (millis() - _lastAtPingMs >= NBIOT_AT_PING_GAP_MS) {
    issueCommand("AT", NBIOT_AT_CMD_TIMEOUT_MS);
    _lastAtPingMs = millis();
  }
}

void ModemNBIoT::tickConfig() {
  if (millis() - _stateEnteredMs > NBIOT_TIMEOUT_CONFIG_MS) {
    enterError("CONFIG timed out");
    return;
  }
  if (_cmd.active) return;

  if (_cmd.outcome != CmdOutcome::NONE) {
    if (consumeOutcome() != CmdOutcome::OK) {
      enterError("a CONFIG step returned ERROR/timeout");
      return;
    }
    _configStep = (ConfigStep)((uint8_t)_configStep + 1);
  }

  switch (_configStep) {
    case ConfigStep::ATE0:
      issueCommand("ATE0", NBIOT_AT_CMD_TIMEOUT_MS);
      break;
    case ConfigStep::CPIN:
      issueCommand("AT+CPIN?", NBIOT_AT_CMD_TIMEOUT_MS);
      break;
    case ConfigStep::CGDCONT: {
      char cmd[80];
      snprintf(cmd, sizeof(cmd), "AT+CGDCONT=%d,\"IP\",\"%s\"", NBIOT_CONTEXT_ID, NBIOT_APN);
      issueCommand(cmd, NBIOT_AT_CMD_TIMEOUT_MS);
      break;
    }
    case ConfigStep::CFUN:
      issueCommand("AT+CFUN=1", NBIOT_AT_CMD_TIMEOUT_MS);
      break;
    case ConfigStep::DONE:
      setState(State::ATTACHING);
      break;
  }
}

void ModemNBIoT::tickAttaching() {
  if (millis() < _backoffUntilMs) return;

  if (!_cgpaddrChecked) {
    if (millis() - _stateEnteredMs > NBIOT_TIMEOUT_ATTACH_MS) {
      handleFailureAtLevel(RecoveryLevel::ATTACH, "attach timed out");
      return;
    }
    if (_cmd.active) return;

    if (_cmd.outcome == CmdOutcome::NONE) {
      if (millis() - _lastCeregPollMs >= NBIOT_ATTACH_POLL_INTERVAL_MS) {
        issueCommand("AT+CEREG?", NBIOT_AT_CMD_TIMEOUT_MS);
        _lastCeregPollMs = millis();
      }
      return;
    }

    if (consumeOutcome() != CmdOutcome::OK) {
      handleFailureAtLevel(RecoveryLevel::ATTACH, "AT+CEREG? failed");
      return;
    }

    NbiotProtocol::CeregState st;
    if (!NbiotProtocol::parseCereg(_cmd.infoLine, st) || !NbiotProtocol::cieregIsAttached(st)) {
      _attached = false;
      return;  // not attached yet - keep polling, the timeout above is the backstop
    }

    _attached = true;
    _attachedSinceMs = millis();
    _attachRetries = 0;
    _consecutiveFailures = 0;

    char cmd[24];
    snprintf(cmd, sizeof(cmd), "AT+CGPADDR=%d", NBIOT_CONTEXT_ID);
    issueCommand(cmd, NBIOT_AT_CMD_TIMEOUT_MS);
    _cgpaddrChecked = true;
    return;
  }

  // Waiting on (or just got) the CGPADDR sanity check.
  if (_cmd.active) return;
  if (_cmd.outcome == CmdOutcome::NONE) return;
  consumeOutcome();  // whatever it said, CEREG already confirmed attachment - not worth failing over
  _socketSub = SocketSub::OPEN;
  setState(State::SOCKET_OPEN);
}

void ModemNBIoT::tickSocketOpen() {
  if (millis() < _backoffUntilMs) return;
  if (millis() - _stateEnteredMs > NBIOT_TIMEOUT_SOCKET_MS) {
    handleFailureAtLevel(RecoveryLevel::SOCKET, "socket open timed out");
    return;
  }
  if (_cmd.active) return;

  if (_socketSub == SocketSub::CLOSE_FIRST) {
    if (_cmd.outcome == CmdOutcome::NONE) {
      char cmd[24];
      snprintf(cmd, sizeof(cmd), "AT+QICLOSE=%d", NBIOT_CONNECT_ID);
      issueCommand(cmd, NBIOT_AT_CMD_TIMEOUT_MS);
      return;
    }
    consumeOutcome();  // proceed to reopen regardless of how the close went
    _socketSub = SocketSub::OPEN;
    return;
  }

  // SocketSub::OPEN
  if (_cmd.outcome == CmdOutcome::NONE) {
    char cmd[96];
    snprintf(cmd, sizeof(cmd), "AT+QIOPEN=%d,%d,\"UDP\",\"%s\",%d,%d,0", NBIOT_CONTEXT_ID, NBIOT_CONNECT_ID,
              NBIOT_SERVER_HOST, NBIOT_SERVER_PORT, NBIOT_LOCAL_PORT);
    issueCommand(cmd, NBIOT_TIMEOUT_SOCKET_MS, CmdKind::QIOPEN);
    return;
  }

  if (consumeOutcome() == CmdOutcome::OK) {
    _socketRetries = 0;
    _consecutiveFailures = 0;
    setState(State::IDLE);
    return;
  }
  handleFailureAtLevel(RecoveryLevel::SOCKET, "socket open failed");
}

void ModemNBIoT::tickIdle() {
  if (_cmd.active) return;
  CmdOutcome outcome = consumeOutcome();

  if (_idleSub == IdleSub::WAKING) {
    if (outcome == CmdOutcome::NONE) return;
    // Whether QSCLK=0 came back OK or not, try to send anyway - if the modem
    // really is unresponsive that will surface through SENDING's own
    // failure path instead of stalling here.
    setState(State::SENDING);
    return;
  }

  if (_idleSub == IdleSub::POLLING_CSQ) {
    if (outcome == CmdOutcome::NONE) return;
    if (outcome == CmdOutcome::OK) {
      int dbm;
      if (NbiotProtocol::parseCsq(_cmd.infoLine, dbm)) _rssiDbm = dbm;
    }
    _idleSub = IdleSub::NORMAL;
    _lastCsqPollMs = millis();
    // fall through - still worth checking whether a batch is due this tick
  }

  bool batchDue = _ringCount > 0 && (_ringCount >= _batchReadingsTarget ||
                                      (millis() - _oldestPendingMs) >= _batchSecondsTarget * 1000UL);

  if (batchDue) {
    if (_idleSub == IdleSub::SLEEPING) {
      issueCommand("AT+QSCLK=0", NBIOT_AT_CMD_TIMEOUT_MS);
      _idleSub = IdleSub::WAKING;
    } else {
      setState(State::SENDING);
    }
    return;
  }

  if (_idleSub != IdleSub::NORMAL) return;

  if (millis() - _lastCsqPollMs >= NBIOT_CSQ_POLL_INTERVAL_MS) {
    issueCommand("AT+CSQ", NBIOT_AT_CMD_TIMEOUT_MS);
    _idleSub = IdleSub::POLLING_CSQ;
    return;
  }

  if (millis() - _idleSinceMs >= NBIOT_PSM_IDLE_MS) {
    // Fire-and-forget: PSM is an optimization, not correctness-critical, so
    // an ERROR here (e.g. QSCLK unsupported) just means the modem stays
    // awake - harmless, and the next wake attempt is then a no-op.
    issueCommand("AT+QSCLK=1", NBIOT_AT_CMD_TIMEOUT_MS);
    _idleSub = IdleSub::SLEEPING;
  }
}

void ModemNBIoT::tickSending() {
  if (millis() < _backoffUntilMs) return;
  if (_cmd.active) return;

  if (!_sendInFlight) {
    // Rebuilt fresh on every attempt (including retries) rather than cached,
    // so a reading that arrives mid-retry is naturally included and nothing
    // here holds a stale pointer across a socket reopen or re-attach.
    _sendPayloadLen = buildBatchPayload(_sendPayload, sizeof(_sendPayload));
    char header[24];
    snprintf(header, sizeof(header), "AT+QISEND=%d,%u", NBIOT_CONNECT_ID, (unsigned)_sendPayloadLen);
    issueCommand(header, NBIOT_TIMEOUT_SEND_MS, CmdKind::QISEND, _sendPayload, _sendPayloadLen);
    _sendInFlight = true;
    return;
  }

  CmdOutcome outcome = consumeOutcome();
  if (outcome == CmdOutcome::NONE) return;

  _sendInFlight = false;
  if (outcome == CmdOutcome::OK) {
    onSendSucceeded();
  } else {
    onSendFailed();
  }
}

void ModemNBIoT::tickError() {
  if (millis() < _backoffUntilMs) return;
  setState(State::OFF);  // backoff already served here - OFF proceeds to POWERING immediately
}

void ModemNBIoT::onSendSucceeded() {
  _packetsSent++;
  _consecutiveFailures = 0;
  _sendRetries = 0;
  popSentReadings();
  _uplinkSeq++;
  setState(State::IDLE);
}

void ModemNBIoT::onSendFailed() {
  _packetsFailed++;
  handleFailureAtLevel(RecoveryLevel::SEND, "send failed");
}

size_t ModemNBIoT::buildBatchPayload(uint8_t *out, size_t cap) {
  uint8_t count = _ringCount;
  if (count > _batchReadingsTarget) count = (uint8_t)_batchReadingsTarget;
  if (count > NBIOT_CFG_READINGS_MAX) count = NBIOT_CFG_READINGS_MAX;

  size_t need = NbiotProtocol::kBatchHeaderSize + (size_t)count * NbiotProtocol::kBatchRecordSize;
  if (need > cap) count = 0;  // shouldn't happen given the ceiling above, but never overflow the caller's buffer

  NbiotProtocol::encodeBatchHeader(_uplinkSeq, count, _appliedCfgVersion, out);
  size_t offset = NbiotProtocol::kBatchHeaderSize;
  for (uint8_t i = 0; i < count; i++) {
    const SensorSnapshot &snap = ringPeek(i);
    uint32_t dtMs = snap.lastHeardMs - _oldestPendingMs;
    uint16_t dtSec = (uint16_t)(dtMs / 1000 > 65535 ? 65535 : dtMs / 1000);
    NbiotProtocol::encodeBatchRecord(snap, dtSec, out + offset);
    offset += NbiotProtocol::kBatchRecordSize;
  }
  _lastSendCount = count;
  return offset;
}
