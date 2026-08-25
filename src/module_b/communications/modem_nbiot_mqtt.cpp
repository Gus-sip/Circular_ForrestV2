#include "modem_nbiot_mqtt.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

ModemNBIoTMqtt::ModemNBIoTMqtt(HardwareSerial &serial, uint8_t rxPin, uint8_t txPin, uint8_t enPin,
                                uint8_t channelPin, uint8_t pwrkeyPin, uint32_t baud)
    : _serial(serial),
      _rxPin(rxPin),
      _txPin(txPin),
      _enPin(enPin),
      _channelPin(channelPin),
      _pwrkeyPin(pwrkeyPin),
      _baud(baud) {}

void ModemNBIoTMqtt::begin() {
  pinMode(_enPin, OUTPUT);
  digitalWrite(_enPin, NBIOT_DISABLE);
  pinMode(_channelPin, OUTPUT);
  digitalWrite(_channelPin, !NBIOT_CHANNEL_ACTIVE);
  pinMode(_pwrkeyPin, OUTPUT);
  digitalWrite(_pwrkeyPin, !NBIOT_PWRKEY_ACTIVE);  // idle until POWERING pulses it
  setState(State::OFF);
}

void ModemNBIoTMqtt::tick() {
  pumpSerial();
  checkCommandTimeout();

  switch (_state) {
    case State::OFF: tickOff(); break;
    case State::POWERING: tickPowering(); break;
    case State::WAIT_AT: tickWaitAt(); break;
    case State::CONFIG: tickConfig(); break;
    case State::ATTACHING: tickAttaching(); break;
    case State::MQTT_CONNECT: tickMqttConnect(); break;
    case State::IDLE: tickIdle(); break;
    case State::PUBLISHING: tickPublishing(); break;
    case State::ERROR: tickError(); break;
  }
}

void ModemNBIoTMqtt::enqueue(const SensorSnapshot &snap) {
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

const SensorSnapshot &ModemNBIoTMqtt::ringPeek(uint8_t i) const {
  return _ring[(_ringHead + i) % NBIOT_RING_CAPACITY];
}

void ModemNBIoTMqtt::popSentReadings() {
  uint8_t n = _lastPublishCount;
  if (n > _ringCount) n = _ringCount;
  _ringHead = (uint8_t)((_ringHead + n) % NBIOT_RING_CAPACITY);
  _ringCount = (uint8_t)(_ringCount - n);
  if (_ringCount > 0) _oldestPendingMs = ringPeek(0).lastHeardMs;
}

// ---------- The one command path ----------

void ModemNBIoTMqtt::issueCommand(const char *cmd, uint32_t timeoutMs, CmdKind kind, const uint8_t *sendPayload,
                                   size_t sendPayloadLen) {
  _cmd = PendingCmd{};
  _cmd.active = true;
  _cmd.kind = kind;
  _cmd.sentAtMs = millis();
  _cmd.timeoutMs = timeoutMs;
  _cmd.sendPayload = sendPayload;
  _cmd.sendPayloadLen = sendPayloadLen;
  _cmd.awaitingPrompt = (kind == CmdKind::QMTPUB);
  Serial.print("[nbiot-mqtt] -> ");
  Serial.println(cmd);
  _serial.print(cmd);
  _serial.print("\r\n");
}

void ModemNBIoTMqtt::pumpSerial() {
  while (_serial.available()) {
    char c = (char)_serial.read();

    // AT+QMTPUB answers with a bare '>' (no CRLF) asking for the raw JSON
    // payload - caught here, before line buffering, since it isn't a line
    // at all. Terminated with Ctrl+Z (0x1A), not a length header the way
    // AT+QISEND was - see modem_nbiot_mqtt.h for why (sidesteps escaping
    // JSON's own quotes into the command line entirely).
    if (_cmd.active && _cmd.kind == CmdKind::QMTPUB && _cmd.awaitingPrompt && c == '>') {
      Serial.print("[nbiot-mqtt] payload (");
      Serial.print((unsigned)_cmd.sendPayloadLen);
      Serial.print(" bytes): ");
      Serial.write(_cmd.sendPayload, _cmd.sendPayloadLen);
      Serial.println();
      _serial.write(_cmd.sendPayload, _cmd.sendPayloadLen);
      _serial.write((uint8_t)0x1A);
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

void ModemNBIoTMqtt::handleLine(const char *line, size_t len) {
  if (!_cmd.active) {
    handleUrc(line, len);
    return;
  }
  Serial.print("[nbiot-mqtt] <- ");
  Serial.println(line);

  if (strcmp(line, "OK") == 0) {
    if (_cmd.kind == CmdKind::QMTOPEN || _cmd.kind == CmdKind::QMTCONN || _cmd.kind == CmdKind::QMTPUB) {
      _cmd.sawOk = true;  // ack received - the real result is the async URC below
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

  if (_cmd.kind == CmdKind::QMTOPEN && _cmd.sawOk) {
    int idx, result;
    if (NbiotProtocol::parseQmtopen(line, idx, result)) {
      completeCommand(result == 0 ? CmdOutcome::OK : CmdOutcome::ERR);
      return;
    }
    handleUrc(line, len);  // interleaved unrelated URC - this command is still pending
    return;
  }
  if (_cmd.kind == CmdKind::QMTCONN && _cmd.sawOk) {
    int idx, result, retCode;
    if (NbiotProtocol::parseQmtconn(line, idx, result, retCode)) {
      completeCommand((result == 0 && retCode == 0) ? CmdOutcome::OK : CmdOutcome::ERR);
      return;
    }
    handleUrc(line, len);
    return;
  }
  if (_cmd.kind == CmdKind::QMTPUB && _cmd.sawOk) {
    int idx, msgId, result;
    if (NbiotProtocol::parseQmtpub(line, idx, msgId, result)) {
      completeCommand(result == 0 ? CmdOutcome::OK : CmdOutcome::ERR);
      return;
    }
    handleUrc(line, len);
    return;
  }

  // Any other line while a plain command is active is its informational
  // response body (e.g. "+CPIN: READY" or "+CEREG: 1,5" ahead of OK).
  strncpy(_cmd.infoLine, line, sizeof(_cmd.infoLine) - 1);
  _cmd.hasInfoLine = true;
}

void ModemNBIoTMqtt::handleUrc(const char *line, size_t len) {
  (void)len;

  int idx, errCode;
  if (NbiotProtocol::parseQmtstat(line, idx, errCode)) {
    if (idx == MQTT_CLIENT_IDX) {
      _mqttConnected = false;
      Serial.printf("[nbiot-mqtt] +QMTSTAT fired (err=%d) - session marked dead, reconnecting on next publish\n",
                    errCode);
    }
    return;
  }

  if (strstr(line, "RDY") || strstr(line, "QNBIOTEVENT")) {
    _sawBootUrc = true;
    return;
  }

  Serial.printf("[nbiot-mqtt] urc: %s\n", line);
}

void ModemNBIoTMqtt::completeCommand(CmdOutcome outcome) {
  _cmd.active = false;
  _cmd.outcome = outcome;
}

void ModemNBIoTMqtt::checkCommandTimeout() {
  if (_cmd.active && millis() - _cmd.sentAtMs > _cmd.timeoutMs) completeCommand(CmdOutcome::TIMEOUT);
}

ModemNBIoTMqtt::CmdOutcome ModemNBIoTMqtt::consumeOutcome() {
  CmdOutcome o = _cmd.outcome;
  _cmd.outcome = CmdOutcome::NONE;
  return o;
}

// ---------- State machine ----------

void ModemNBIoTMqtt::setState(State s) {
  _state = s;
  _stateEnteredMs = millis();
  switch (s) {
    case State::ATTACHING:
      _cgpaddrChecked = false;
      _lastCeregPollMs = 0;
      break;
    case State::IDLE:
      _lastCsqPollMs = 0;  // poll once shortly after entering IDLE
      break;
    case State::PUBLISHING:
      _publishInFlight = false;
      break;
    default:
      break;
  }
}

void ModemNBIoTMqtt::enterPowering() {
  if (!_uartStarted) {
    _serial.begin(_baud, SERIAL_8N1, _rxPin, _txPin);
    _uartStarted = true;
  }
  digitalWrite(_channelPin, NBIOT_CHANNEL_ACTIVE);
  digitalWrite(_enPin, NBIOT_EN_ACTIVE);
  _sawBootUrc = false;
  _pwrkeyReleased = false;
  setState(State::POWERING);
}

void ModemNBIoTMqtt::enterError(const char *reason) {
  setLastError(reason);
  _consecutiveFailures++;
  _backoffUntilMs = millis() + computeBackoff(_consecutiveFailures);
  setState(State::ERROR);
}

void ModemNBIoTMqtt::powerCycle() {
  setLastError("power cycling modem (last resort)");
  digitalWrite(_enPin, NBIOT_DISABLE);
  digitalWrite(_channelPin, !NBIOT_CHANNEL_ACTIVE);
  digitalWrite(_pwrkeyPin, !NBIOT_PWRKEY_ACTIVE);
  _attached = false;
  _mqttConnected = false;
  _powerCycleSettleUntilMs = millis() + NBIOT_POWER_OFF_SETTLE_MS;
  setState(State::OFF);
}

void ModemNBIoTMqtt::handleFailureAtLevel(RecoveryLevel level, const char *reason) {
  setLastError(reason);
  _consecutiveFailures++;

  switch (level) {
    case RecoveryLevel::PUBLISH:
      _publishRetries++;
      if (_publishRetries <= NBIOT_MAX_SEND_RETRIES) {
        applyBackoffAndSetState(State::PUBLISHING);
        return;
      }
      _publishRetries = 0;
      _mqttConnected = false;  // force a fresh QMTOPEN/QMTCONN, not just another publish attempt
      _mqttConnectSub = MqttConnectSub::CLOSE_FIRST;
      applyBackoffAndSetState(State::MQTT_CONNECT);
      return;

    case RecoveryLevel::MQTT_CONNECT:
      _mqttConnectRetries++;
      if (_mqttConnectRetries <= NBIOT_MAX_SOCKET_RETRIES) {
        _mqttConnectSub = MqttConnectSub::CLOSE_FIRST;
        applyBackoffAndSetState(State::MQTT_CONNECT);
        return;
      }
      _mqttConnectRetries = 0;
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

void ModemNBIoTMqtt::applyBackoffAndSetState(State s) {
  _backoffUntilMs = millis() + computeBackoff(_consecutiveFailures);
  setState(s);
}

uint32_t ModemNBIoTMqtt::computeBackoff(uint32_t attempt) const {
  if (attempt > 10) attempt = 10;
  uint32_t v = (uint32_t)NBIOT_BACKOFF_BASE_MS << attempt;
  if (v > NBIOT_BACKOFF_MAX_MS || v < NBIOT_BACKOFF_BASE_MS) v = NBIOT_BACKOFF_MAX_MS;
  return v;
}

void ModemNBIoTMqtt::setLastError(const char *msg) {
  strncpy(_lastError, msg, sizeof(_lastError) - 1);
  _lastError[sizeof(_lastError) - 1] = '\0';
}

const char *ModemNBIoTMqtt::stateName() const {
  switch (_state) {
    case State::OFF: return "OFF";
    case State::POWERING: return "POWERING";
    case State::WAIT_AT: return "WAIT_AT";
    case State::CONFIG: return "CONFIG";
    case State::ATTACHING: return "ATTACHING";
    case State::MQTT_CONNECT: return "MQTT_CONNECT";
    case State::IDLE: return "IDLE";
    case State::PUBLISHING: return "PUBLISHING";
    case State::ERROR: return "ERROR";
  }
  return "?";
}

uint32_t ModemNBIoTMqtt::attachedUptimeMs() const {
  return _attached ? (millis() - _attachedSinceMs) : 0;
}

void ModemNBIoTMqtt::tickOff() {
  if (millis() < _powerCycleSettleUntilMs) return;
  enterPowering();
}

void ModemNBIoTMqtt::tickPowering() {
  // Non-blocking PWRKEY pulse: settle after VIN/channel go live, pulse
  // PWRKEY low for NBIOT_PWRKEY_PULSE_MS, release, then wait for a boot URC
  // or a flat timeout before moving on - WAIT_AT's own AT retries are the
  // real synchronization point, same philosophy as the original class, this
  // is just a head start. Hardware-confirmed sequence, see modem_nbiot_mqtt.h.
  static const uint32_t kPowerSettleMs = 300;
  uint32_t elapsed = millis() - _stateEnteredMs;

  if (elapsed < kPowerSettleMs) return;

  if (elapsed < kPowerSettleMs + NBIOT_PWRKEY_PULSE_MS) {
    digitalWrite(_pwrkeyPin, NBIOT_PWRKEY_ACTIVE);
    return;
  }

  if (!_pwrkeyReleased) {
    digitalWrite(_pwrkeyPin, !NBIOT_PWRKEY_ACTIVE);
    _pwrkeyReleased = true;
  }

  if (_sawBootUrc || elapsed > NBIOT_TIMEOUT_POWERING_MS) {
    _lastAtPingMs = 0;
    setState(State::WAIT_AT);
  }
}

void ModemNBIoTMqtt::tickWaitAt() {
  if (millis() - _stateEnteredMs > NBIOT_TIMEOUT_WAIT_AT_MS) {
    enterError("modem never responded to AT");
    return;
  }
  if (_cmd.active) return;

  if (consumeOutcome() == CmdOutcome::OK) {
    _configStep = ConfigStep::ATE0;
    _configStepEnteredMs = millis();
    setState(State::CONFIG);
    return;
  }

  if (millis() - _lastAtPingMs >= NBIOT_AT_PING_GAP_MS) {
    issueCommand("AT", NBIOT_AT_CMD_TIMEOUT_MS);
    _lastAtPingMs = millis();
  }
}

void ModemNBIoTMqtt::tickConfig() {
  if (millis() - _stateEnteredMs > NBIOT_TIMEOUT_CONFIG_MS) {
    enterError("CONFIG timed out");
    return;
  }
  if (_cmd.active) return;

  if (_cmd.outcome != CmdOutcome::NONE) {
    CmdOutcome outcome = consumeOutcome();
    // QSCLK is best-effort (see its case below) - every other step is
    // required, an ERROR/timeout there is fatal.
    if (outcome != CmdOutcome::OK && _configStep != ConfigStep::QSCLK) {
      enterError("a CONFIG step returned ERROR/timeout");
      return;
    }
    _configStep = (ConfigStep)((uint8_t)_configStep + 1);
    _configStepEnteredMs = millis();
  }

  switch (_configStep) {
    case ConfigStep::ATE0:
      issueCommand("ATE0", NBIOT_AT_CMD_TIMEOUT_MS);
      break;
    case ConfigStep::CMEE:
      issueCommand("AT+CMEE=2", NBIOT_AT_CMD_TIMEOUT_MS);  // verbose errors - "ue not power on" etc, not bare ERROR
      break;
    case ConfigStep::QSCLK:
      // BENCH SETTING, NOT A DESIGN DECISION - the hub runs off solar +
      // supercapacitors and a modem that never sleeps is real, ongoing
      // power draw. This disables sleep permanently only because getting a
      // clean bring-up read on the QMTPUB path mattered more than power
      // budget today. The real design is: leave sleep enabled, wake the
      // modem explicitly before publishing, confirm it's actually awake,
      // then publish - that's a separate iteration, not done here.
      //
      // Without this, the modem can autonomously enter deep sleep mid-
      // command (confirmed 2026-08-19: an AT+QMTPUB sent while it was
      // asleep never got a '>' prompt at all - just a
      // "+QNBIOTEVENT: EXIT DEEPSLEEP" once it later woke on its own, with
      // the publish itself lost, no error/timeout). Best-effort - if
      // unsupported, proceed anyway rather than blocking bring-up on it.
      issueCommand("AT+QSCLK=0", NBIOT_AT_CMD_TIMEOUT_MS);
      break;
    case ConfigStep::CFUN_ON_1:
      // MUST come before AT+CPIN? - this module does not auto-enable its
      // radio/protocol stack on boot (confirmed 2026-08-19: AT+CPIN? fails
      // "+CME ERROR: ue not power on" without this first).
      issueCommand("AT+CFUN=1", NBIOT_AT_CMD_TIMEOUT_MS);
      break;
    case ConfigStep::CPIN:
      issueCommand("AT+CPIN?", NBIOT_AT_CMD_TIMEOUT_MS);
      break;
    case ConfigStep::CFUN_OFF:
      issueCommand("AT+CFUN=0", NBIOT_AT_CMD_TIMEOUT_MS);
      break;
    case ConfigStep::QCGDEFCONT: {
      // Let CFUN=0's SIM-interface transition settle first (it briefly
      // reports NOT READY) - confirmed 2026-08-19 this needs ~3s or
      // AT+QCGDEFCONT races it. Non-blocking: just re-polled until due.
      if (millis() - _configStepEnteredMs < 3000) return;
      char cmd[64];
      snprintf(cmd, sizeof(cmd), "AT+QCGDEFCONT=\"IP\",\"%s\"", NBIOT_APN);
      issueCommand(cmd, NBIOT_AT_CMD_TIMEOUT_MS);
      break;
    }
    case ConfigStep::CFUN_ON_2:
      issueCommand("AT+CFUN=1", NBIOT_AT_CMD_TIMEOUT_MS);
      break;
    case ConfigStep::DONE:
      setState(State::ATTACHING);
      break;
  }
}

void ModemNBIoTMqtt::tickAttaching() {
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

  if (_cmd.active) return;
  if (_cmd.outcome == CmdOutcome::NONE) return;
  consumeOutcome();  // whatever it said, CEREG already confirmed attachment - not worth failing over
  _mqttConnectSub = MqttConnectSub::KEEPALIVE_CFG;
  setState(State::MQTT_CONNECT);
}

void ModemNBIoTMqtt::tickMqttConnect() {
  if (millis() < _backoffUntilMs) return;
  if (millis() - _stateEnteredMs > NBIOT_TIMEOUT_SOCKET_MS) {
    handleFailureAtLevel(RecoveryLevel::MQTT_CONNECT, "MQTT connect timed out");
    return;
  }
  if (_cmd.active) return;

  switch (_mqttConnectSub) {
    case MqttConnectSub::CLOSE_FIRST: {
      // Defensive close before every (re)connect attempt - MQTT client
      // state lives on the modem, not the ESP32, and survives an ESP32
      // reflash/reset. A leftover session from an earlier attempt left
      // client_idx 0 stuck; AT+QMTOPEN then failed outright with
      // "+CME ERROR: operation not allowed" until this was added
      // (2026-08-19) - same class of bug this project already hit once
      // with AT+QICLOSE/QIOPEN on the plain-socket path. Best-effort: an
      // ERROR here just means there was nothing to close.
      if (_cmd.outcome != CmdOutcome::NONE) {
        consumeOutcome();
        _mqttConnectSub = MqttConnectSub::KEEPALIVE_CFG;
        return;
      }
      char cmd[24];
      snprintf(cmd, sizeof(cmd), "AT+QMTCLOSE=%d", MQTT_CLIENT_IDX);
      issueCommand(cmd, NBIOT_AT_CMD_TIMEOUT_MS);
      return;
    }

    case MqttConnectSub::KEEPALIVE_CFG: {
      if (_cmd.outcome != CmdOutcome::NONE) {
        consumeOutcome();  // best-effort - proceed to OPEN even if QMTCFG was rejected
        _mqttConnectSub = MqttConnectSub::OPEN;
        return;
      }
      char cmd[48];
      snprintf(cmd, sizeof(cmd), "AT+QMTCFG=\"keepalive\",%d,%lu", MQTT_CLIENT_IDX, (unsigned long)MQTT_KEEPALIVE_S);
      issueCommand(cmd, NBIOT_AT_CMD_TIMEOUT_MS);
      return;
    }

    case MqttConnectSub::OPEN: {
      if (_cmd.outcome != CmdOutcome::NONE) {
        if (consumeOutcome() != CmdOutcome::OK) {
          handleFailureAtLevel(RecoveryLevel::MQTT_CONNECT, "QMTOPEN failed");
          return;
        }
        // CRITICAL: chain straight into QMTCONN with no delay/backoff - the
        // broker closes the socket ~5-6s after QMTOPEN if CONNECT doesn't
        // follow (+QMTSTAT: <idx>,7). The only latency here is however long
        // until the next tick(), which is effectively immediate.
        _mqttConnectSub = MqttConnectSub::CONN;
        return;
      }
      char cmd[96];
      snprintf(cmd, sizeof(cmd), "AT+QMTOPEN=%d,\"%s\",%d", MQTT_CLIENT_IDX, MQTT_BROKER_HOST, MQTT_BROKER_PORT);
      issueCommand(cmd, NBIOT_TIMEOUT_SOCKET_MS, CmdKind::QMTOPEN);
      return;
    }

    case MqttConnectSub::CONN: {
      if (_cmd.outcome != CmdOutcome::NONE) {
        if (consumeOutcome() != CmdOutcome::OK) {
          handleFailureAtLevel(RecoveryLevel::MQTT_CONNECT, "QMTCONN rejected/failed");
          return;
        }
        _mqttConnected = true;
        _mqttConnectRetries = 0;
        _consecutiveFailures = 0;
        setState(State::IDLE);
        return;
      }
      // ThingsBoard access-token auth: token as username, no password.
      // (2026-08-19: tried an explicit "" password param here on a
      // hunch after a CONNACK 5 "not authorized" with a confirmed-correct
      // token - that was rejected outright with "+CME ERROR: Incorrect
      // parameters", so this 3-arg form is the correct syntax. The real
      // cause of the CONNACK 5 was very likely a stale session left open
      // on the modem from an earlier test flash - see the CLOSE_FIRST sub-
      // state above, added for exactly this.)
      char cmd[128];
      snprintf(cmd, sizeof(cmd), "AT+QMTCONN=%d,\"%s\",\"%s\"", MQTT_CLIENT_IDX, MQTT_CLIENT_ID, MQTT_ACCESS_TOKEN);
      issueCommand(cmd, NBIOT_TIMEOUT_SOCKET_MS, CmdKind::QMTCONN);
      return;
    }
  }
}

void ModemNBIoTMqtt::tickIdle() {
  if (_cmd.active) return;

  CmdOutcome outcome = consumeOutcome();
  if (outcome == CmdOutcome::OK && _cmd.hasInfoLine) {
    int dbm;
    if (NbiotProtocol::parseCsq(_cmd.infoLine, dbm)) _rssiDbm = dbm;
  }

  bool batchDue = _ringCount > 0 && (_ringCount >= _batchReadingsTarget ||
                                      (millis() - _oldestPendingMs) >= _batchSecondsTarget * 1000UL);
  if (batchDue) {
    setState(State::PUBLISHING);
    return;
  }

  if (millis() - _lastCsqPollMs >= NBIOT_CSQ_POLL_INTERVAL_MS) {
    issueCommand("AT+CSQ", NBIOT_AT_CMD_TIMEOUT_MS);
    _lastCsqPollMs = millis();
  }
}

void ModemNBIoTMqtt::tickPublishing() {
  if (millis() < _backoffUntilMs) return;

  if (!_mqttConnected) {
    // Lazy reconnect: never attempt to publish over a session already
    // known dead. IDLE's own batchDue check routes straight back here once
    // MQTT_CONNECT succeeds - see project memory "project_thingsboard_mqtt_plan"
    // for why this beats a proactive keepalive ping cadence.
    _mqttReconnects++;
    _mqttConnectSub = MqttConnectSub::KEEPALIVE_CFG;
    setState(State::MQTT_CONNECT);
    return;
  }

  if (_cmd.active) return;

  if (!_publishInFlight) {
    _publishPayloadLen = buildGatewayPayload(_publishPayload, sizeof(_publishPayload));
    if (_publishPayloadLen == 0) {
      setState(State::IDLE);  // nothing to publish - shouldn't happen, batchDue implies a non-empty ring
      return;
    }
    char header[48];
    snprintf(header, sizeof(header), "AT+QMTPUB=%d,0,0,0,\"v1/gateway/telemetry\"", MQTT_CLIENT_IDX);
    issueCommand(header, NBIOT_TIMEOUT_SEND_MS, CmdKind::QMTPUB, _publishPayload, _publishPayloadLen);
    _publishInFlight = true;
    return;
  }

  CmdOutcome outcome = consumeOutcome();
  if (outcome == CmdOutcome::NONE) return;

  _publishInFlight = false;
  if (outcome == CmdOutcome::OK) {
    onPublishSucceeded();
  } else {
    onPublishFailed();
  }
}

void ModemNBIoTMqtt::tickError() {
  if (millis() < _backoffUntilMs) return;
  setState(State::OFF);
}

void ModemNBIoTMqtt::onPublishSucceeded() {
  _packetsSent++;
  _consecutiveFailures = 0;
  _publishRetries = 0;
  popSentReadings();
  setState(State::IDLE);
}

void ModemNBIoTMqtt::onPublishFailed() {
  _packetsFailed++;
  handleFailureAtLevel(RecoveryLevel::PUBLISH, "publish failed");
}

// Builds the ThingsBoard Gateway API telemetry payload:
//   {"<node>":[{"values":{...}},...], "<otherNode>":[...]}
// Groups ring entries by resolved node name (see Config.h's
// nbiotResolveNodeName) rather than assuming a single node - today there's
// only ever one, but a mixed batch shouldn't silently mis-group once a
// second node exists. No "ts" field - see project memory
// "project_thingsboard_mqtt_plan" for why (no reliable clock yet;
// ThingsBoard timestamps with its own receipt time when ts is absent).
size_t ModemNBIoTMqtt::buildGatewayPayload(uint8_t *out, size_t cap) {
  char *buf = reinterpret_cast<char *>(out);
  size_t pos = 0;
  auto append = [&](const char *s) {
    size_t n = strlen(s);
    if (pos + n < cap) {
      memcpy(buf + pos, s, n);
      pos += n;
    }
  };

  uint8_t count = _ringCount;
  if (count > _batchReadingsTarget) count = (uint8_t)_batchReadingsTarget;

  bool grouped[NBIOT_RING_CAPACITY] = {false};
  char groupName[24];

  append("{");
  bool firstNode = true;
  for (uint8_t i = 0; i < count; i++) {
    if (grouped[i]) continue;
    nbiotResolveNodeName(ringPeek(i).senderAddr, groupName, sizeof(groupName));

    if (!firstNode) append(",");
    firstNode = false;
    append("\"");
    append(groupName);
    append("\":[");

    bool firstReading = true;
    for (uint8_t j = i; j < count; j++) {
      if (grouped[j]) continue;
      const SensorSnapshot &snap = ringPeek(j);
      char thisName[24];
      nbiotResolveNodeName(snap.senderAddr, thisName, sizeof(thisName));
      if (strcmp(thisName, groupName) != 0) continue;

      grouped[j] = true;
      if (!firstReading) append(",");
      firstReading = false;

      char record[400];
      snprintf(record, sizeof(record),
               "{\"values\":{"
               "\"temp\":%.2f,\"rh\":%.2f,\"pres\":%.2f,\"gas\":%.2f,"
               "\"pm1\":%.2f,\"pm25\":%.2f,\"pm10\":%.2f,\"co2\":%.2f,"
               "\"co\":%.2f,\"coTemp\":%.2f,\"windAngle\":%.2f,\"windSpeed\":%.2f,"
               "\"windValid\":%s,\"rssi\":%d,\"snr\":%d}}",
               snap.temp, snap.hum, snap.pres, snap.gas, snap.pm1, snap.pm25, snap.pm10, snap.co2, snap.co,
               snap.coTemp, snap.windAngle, snap.windSpeed, snap.windValid ? "true" : "false", (int)snap.rssi,
               (int)snap.snr);
      append(record);
    }
    append("]");
  }
  append("}");

  _lastPublishCount = count;
  return pos;
}
