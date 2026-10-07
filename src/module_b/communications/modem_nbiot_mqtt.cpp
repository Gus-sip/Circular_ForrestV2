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
    case State::RPC_REPLY: tickRpcReply(); break;
    case State::ERROR: tickError(); break;
  }

  tickNodeCommandTimeout();
}

// ---------- Runtime config, settable from Module A ----------

uint32_t ModemNBIoTMqtt::setBatchSeconds(uint32_t seconds) {
  if (seconds < MQTT_BATCH_SECONDS_MIN) seconds = MQTT_BATCH_SECONDS_MIN;
  if (seconds > MQTT_BATCH_SECONDS_MAX) seconds = MQTT_BATCH_SECONDS_MAX;
  _batchSecondsTarget = seconds;
  Serial.printf("[nbiot-mqtt] uplink period set to %lus\n", (unsigned long)seconds);
  return seconds;
}

uint16_t ModemNBIoTMqtt::setBatchReadings(uint16_t readings) {
  if (readings < MQTT_BATCH_READINGS_MIN) readings = MQTT_BATCH_READINGS_MIN;
  if (readings > MQTT_BATCH_READINGS_MAX) readings = MQTT_BATCH_READINGS_MAX;
  _batchReadingsTarget = readings;
  Serial.printf("[nbiot-mqtt] uplink batch size set to %u readings\n", readings);
  return readings;
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
      if (_lineOverflowed) {
        // Say so. The previous code restarted the buffer mid-line and let the
        // REMAINDER be handled as a complete line, which turned an oversized
        // message into a plausible-looking fragment and hid the real fault.
        Serial.printf("[nbiot-mqtt] URC LONGER THAN %u BYTES - DISCARDED. A command "
                      "may have been lost; raise kLineBufLen\n",
                      (unsigned)kLineBufLen);
        _lineOverflowed = false;
      } else if (_lineLen > 0) {
        _lineBuf[_lineLen] = '\0';
        handleLine(_lineBuf, _lineLen);
      }
      _lineLen = 0;
    } else if (c != '\r') {
      if (_lineLen < sizeof(_lineBuf) - 1) {
        _lineBuf[_lineLen++] = c;
      } else {
        // Drop the REST of this line rather than starting a new one inside it.
        // A partial line is never valid input, and pretending otherwise is what
        // fed JSON tails to the parser.
        _lineOverflowed = true;
      }
    }
  }
}

void ModemNBIoTMqtt::handleLine(const char *line, size_t len) {
  // A DOWNLINK IS UNSOLICITED BY DEFINITION, SO IT IS TAKEN BEFORE ANYTHING ELSE.
  //
  // This used to sit below the command dispatch, which meant a +QMTRECV landing
  // while a command was in flight was only rescued for the four commands whose
  // real result arrives as an async URC (QMTOPEN/QMTCONN/QMTPUB/QMTSUB). During
  // any other command - AT+CSQ, AT+QMTCFG, a QMTPUB still waiting for its OK -
  // it fell through to the bottom of this function and was stored as the
  // command's informational response line, i.e. silently discarded.
  //
  // Observed live on 2026-10-05: a command was delivered the instant we
  // subscribed (the persistent session working as intended - the broker had it
  // queued) and was swallowed by the AT+QMTSUB exchange itself. It appeared in
  // the log as "<- +QMTRECV: ..." rather than "downlink on ...", which is the
  // tell: the "<- " prefix means a command consumed it.
  //
  // The modem interleaves URCs with command responses whenever it feels like
  // it, so there is no state in which ignoring one is correct.
  if (strncmp(line, "+QMTRECV", 8) == 0) {
    handleUrc(line, len);
    return;
  }

  if (!_cmd.active) {
    handleUrc(line, len);
    return;
  }
  Serial.print("[nbiot-mqtt] <- ");
  Serial.println(line);

  if (strcmp(line, "OK") == 0) {
    if (_cmd.kind == CmdKind::QMTOPEN || _cmd.kind == CmdKind::QMTCONN || _cmd.kind == CmdKind::QMTPUB ||
        _cmd.kind == CmdKind::QMTSUB) {
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
  if (_cmd.kind == CmdKind::QMTSUB && _cmd.sawOk) {
    int idx, msgId, result;
    if (NbiotProtocol::parseQmtsub(line, idx, msgId, result)) {
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
      Serial.printf("[nbiot-mqtt] +QMTSTAT fired (err=%d) - session marked dead, "
                    "will reconnect from IDLE\n",
                    errCode);
    }
    return;
  }

  // Inbound MQTT message - the downlink path from Module A. Arrives as an
  // unsolicited URC at any time, including in the middle of another command's
  // response, which is why it's handled here rather than in a command branch.
  NbiotProtocol::MqttMessage inbound;
  if (NbiotProtocol::parseQmtrecv(line, inbound)) {
    _downlinksReceived++;
    Serial.printf("[nbiot-mqtt] downlink on \"%s\": %s\n", inbound.topic, inbound.payload);
    handleInboundMqtt(inbound);
    return;
  }

  if (strstr(line, "RDY") || strstr(line, "QNBIOTEVENT")) {
    _sawBootUrc = true;
    return;
  }

  Serial.printf("[nbiot-mqtt] urc: %s\n", line);
}

// ---------- Downlink: Module A -> Module B (-> Module C) ----------

void ModemNBIoTMqtt::handleInboundMqtt(const NbiotProtocol::MqttMessage &msg) {
  // The gateway's own RPC topic carries the request id in the topic itself:
  //   v1/devices/me/rpc/request/<id>
  if (strncmp(msg.topic, "v1/devices/me/rpc/request/", 26) == 0) {
    handleOwnRpc(msg);
    return;
  }
  if (strcmp(msg.topic, MQTT_TOPIC_GATEWAY_RPC) == 0) {
    handleGatewayRpc(msg);
    return;
  }
  Serial.printf("[nbiot-mqtt] downlink on an unhandled topic, ignored: %s\n", msg.topic);
}

// RPC aimed at the gateway device itself = reconfigure Module B.
//   {"method":"cfg","params":{"UPLINK":300,"BATCH":6}}
void ModemNBIoTMqtt::handleOwnRpc(const NbiotProtocol::MqttMessage &msg) {
  const char *idStr = msg.topic + 26;  // past "v1/devices/me/rpc/request/"

  char method[24] = {0};
  NbiotProtocol::jsonString(msg.payload, "method", method, sizeof(method));

  char topic[NbiotProtocol::kMqttTopicLen];
  snprintf(topic, sizeof(topic), "%s%s", MQTT_TOPIC_DEVICE_RPC_RESP, idStr);

  char params[160] = {0};
  char reply[224];

  // ---- configurarModuloC: the agreed A -> C protocol ----
  //
  // Module A addresses the GATEWAY and names the target node inside params, rather
  // than using the gateway-RPC topic. The reply is sent IMMEDIATELY - "enviado" -
  // and the real outcome follows later as a commandLog record.
  //
  // That immediate ack is the point of the design, not a shortcut. A node is
  // asleep between uplink windows and can be minutes from hearing anything; an RPC
  // held open that long times out in ThingsBoard and reports a failure for a
  // command that was in fact delivered and applied.
  if (strcmp(method, "configurarModuloC") == 0) {
    handleConfigurarModuloC(msg, idStr);
    return;
  }

  if (strcmp(method, "cfg") != 0) {
    snprintf(reply, sizeof(reply),
             "{\"error\":\"unknown method '%s', expected 'configurarModuloC' or 'cfg'\"}", method);
    queueRpcReply(topic, reply);
    return;
  }
  if (!NbiotProtocol::jsonObject(msg.payload, "params", params, sizeof(params))) {
    snprintf(reply, sizeof(reply), "{\"error\":\"missing params\"}");
    queueRpcReply(topic, reply);
    return;
  }

  char applied[128] = {0};
  if (!applyOwnConfig(params, applied, sizeof(applied))) {
    snprintf(reply, sizeof(reply), "{\"error\":\"no recognized keys (want UPLINK and/or BATCH)\"}");
    queueRpcReply(topic, reply);
    return;
  }

  // Echo back the post-clamp values so a clamped request is visible.
  snprintf(reply, sizeof(reply), "{\"applied\":{%s}}", applied);
  queueRpcReply(topic, reply);
}

// Applies {"UPLINK":300,"BATCH":6} to this module's own runtime config.
bool ModemNBIoTMqtt::applyOwnConfig(const char *params, char *applied, size_t cap) {
  const char *cur = params;
  char key[24], value[24];
  size_t pos = 0;
  bool any = false;

  auto appendKv = [&](const char *k, unsigned long v) {
    int n = snprintf(applied + pos, cap - pos, "%s\"%s\":%lu", pos ? "," : "", k, v);
    if (n > 0 && (size_t)n < cap - pos) pos += (size_t)n;
  };

  while (NbiotProtocol::jsonNextPair(cur, key, sizeof(key), value, sizeof(value))) {
    long v = strtol(value, nullptr, 10);
    if (strcmp(key, "UPLINK") == 0) {
      appendKv("UPLINK", setBatchSeconds((uint32_t)(v < 0 ? 0 : v)));
      any = true;
    } else if (strcmp(key, "BATCH") == 0) {
      appendKv("BATCH", setBatchReadings((uint16_t)(v < 0 ? 0 : v)));
      any = true;
    } else {
      Serial.printf("[nbiot-mqtt] rpc: unrecognized own-config key \"%s\", ignored\n", key);
    }
  }
  applied[pos] = '\0';
  return any;
}

// RPC aimed at a child device = relay to that node over LoRa.
//   {"device":"NodoC-1","data":{"id":42,"method":"cfg","params":{"INTERVAL":300}}}
void ModemNBIoTMqtt::handleGatewayRpc(const NbiotProtocol::MqttMessage &msg) {
  char device[24] = {0};
  char data[224] = {0};
  if (!NbiotProtocol::jsonString(msg.payload, "device", device, sizeof(device)) ||
      !NbiotProtocol::jsonObject(msg.payload, "data", data, sizeof(data))) {
    Serial.println("[nbiot-mqtt] gateway rpc missing device/data, ignored");
    return;
  }

  long rpcId = 0;
  NbiotProtocol::jsonInt(data, "id", rpcId);

  char reply[224];
  auto replyErr = [&](const char *err) {
    snprintf(reply, sizeof(reply), "{\"device\":\"%s\",\"id\":%ld,\"data\":{\"error\":\"%s\"}}", device, rpcId,
             err);
    queueRpcReply(MQTT_TOPIC_GATEWAY_RPC, reply);
  };

  char method[24] = {0};
  NbiotProtocol::jsonString(data, "method", method, sizeof(method));

  // BOTH method names are accepted on BOTH topics.
  //
  // Which topic a command arrives on depends entirely on which ThingsBoard device
  // Module A addresses: the gateway CON-1 lands on v1/devices/me/rpc/request/<id>,
  // a child device lands here on v1/gateway/rpc. The protocol document specifies
  // configurarModuloC and describes the gateway path, but a dashboard built per
  // node will naturally address the node - and rejecting it for the sake of a name
  // would be a refusal on a technicality, when the intent is unambiguous.
  //
  // Both carry the same thing: a target node and a set of keys to apply.
  if (strcmp(method, "cfg") != 0 && strcmp(method, "configurarModuloC") != 0) {
    replyErr("unknown method, expected 'configurarModuloC' or 'cfg'");
    return;
  }

  char params[160] = {0};
  if (!NbiotProtocol::jsonObject(data, "params", params, sizeof(params))) {
    replyErr("missing params");
    return;
  }

  const int slot = freeNodeCmdSlot();
  if (slot < 0) {
    replyErr("command queue full");
    return;
  }

  // Translate the params object into the "CFG,K=V,..." string Module C's
  // applyConfigCommand() already parses. Keys aren't validated here - Module C
  // ignores what it doesn't recognize and ACKs only what it applied, so the
  // node stays the authority on its own config vocabulary.
  const char *cur = params;
  char key[24], value[24];
  size_t pos = 0;
  pos += (size_t)snprintf(_pendingCfgScratch, sizeof(_pendingCfgScratch), "CFG");
  bool any = false;
  while (NbiotProtocol::jsonNextPair(cur, key, sizeof(key), value, sizeof(value))) {
    int n = snprintf(_pendingCfgScratch + pos, sizeof(_pendingCfgScratch) - pos, ",%s=%s", key, value);
    if (n > 0 && (size_t)n < sizeof(_pendingCfgScratch) - pos) {
      pos += (size_t)n;
      any = true;
    }
  }
  if (!any) {
    _pendingCfgScratch[0] = '\0';
    replyErr("params object was empty");
    return;
  }

  NodeCommand &nc = _nodeCmds[slot];
  strncpy(nc.cfg, _pendingCfgScratch, sizeof(nc.cfg) - 1);
  nc.cfg[sizeof(nc.cfg) - 1] = '\0';
  strncpy(nc.device, device, sizeof(nc.device) - 1);
  nc.device[sizeof(nc.device) - 1] = '\0';
  nc.rpcId = rpcId;
  nc.queuedMs = millis();
  nc.awaitingAck = false;
  nc.viaCommandLog = false;
  nc.pending = true;

  Serial.printf("[nbiot-mqtt] queued for %s: \"%s\" (waiting for its next uplink window)\n", nc.device,
                nc.cfg);
}

void ModemNBIoTMqtt::onNodeCommandDelivered(const char *device) {
  const int i = deliverableIndexFor(device);
  if (i < 0) return;
  NodeCommand &nc = _nodeCmds[i];
  nc.awaitingAck = true;
  nc.lastSentMs = millis();
  if (nc.attempts < 255) nc.attempts++;
  if (nc.attempts == 1) {
    Serial.printf("[nbiot-mqtt] sent to %s over LoRa, awaiting ACK\n", nc.device);
  } else {
    Serial.printf("[nbiot-mqtt] RESENT to %s over LoRa (attempt %u - no ACK to the "
                  "previous one), awaiting ACK\n", nc.device, (unsigned)nc.attempts);
  }
}

void ModemNBIoTMqtt::onNodeCommandNack(const char *device, const char *reason) {
  int i = awaitingAckIndexFor(device);
  if (i < 0) i = undeliveredIndexFor(device);
  if (i < 0) return;
  NodeCommand &nc = _nodeCmds[i];

  if (nc.viaCommandLog) {
    publishCommandLogApplied(nc.rpcId, "error", reason, nc.device);
  } else {
    char reply[224];
    snprintf(reply, sizeof(reply), "{\"device\":\"%s\",\"id\":%ld,\"data\":{\"error\":\"%s\"}}",
             nc.device, nc.rpcId, reason);
    queueRpcReply(MQTT_TOPIC_GATEWAY_RPC, reply);
  }
  Serial.printf("[nbiot-mqtt] %s REJECTED the command: %s\n", nc.device, reason);
  nc = NodeCommand{};
}

void ModemNBIoTMqtt::onNodeCommandAck(const char *device, const char *ackPayload) {
  // Match the ACK to the command awaiting one FROM THAT NODE. With several
  // commands in flight to different nodes, clearing the wrong one would report
  // the wrong requestId as confirmed - a command would appear to have succeeded
  // while another silently timed out.
  int i = awaitingAckIndexFor(device);
  if (i < 0) i = undeliveredIndexFor(device);  // ACK beat our delivery bookkeeping
  if (i < 0) return;
  NodeCommand &_nodeCmd = _nodeCmds[i];

  // "ACK,INTERVAL=300,BMV080=0" -> report the applied (post-clamp) values back
  // as the RPC result, so a clamp on the node is visible in ThingsBoard.
  const char *body = strncmp(ackPayload, "ACK,", 4) == 0 ? ackPayload + 4 : ackPayload;
  if (_nodeCmd.viaCommandLog) {
    // Carries WHAT was applied, not just that something was. The node ACKs only
    // the keys it actually took, so this is the difference between Module A
    // believing a threshold changed and being able to show the value the node is
    // really using.
    publishCommandLogApplied(_nodeCmd.rpcId, "confirmado", body, _nodeCmd.device);
  } else {
    char reply[224];
    snprintf(reply, sizeof(reply), "{\"device\":\"%s\",\"id\":%ld,\"data\":{\"applied\":\"%s\"}}",
             _nodeCmd.device, _nodeCmd.rpcId, body);
    queueRpcReply(MQTT_TOPIC_GATEWAY_RPC, reply);
  }

  Serial.printf("[nbiot-mqtt] %s acked: %s\n", _nodeCmd.device, body);
  _nodeCmd = NodeCommand{};
}

void ModemNBIoTMqtt::tickNodeCommandTimeout() {
  // Every slot is timed independently: one node being unreachable must not keep
  // another node's command alive past its own deadline.
  for (uint8_t qi = 0; qi < kNodeCmdQueue; qi++) {
    NodeCommand &_nodeCmd = _nodeCmds[qi];
    if (!_nodeCmd.pending) continue;
    if (millis() - _nodeCmd.queuedMs < DOWNLINK_QUEUE_TIMEOUT_MS) continue;

  // Built once, above the branch, because the serial line at the end of this
  // loop reports it as well - whichever protocol the command arrived on.
  char why[72];
  if (_nodeCmd.awaitingAck) {
    snprintf(why, sizeof(why), "delivered %u time(s), no ACK", (unsigned)_nodeCmd.attempts);
  } else {
    snprintf(why, sizeof(why), "node never uplinked - not delivered");
  }

  if (_nodeCmd.viaCommandLog) {
    // SAY WHICH HALF FAILED. This used to publish a bare "timeout", throwing away
    // the one fact the operator needs: whether the command was ever put on the
    // air. Module B already knows - awaitingAck means the node uplinked, the
    // downlink went out in its listen window, and it then said nothing; without
    // it, the node never transmitted at all.
    //
    // The two call for opposite responses. "Delivered, no ACK" is a node that is
    // alive and refusing or crashing - look at the node. "Never delivered" is a
    // node that is not transmitting - look at the radio, the power or the siting.
    // A single word for both sends you to the wrong half, and this is a tower in
    // a forest, not a board on a desk.
    //
    // Carried in `detail`, which the protocol document leaves free, so the four
    // documented status values are unchanged.
    publishCommandLogApplied(_nodeCmd.rpcId, "timeout", why, _nodeCmd.device);
  } else {
    char reply[224];
    snprintf(reply, sizeof(reply), "{\"device\":\"%s\",\"id\":%ld,\"data\":{\"error\":\"%s\"}}", _nodeCmd.device,
             _nodeCmd.rpcId,
             _nodeCmd.awaitingAck ? "delivered but the node never acked" : "node never uplinked - not delivered");
    queueRpcReply(MQTT_TOPIC_GATEWAY_RPC, reply);
  }
    Serial.printf("[nbiot-mqtt] command for %s timed out (%s)\n", _nodeCmd.device, why);
    _nodeCmd = NodeCommand{};
  }
}

static size_t appendI64(char *out, int64_t v);  // defined below, near the payload builder

// The outcome half of the agreed protocol. The RPC itself was answered "enviado"
// when it arrived; this is what actually says whether the node did the thing.
//
// Published as TELEMETRY, not as an RPC response, because the RPC was closed
// minutes ago - a node can be a whole uplink window away from hearing a command,
// and holding an RPC open that long is what made commands look like failures when
// they were merely slow.
// Turns the node's ACK body - "temp_pre_off=48,temp_pre_on=50" - into JSON
// fields: ,"temp_pre_off":48,"temp_pre_on":50
//
// WHY AS SEPARATE KEYS AND NOT JUST THE STRING: Module A needs to know the
// threshold the node is now using, and `detail` carries that only as text it
// would have to parse. As telemetry keys each one is a value a dashboard can
// bind to, plot and compare against what it asked for. The names are the node's
// own vocabulary - magnitude plus suffix, exactly the spreadsheet's names - so
// nothing new is invented here.
//
// Numbers go out bare so they are stored as numbers; anything that does not
// parse cleanly as one is quoted instead of being emitted as broken JSON. These
// pairs come off the radio, so they are not assumed to be well formed.
static size_t appendKvAsJson(char *out, size_t cap, const char *kv) {
  size_t pos = 0;
  const char *p = kv;
  while (*p && pos + 8 < cap) {
    const char *comma = strchr(p, ',');
    const char *end = comma ? comma : p + strlen(p);
    const char *eq = nullptr;
    for (const char *q = p; q < end; q++) {
      if (*q == '=') { eq = q; break; }
    }
    if (eq && eq > p && (end - eq) > 1) {
      const size_t klen = (size_t)(eq - p);
      const size_t vlen = (size_t)(end - eq - 1);

      // Numeric, by inspection rather than by trusting strtod's tail.
      bool numeric = vlen > 0;
      bool dot = false;
      for (size_t i = 0; i < vlen && numeric; i++) {
        const char c = eq[1 + i];
        if (c == '-' && i == 0) continue;
        if (c == '.' && !dot) { dot = true; continue; }
        if (c < '0' || c > '9') numeric = false;
      }

      const size_t need = klen + vlen + (numeric ? 4 : 6);
      if (pos + need >= cap) break;
      out[pos++] = ',';
      out[pos++] = '"';
      memcpy(out + pos, p, klen); pos += klen;
      out[pos++] = '"';
      out[pos++] = ':';
      if (!numeric) out[pos++] = '"';
      memcpy(out + pos, eq + 1, vlen); pos += vlen;
      if (!numeric) out[pos++] = '"';
      out[pos] = 0;
    }
    if (!comma) break;
    p = comma + 1;
  }
  return pos;
}

void ModemNBIoTMqtt::publishCommandLog(long requestId, const char *status) {
  publishCommandLogApplied(requestId, status, nullptr, nullptr);
}

void ModemNBIoTMqtt::publishCommandLogApplied(long requestId, const char *status,
                                              const char *detail, const char *device) {
  // Built by hand rather than with one snprintf because the timestamp is an
  // int64 and %lld is NOT reliably compiled into this toolchain's snprintf -
  // nano newlib often omits it, which is why appendI64() exists at all. A %llu
  // here would have emitted a literal "llu" or garbage into live telemetry.
  char payload[192];
  int n = snprintf(payload, sizeof(payload),
                   "{\"seguimientoCmd\":{\"requestId\":%ld,\"status\":\"%s\"", requestId, status);
  if (n < 0) return;
  size_t pos = (size_t)n;

  // Quotes and backslashes in `detail` would break the JSON. The node's ACK body
  // is "k=v,k=v" and its NACK a short phrase, so neither should contain them -
  // but a malformed packet must not be able to emit invalid telemetry, so they
  // are dropped rather than trusted.
  if (detail && *detail && pos + 24 < sizeof(payload)) {
    pos += (size_t)snprintf(payload + pos, sizeof(payload) - pos, ",\"detail\":\"");
    for (const char *p = detail; *p && pos < sizeof(payload) - 8; p++) {
      if (*p == '"' || *p == '\\' || (unsigned char)*p < 0x20) continue;
      payload[pos++] = *p;
    }
    if (pos < sizeof(payload) - 4) payload[pos++] = '"';
    payload[pos] = 0;
  }

  if (_haveNetTime && pos + 40 < sizeof(payload)) {
    pos += (size_t)snprintf(payload + pos, sizeof(payload) - pos, ",\"timestamp\":");
    pos += appendI64(payload + pos, netNowMs());
  }
  // Without network time the field is OMITTED rather than sent as 0. A commandLog
  // stamped 1970 would sort to the beginning of every dashboard and look like the
  // oldest record in the system.
  snprintf(payload + pos, sizeof(payload) - pos, "}}");

  queueRpcReply(MQTT_TOPIC_DEVICE_TELEMETRY, payload);

  // THE SAME RECORD, ALSO ON THE NODE'S OWN TELEMETRY.
  //
  // v1/devices/me/telemetry carries the gateway's access token, so `seguimientoCmd`
  // lands on the GATEWAY device (CON-1) and never on NodoC-3. "enviado" goes to
  // the node's RPC response topic, so a dashboard bound to the node sees the
  // command leave and never sees it confirmed - it sits at "en transito al
  // MODULO-C" indefinitely while Module B's log says confirmado and the broker
  // acked the publish. Reported repeatedly on 2026-10-05 for requests 6..10.
  //
  // Which device the dashboard is actually bound to is not something this
  // firmware can know, so the record goes to BOTH rather than being moved from
  // one to the other - moving it would break whichever side is working today.
  // Same key, same shape, no new vocabulary invented: that is what the last two
  // bugs came from.
  //
  // `values` for the gateway topic is exactly the device-topic payload, so the
  // object is reused rather than rebuilt - there is no second place for the two
  // to drift apart.
  if (device && *device) {
    // The node mirror's values: the commandLog record, PLUS one field per
    // threshold the node reported applying. Built by reopening the payload's
    // closing brace rather than formatting the record twice, so the two copies
    // cannot drift.
    char vals[384];
    size_t vp = 0;
    const size_t plen = strlen(payload);
    if (plen >= 2 && plen < sizeof(vals) - 2) {
      memcpy(vals, payload, plen - 1);  // everything but the final '}'
      vp = plen - 1;
      vals[vp] = 0;
      // Only a confirmation carries applied values. A timeout's detail is prose
      // and an error's is a refusal reason - neither is a threshold.
      if (detail && *detail && strcmp(status, "confirmado") == 0 && strchr(detail, '=')) {
        vp += appendKvAsJson(vals + vp, sizeof(vals) - vp, detail);
      }
      if (vp + 2 < sizeof(vals)) { vals[vp++] = '}'; vals[vp] = 0; }
    } else {
      snprintf(vals, sizeof(vals), "%s", payload);
    }

    char mirror[480];
    if (_haveNetTime) {
      int m = snprintf(mirror, sizeof(mirror), "{\"%s\":[{\"ts\":", device);
      if (m > 0 && (size_t)m < sizeof(mirror)) {
        size_t mp = (size_t)m;
        mp += appendI64(mirror + mp, netNowMs());
        if (mp + strlen(vals) + 16 < sizeof(mirror)) {
          snprintf(mirror + mp, sizeof(mirror) - mp, ",\"values\":%s}]}", vals);
          queueRpcReply("v1/gateway/telemetry", mirror);
        }
      }
    } else {
      // No network clock: send it flat and let ThingsBoard stamp it on receipt,
      // rather than emitting a 1970 timestamp that sorts to the top of every
      // dashboard - the same reasoning as the omitted field above.
      int m = snprintf(mirror, sizeof(mirror), "{\"%s\":[%s]}", device, vals);
      if (m > 0 && (size_t)m < sizeof(mirror)) queueRpcReply("v1/gateway/telemetry", mirror);
    }
  }

  // THE DASHBOARD READS ATTRIBUTES, NOT TELEMETRY.
  //
  // Straight from the protocol document, under "Control del Comando en MODULOA":
  //
  //     MODULOA recibe la telemetria
  //     Detecta "seguimientoCmd" (v2; la v1 lo llamaba "commandLog")
  //     Copia automaticamente a atributos del ModC
  //     Dashboard lee atributos (rapido)
  //     Historico guardado en telemetria
  //
  // So the telemetry record above is the HISTORY, and the dashboard reads the
  // node's ATTRIBUTES - with Module A responsible for copying one to the other
  // via a ThingsBoard rule chain. If that copy is not configured, the telemetry
  // arrives, the broker acks it, Module B's log says confirmado, and the
  // dashboard sits at "en transito al MODULO-C" forever with nothing wrong
  // anywhere in the firmware. That is exactly what was happening.
  //
  // Writing the attributes here removes the dependency on that rule chain
  // existing. It does not replace the telemetry record, which the document
  // requires and which remains the history.
  //
  // Key names: `seguimientoCmd` is what Module A actually subscribes to. v1 of the
  // protocol document says `commandLog`, and that is what this published until
  // 2026-10-07 - v2 of the document renamed it and we were working from v1. The
  // records were arriving and being stored correctly the whole time under a name
  // nothing was listening for. The threshold keys are the node's own
  // vocabulary, as echoed in its ACK. Nothing is invented - inventing names is
  // what the action-name mismatch cost us.
  if (device && *device) {
    char attrs[480];
    int a = snprintf(attrs, sizeof(attrs), "{\"%s\":{\"seguimientoCmd\":", device);
    if (a > 0 && (size_t)a < sizeof(attrs)) {
      size_t ap = (size_t)a;
      // The record without its {"commandLog": wrapper - reuse, do not rebuild.
      // sizeof-1 for the length, not a hand-counted offset. Counting it by eye
      // gave 15 for a 13-character key and ate the "{\"" of "{\"requestId\"",
      // putting {"NodoC-3":{"commandLog":requestId":11,... on the wire - valid
      // MQTT carrying invalid JSON, which ThingsBoard drops without complaint.
      static const char kKey[] = "\"seguimientoCmd\":";
      const char *inner = strstr(payload, kKey);
      if (inner) {
        inner += sizeof(kKey) - 1;
        const size_t ilen = strlen(inner);
        if (ilen >= 1 && ap + ilen + 8 < sizeof(attrs)) {
          memcpy(attrs + ap, inner, ilen - 1);  // drop payload's outer closing brace
          ap += ilen - 1;
          attrs[ap] = 0;
          if (detail && *detail && strcmp(status, "confirmado") == 0 && strchr(detail, '=')) {
            ap += appendKvAsJson(attrs + ap, sizeof(attrs) - ap, detail);
          }
          // TWO braces: one closes the per-device object, one closes the
          // envelope. {"NodoC-3":{ ... }} - the envelope's was missing.
          if (ap + 4 < sizeof(attrs)) {
            attrs[ap++] = '}';
            attrs[ap++] = '}';
            attrs[ap] = 0;
            queueRpcReply("v1/gateway/attributes", attrs);
          }
        }
      }
    }
  }

  Serial.printf("[nbiot-mqtt] seguimientoCmd %ld -> %s%s\n", requestId, status,
                (device && *device) ? " (gateway telemetry + node telemetry + node attributes)" : "");
}

// {"method":"configurarModuloC",
//  "params":{"moduloC_id":"NodoC-1","action":"frecuencia","value":"freq_alarma"}}
//
// Answers the RPC immediately with "enviado", then queues the command for the
// node's next uplink window. The outcome arrives later as commandLog.
void ModemNBIoTMqtt::handleConfigurarModuloC(const NbiotProtocol::MqttMessage &msg,
                                             const char *idStr) {
  const long requestId = strtol(idStr, nullptr, 10);

  char respTopic[NbiotProtocol::kMqttTopicLen];
  snprintf(respTopic, sizeof(respTopic), "%s%s", MQTT_TOPIC_DEVICE_RPC_RESP, idStr);

  char params[224] = {0};
  char device[24] = {0};
  char action[24] = {0};
  char value[32] = {0};
  char reply[224];

  auto refuse = [&](const char *err) {
    // A refusal is answered on the RPC itself, not as commandLog: nothing was
    // ever queued, so there is no outcome to report later.
    snprintf(reply, sizeof(reply), "{\"status\":\"error\",\"error\":\"%s\"}", err);
    queueRpcReply(respTopic, reply);
    Serial.printf("[nbiot-mqtt] configurarModuloC %ld rechazado: %s\n", requestId, err);
  };

  if (!NbiotProtocol::jsonObject(msg.payload, "params", params, sizeof(params))) {
    refuse("missing params");
    return;
  }
  if (!NbiotProtocol::jsonString(params, "moduloC_id", device, sizeof(device))) {
    refuse("missing moduloC_id");
    return;
  }
  NbiotProtocol::jsonString(params, "action", action, sizeof(action));
  NbiotProtocol::jsonString(params, "value", value, sizeof(value));

  const int slot = freeNodeCmdSlot();
  if (slot < 0) {
    refuse("command queue full");
    return;
  }

  // ---- action -> the CFG vocabulary Module C already parses ----
  char cfg[128] = {0};
  if (strcmp(action, "frecuencia") == 0) {
    // The three named rates. THESE SECONDS ARE A CHOICE, NOT FROM THE SPEC, which
    // names the presets without giving values - they are the sane mapping onto
    // CFG,INTERVAL and are the obvious thing to revise once the real numbers are
    // agreed. freq_alarma is the LORA_TX_PERIOD_MIN_MS floor.
    unsigned secs = 0;
    if (strcmp(value, "freq_normal") == 0) secs = 300;
    else if (strcmp(value, "freq_prealarma") == 0) secs = 60;
    else if (strcmp(value, "freq_alarma") == 0) secs = 15;
    if (secs == 0) {
      refuse("value must be freq_normal, freq_prealarma or freq_alarma");
      return;
    }
    snprintf(cfg, sizeof(cfg), "CFG,INTERVAL=%u", secs);

  } else if (strcmp(action, "umbral") == 0 || strcmp(action, "umbrales_de1variable") == 0) {
    // THE MAGNITUDE AND THE THRESHOLD ARRIVE IN SEPARATE FIELDS, and have to be
    // recombined before the node will recognise anything. Observed live on
    // 2026-10-05, and not what the protocol document shows:
    //
    //   {"action":"umbrales_de1variable","variable":"temp",
    //    "umbral_pre_off":48,"umbral_pre_on":50,
    //    "umbral_alarm_off":55,"umbral_alarm_on":60}
    //
    // The node addresses a threshold as <magnitude><suffix> - temp_pre_on - so a
    // key of "umbral_pre_on" matches no rule and is silently ignored. Every
    // threshold in the command would be dropped and the node would NACK the lot,
    // which reads like a firmware fault and is really a naming mismatch.
    //
    // So "umbral_" is rewritten to "<variable>_". A key that already carries its
    // magnitude (temp_pre_on, as the dashboard's own table uses) passes through
    // untouched - both forms work.
    char variable[24] = {0};
    NbiotProtocol::jsonString(params, "variable", variable, sizeof(variable));

    const char *cur = params;
    char k[40], v[32];
    size_t pos = (size_t)snprintf(cfg, sizeof(cfg), "CFG");
    bool any = false;
    while (NbiotProtocol::jsonNextPair(cur, k, sizeof(k), v, sizeof(v))) {
      if (strcmp(k, "moduloC_id") == 0 || strcmp(k, "action") == 0 ||
          strcmp(k, "variable") == 0 || strcmp(k, "value") == 0) {
        continue;  // routing, not a threshold
      }

      int n;
      if (strncmp(k, "umbral_", 7) == 0 && variable[0]) {
        n = snprintf(cfg + pos, sizeof(cfg) - pos, ",%s_%s=%s", variable, k + 7, v);
      } else {
        n = snprintf(cfg + pos, sizeof(cfg) - pos, ",%s=%s", k, v);
      }
      if (n > 0 && (size_t)n < sizeof(cfg) - pos) {
        pos += (size_t)n;
        any = true;
      }
    }
    if (!any) {
      refuse("umbral with no threshold keys");
      return;
    }
    if (strncmp(params, "", 0) == 0 && !variable[0]) {
      // Not fatal - keys may already carry their magnitude - but worth saying,
      // because if they do not, the node will refuse every one of them.
      Serial.println("[nbiot-mqtt] umbral without a \"variable\" field - keys must "
                     "already name their magnitude");
    }

  } else {
    refuse("unknown action, expected 'frecuencia' or 'umbral'");
    return;
  }

  NodeCommand &nc = _nodeCmds[slot];
  strncpy(nc.device, device, sizeof(nc.device) - 1);
  nc.device[sizeof(nc.device) - 1] = 0;
  strncpy(nc.cfg, cfg, sizeof(nc.cfg) - 1);
  nc.cfg[sizeof(nc.cfg) - 1] = 0;
  nc.rpcId = requestId;
  nc.queuedMs = millis();
  nc.awaitingAck = false;
  nc.viaCommandLog = true;   // outcome goes out as commandLog, not as this RPC
  nc.pending = true;

  // ANSWER NOW. The node may be minutes from its next uplink window, and an RPC
  // held open that long times out in ThingsBoard and reports a failure for a
  // command that was delivered and applied.
  snprintf(reply, sizeof(reply), "{\"status\":\"enviado\",\"requestId\":%ld}", requestId);
  queueRpcReply(respTopic, reply);

  Serial.printf("[nbiot-mqtt] configurarModuloC %ld -> %s: \"%s\" (enviado; esperando ventana)\n",
                requestId, nc.device, nc.cfg);
}

void ModemNBIoTMqtt::queueRpcReply(const char *topic, const char *payload) {
  if (_rpcReplyCount >= kRpcReplyQueue) {
    // Full. The OLDEST goes, not the newest: a newer record supersedes an older
    // one for the same command, and the newest is the one still worth sending.
    Serial.println("[nbiot-mqtt] rpc reply queue full - dropping the oldest");
    popRpcReply();
  }
  const uint8_t slot = (uint8_t)((_rpcReplyHead + _rpcReplyCount) % kRpcReplyQueue);
  RpcReply &r = _rpcReplies[slot];
  strncpy(r.topic, topic, sizeof(r.topic) - 1);
  r.topic[sizeof(r.topic) - 1] = '\0';
  strncpy(r.payload, payload, sizeof(r.payload) - 1);
  r.payload[sizeof(r.payload) - 1] = '\0';
  _rpcReplyCount++;
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
  // Trace every transition. The OLED shows only the CURRENT state name, so a
  // fast POWERING -> WAIT_AT -> ERROR -> OFF loop looks like a single stuck
  // state on the screen - which is exactly how "NBIoT: WAIT_AT" got read as
  // "connected to the NB-IoT band". On serial the cycling is unmistakable.
  if (_state != s) {
    Serial.printf("[nbiot-mqtt] state %s -> %s\n", stateName(), stateNameOf(s));
    Serial.flush();
  }
  _state = s;
  _stateEnteredMs = millis();
  switch (s) {
    case State::ATTACHING:
      _cgpaddrChecked = false;
      _csqCheckedAfterAttach = false;
      _cclkChecked = false;
      _lastCeregPollMs = 0;
      break;
    case State::IDLE:
      _lastCsqPollMs = 0;  // poll once shortly after entering IDLE
      break;
    case State::PUBLISHING:
      _publishInFlight = false;
      break;
    case State::RPC_REPLY:
      _rpcReplyInFlight = false;
      break;
    default:
      break;
  }
}

// Stop driving the modem UART entirely. Ending the peripheral is not enough on
// its own - the pin must also be taken out of output mode, or it keeps its last
// level and continues to feed the unpowered module.
void ModemNBIoTMqtt::releaseUart() {
  if (_uartStarted) {
    _serial.end();
    _uartStarted = false;
  }
  pinMode(_txPin, INPUT);
  pinMode(_rxPin, INPUT);
}

void ModemNBIoTMqtt::startUart() {
  if (_uartStarted) return;
  _serial.begin(_baud, SERIAL_8N1, _rxPin, _txPin);
  _uartStarted = true;
  while (_serial.available()) _serial.read();  // drop power-up line noise
}

void ModemNBIoTMqtt::enterPowering() {
  // UART DOWN FIRST, and the TX line released, before power is touched. See the
  // block comment above: a driven TX pin back-powers the module while VIN is off
  // and prevents a genuine cold start. The UART is reopened in tickPowering()
  // once the rail has come up and settled.
  releaseUart();

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

void ModemNBIoTMqtt::powerDown() {
  // Release TX before cutting VIN - otherwise the module is still being fed
  // through it and this is not a power cycle at all.
  releaseUart();
  digitalWrite(_enPin, NBIOT_DISABLE);
  digitalWrite(_channelPin, !NBIOT_CHANNEL_ACTIVE);
  digitalWrite(_pwrkeyPin, !NBIOT_PWRKEY_ACTIVE);
  _attached = false;
  _mqttConnected = false;
  _powerCycleSettleUntilMs = millis() + NBIOT_POWER_OFF_SETTLE_MS;
}

void ModemNBIoTMqtt::powerCycle() {
  setLastError("power cycling modem (last resort)");
  powerDown();
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
  // Also PRINT it. This only stored the string before, which is why a modem that
  // never answered produced a serial log containing nothing but "-> AT" - the
  // give-up, the power cycle and the reason were all invisible, and the only
  // place the error surfaced was a getter nothing was calling. A silent recovery
  // loop is indistinguishable from a hung one.
  Serial.printf("[nbiot-mqtt] ERROR: %s\n", msg);
  Serial.flush();
}

const char *ModemNBIoTMqtt::stateName() const { return stateNameOf(_state); }

const char *ModemNBIoTMqtt::stateNameOf(State st) {
  switch (st) {
    case State::OFF: return "OFF";
    case State::POWERING: return "POWERING";
    case State::WAIT_AT: return "WAIT_AT";
    case State::CONFIG: return "CONFIG";
    case State::ATTACHING: return "ATTACHING";
    case State::MQTT_CONNECT: return "MQTT_CONNECT";
    case State::IDLE: return "IDLE";
    case State::PUBLISHING: return "PUBLISHING";
    case State::RPC_REPLY: return "RPC_REPLY";
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
  // VIN and the channel gate must be stable BEFORE PWRKEY is pulsed - pulsing
  // into a rail that is still coming up can be ignored entirely by the module.
  // Raised 300 -> 2000 (2026-09-11) along with the WAIT_AT ceiling; both were
  // part of the same too-aggressive power-on race.
  static const uint32_t kPowerSettleMs = 2000;
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
    // Only now, with the module powered and settled, is it safe to drive TX.
    startUart();
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
    // CTZU and QSCLK are best-effort (see their cases below) - every other
    // step is required, an ERROR/timeout there is fatal.
    if (outcome != CmdOutcome::OK && _configStep != ConfigStep::QSCLK &&
        _configStep != ConfigStep::CTZU) {
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
    case ConfigStep::CTZU:
      // Auto network time-zone/clock update, so AT+CCLK? later returns real
      // network time to timestamp each uplink reading. Set before CFUN=1 so
      // the sync happens during attach. Best-effort: if the module rejects
      // it, per-reading "ts" is just skipped (see buildGatewayPayload).
      issueCommand("AT+CTZU=1", NBIOT_AT_CMD_TIMEOUT_MS);
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

  if (!_csqCheckedAfterAttach) {
    consumeOutcome();  // whatever CGPADDR said, CEREG already confirmed attachment - not worth failing over
    // One AT+CSQ right here, not gated behind MQTT_CONNECT succeeding -
    // otherwise the signal reading (and anything displaying it, e.g. the
    // OLED's bars) never updates while Module A/the broker is unreachable,
    // even though the modem is genuinely attached with a real signal.
    issueCommand("AT+CSQ", NBIOT_AT_CMD_TIMEOUT_MS);
    _csqCheckedAfterAttach = true;
    return;
  }

  if (!_cclkChecked) {
    CmdOutcome csqOutcome = consumeOutcome();
    if (csqOutcome == CmdOutcome::OK && _cmd.hasInfoLine) {
      int dbm;
      if (NbiotProtocol::parseCsq(_cmd.infoLine, dbm)) _rssiDbm = dbm;
    }
    // One AT+CCLK? per attach to seed the epoch<->millis() offset used to
    // timestamp each reading in the MQTT uplink. Best-effort: a failure (or
    // an implausible clock) just leaves _haveNetTime false and the uplink
    // omits "ts".
    issueCommand("AT+CCLK?", NBIOT_AT_CMD_TIMEOUT_MS);
    _cclkChecked = true;
    return;
  }

  CmdOutcome cclkOutcome = consumeOutcome();
  if (cclkOutcome == CmdOutcome::OK && _cmd.hasInfoLine) {
    int64_t epochMs;
    if (NbiotProtocol::parseCclk(_cmd.infoLine, epochMs) && epochMs >= NBIOT_MIN_VALID_EPOCH_MS) {
      _netEpochMsAtSync = epochMs;
      _netSyncLocalMs = millis();
      _haveNetTime = true;
      Serial.printf("[nbiot-mqtt] network time acquired: epoch %lld ms - uplink readings will carry per-reading ts\n",
                    (long long)epochMs);
    } else {
      Serial.println("[nbiot-mqtt] no usable network time (AT+CCLK?) - uplink will use ThingsBoard receipt time");
    }
  }

  _mqttConnectSub = MqttConnectSub::KEEPALIVE_CFG;
  setState(State::MQTT_CONNECT);
}

int64_t ModemNBIoTMqtt::netNowMs() const {
  return _netEpochMsAtSync + (int64_t)(int32_t)(millis() - _netSyncLocalMs);
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
        consumeOutcome();  // best-effort - proceed even if QMTCFG was rejected
        _mqttConnectSub = MqttConnectSub::SESSION_CFG;
        return;
      }
      char cmd[48];
      snprintf(cmd, sizeof(cmd), "AT+QMTCFG=\"keepalive\",%d,%lu", MQTT_CLIENT_IDX, (unsigned long)MQTT_KEEPALIVE_S);
      issueCommand(cmd, NBIOT_AT_CMD_TIMEOUT_MS);
      return;
    }

        case MqttConnectSub::SESSION_CFG: {
      // PERSISTENT SESSION - setCleanSession(false) in the protocol document.
      //
      // clean_session = 0 tells the broker to KEEP this client's session across a
      // disconnect: its subscriptions survive, and QoS 1 messages published while
      // it is briefly away are QUEUED rather than discarded.
      //
      // That matters here more than it would on a wired link. This module sends no
      // PINGREQ - see MQTT_KEEPALIVE_S - so the connection is kept warm only by its
      // own telemetry, and an NB-IoT link that drops between publishes is not
      // noticed until the next send. With a clean session every command sent in
      // that gap is thrown away by the broker and nobody is told. With a persistent
      // session it is waiting when the link comes back.
      //
      // Best-effort: a modem that rejects the setting still gets a working uplink,
      // and refusing to connect over it would trade a fragile downlink for no link
      // at all.
      if (_cmd.outcome != CmdOutcome::NONE) {
        if (consumeOutcome() != CmdOutcome::OK) {
          Serial.println("[nbiot-mqtt] persistent session REJECTED - commands sent while "
                         "briefly offline will be lost");
        }
        _mqttConnectSub = MqttConnectSub::OPEN;
        return;
      }
      char cmd[48];
      snprintf(cmd, sizeof(cmd), "AT+QMTCFG=\"session\",%d,0", MQTT_CLIENT_IDX);
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
        _mqttConnectSub = MqttConnectSub::SUB_DEVICE;  // bring downlink up before going IDLE
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

    // Both subscribes are best-effort: losing downlink costs remote control,
    // but telemetry is this module's actual job and must not be blocked by a
    // rejected SUBSCRIBE. Failures are logged and we continue to IDLE.
    case MqttConnectSub::SUB_DEVICE: {
      if (_cmd.outcome != CmdOutcome::NONE) {
        if (consumeOutcome() != CmdOutcome::OK)
          Serial.println("[nbiot-mqtt] subscribe to this device's RPC topic FAILED - no remote config of Module B");
        _mqttConnectSub = MqttConnectSub::SUB_GATEWAY;
        return;
      }
      char cmd[96];
      snprintf(cmd, sizeof(cmd), "AT+QMTSUB=%d,%u,\"%s\",%d", MQTT_CLIENT_IDX, _subMsgId++,
               MQTT_TOPIC_DEVICE_RPC_SUB, MQTT_DOWNLINK_QOS);
      issueCommand(cmd, NBIOT_TIMEOUT_SOCKET_MS, CmdKind::QMTSUB);
      return;
    }

    case MqttConnectSub::SUB_GATEWAY: {
      if (_cmd.outcome != CmdOutcome::NONE) {
        if (consumeOutcome() != CmdOutcome::OK)
          Serial.println("[nbiot-mqtt] subscribe to the gateway RPC topic FAILED - no remote config of the nodes");
        else
          Serial.println("[nbiot-mqtt] downlink live - announcing the nodes next");
        _announceIdx = 0;
        _mqttConnectSub = MqttConnectSub::ANNOUNCE;
        return;
      }
      char cmd[96];
      snprintf(cmd, sizeof(cmd), "AT+QMTSUB=%d,%u,\"%s\",%d", MQTT_CLIENT_IDX, _subMsgId++,
               MQTT_TOPIC_GATEWAY_RPC, MQTT_DOWNLINK_QOS);
      issueCommand(cmd, NBIOT_TIMEOUT_SOCKET_MS, CmdKind::QMTSUB);
      return;
    }

    case MqttConnectSub::ANNOUNCE: {
      // One v1/gateway/connect per node, one per pass through this state.
      //
      // WITHOUT THIS, RPC TO A NODE NEVER ARRIVES. ThingsBoard only routes a
      // command to a gateway's child device once the gateway has said that device
      // is connected through it. Telemetry needs no such announcement, so the
      // nodes showed up populated and healthy while every command to them was
      // dropped server-side - the failure looked like a firmware bug at this end
      // for two sessions.
      //
      // Re-sent on every MQTT connect, because the announcement is scoped to the
      // session: a reconnect silently un-registers every node otherwise.
      if (_cmd.outcome != CmdOutcome::NONE) {
        consumeOutcome();  // a failed announce is not fatal - telemetry still flows
        _announceIdx++;
      }

      const size_t nodeCount = sizeof(NBIOT_NODE_NAMES) / sizeof(NBIOT_NODE_NAMES[0]);
      if (_announceIdx >= nodeCount) {
        Serial.printf("[nbiot-mqtt] %u node(s) announced - downlink live\n", (unsigned)nodeCount);
        setState(State::IDLE);
        return;
      }

      const char *name = NBIOT_NODE_NAMES[_announceIdx].name;
      _publishPayloadLen = (size_t)snprintf((char *)_publishPayload, sizeof(_publishPayload),
                                            "{\"device\":\"%s\"}", name);
      char header[96];
      snprintf(header, sizeof(header), "AT+QMTPUB=%d,0,0,0,\"v1/gateway/connect\"", MQTT_CLIENT_IDX);
      Serial.printf("[nbiot-mqtt] announcing %s to the gateway\n", name);
      issueCommand(header, NBIOT_TIMEOUT_SEND_MS, CmdKind::QMTPUB, _publishPayload, _publishPayloadLen);
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

  // A DEAD SESSION MUST BE REBUILT HERE, not left for the next publish.
  //
  // handleUrc() clears _mqttConnected when +QMTSTAT arrives and says it will
  // reconnect "on next publish". But a publish only happens when LoRa data
  // arrives, and IDLE is precisely the state where none is arriving. A node that
  // goes hours between readings - or is simply switched off - means no publish,
  // therefore no reconnect, EVER: the state machine sits here polling CSQ with a
  // dead session and the OLED reading "MQTT: caido" indefinitely.
  //
  // Observed 2026-09-14: Module B idle for days, cellular fine (CSQ 31/31,
  // registered), MQTT dead on screen and never retried, because nothing was
  // transmitting to trigger a publish.
  //
  // Gated on the shared backoff so a broker that is genuinely down is retried
  // steadily rather than hammered - MQTT_CONNECT failures feed that same backoff
  // through handleFailureAtLevel().
  if (!_mqttConnected && millis() >= _backoffUntilMs) {
    Serial.println("[nbiot-mqtt] idle with a dead MQTT session - reconnecting now "
                   "(not waiting for a publish)");
    Serial.flush();
    _mqttConnectSub = MqttConnectSub::CLOSE_FIRST;
    setState(State::MQTT_CONNECT);
    return;
  }

  // An RPC response goes out ahead of the telemetry batch - it's what someone
  // is actively waiting on in the ThingsBoard UI, and the batch loses nothing
  // by waiting one more pass.
  if (rpcReplyPending() && _mqttConnected) {
    setState(State::RPC_REPLY);
    return;
  }

  // _flushRequested short-circuits the batch window - see requestFlush(). It is
  // cleared on a successful publish, not here, so a flush that fails to go out is
  // retried rather than quietly forgotten.
  bool batchDue = _ringCount > 0 && (_flushRequested ||
                                      _ringCount >= _batchReadingsTarget ||
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
    _publishPayloadLen = buildGatewayPayload(_publishPayload, sizeof(_publishPayload), MQTT_PUB_CHUNK_READINGS);
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

// Publishes one queued RPC response, then straight back to IDLE. Deliberately
// simpler than PUBLISHING: no retries, no chunking, no ring to pop. If it
// fails, the reply is dropped and ThingsBoard times the RPC out on its own -
// retrying a stale command result is worse than not answering.
void ModemNBIoTMqtt::tickRpcReply() {
  if (!rpcReplyPending()) {
    setState(State::IDLE);
    return;
  }
  if (!_mqttConnected) {
    popRpcReply();  // can't answer; let ThingsBoard time it out
    setState(State::IDLE);
    return;
  }
  if (_cmd.active) return;

  RpcReply &r = _rpcReplies[_rpcReplyHead];

  if (!_rpcReplyInFlight) {
    _publishPayloadLen = strlen(r.payload);
    if (_publishPayloadLen >= sizeof(_publishPayload)) _publishPayloadLen = sizeof(_publishPayload) - 1;
    memcpy(_publishPayload, r.payload, _publishPayloadLen);

    char header[128];
    snprintf(header, sizeof(header), "AT+QMTPUB=%d,0,0,0,\"%s\"", MQTT_CLIENT_IDX, r.topic);
    issueCommand(header, NBIOT_TIMEOUT_SEND_MS, CmdKind::QMTPUB, _publishPayload, _publishPayloadLen);
    _rpcReplyInFlight = true;
    return;
  }

  CmdOutcome outcome = consumeOutcome();
  if (outcome == CmdOutcome::NONE) return;

  if (outcome != CmdOutcome::OK) Serial.println("[nbiot-mqtt] rpc reply publish failed - dropped");
  _rpcReplyInFlight = false;
  popRpcReply();
  // Straight back out if more are waiting - IDLE re-enters this state anyway,
  // but saying so keeps the queue draining in one pass rather than one per tick.
  setState(rpcReplyPending() ? State::RPC_REPLY : State::IDLE);
}

void ModemNBIoTMqtt::tickError() {
  if (millis() < _backoffUntilMs) return;

  // A RETRY MUST ACTUALLY REMOVE POWER, and this did not.
  //
  // The loop was ERROR -> OFF -> enterPowering() -> POWERING -> WAIT_AT -> ERROR.
  // enterPowering() only ASSERTS the EN and channel gates - which were already
  // asserted, because nothing on this path ever dropped them - and then pulses
  // PWRKEY. powerCycle(), the one function that actually cuts VIN, is reachable
  // only from RecoveryLevel::POWER and was never on this path, so
  // _powerCycleSettleUntilMs stayed 0 and tickOff() fell straight through.
  //
  // PWRKEY ON A QUECTEL MODULE TOGGLES. A pulse at a powered-off module turns it
  // on; the same pulse at a RUNNING module turns it OFF. So every "retry" was
  // pulsing PWRKEY at a module whose power state was unknown and, after the
  // first attempt, probably on - meaning the retry was as likely to switch the
  // modem off as to start it. That also explains why the fault sometimes cleared
  // "by itself" after several cycles, and why it hit both boards identically:
  // it is parity, not hardware.
  //
  // Dropping VIN first makes the PWRKEY pulse unambiguous - the module is known
  // to be off, so a pulse can only mean "turn on". NBIOT_POWER_OFF_SETTLE_MS
  // (4s) then gives the rail time to actually fall before it comes back up;
  // tickOff() waits on the timer powerDown() arms.
  powerDown();
  setState(State::OFF);
}

void ModemNBIoTMqtt::onPublishSucceeded() {
  _packetsSent++;
  _flushRequested = false;  // only once it has actually left
  _consecutiveFailures = 0;
  _publishRetries = 0;
  popSentReadings();
  // One batch is drained as several MQTT_PUB_CHUNK_READINGS-sized QMTPUBs -
  // stay in PUBLISHING until the ring is empty, only then back to IDLE. The
  // session is already open, so the next chunk goes out on the following
  // tick with no reconnect. Arrival rate (~1 reading/15s) is far below drain
  // rate (a chunk per tick), so this always terminates.
  if (_ringCount > 0) {
    _publishInFlight = false;
    return;
  }
  setState(State::IDLE);
}

void ModemNBIoTMqtt::onPublishFailed() {
  _packetsFailed++;
  handleFailureAtLevel(RecoveryLevel::PUBLISH, "publish failed");
}

// Writes the decimal digits of a non-negative int64 into out (no NUL), and
// returns the count. Avoids relying on %lld being compiled into this
// toolchain's snprintf (nano newlib often omits it).
static size_t appendI64(char *out, int64_t v) {
  if (v < 0) v = 0;  // epoch ms is always positive here - defensive only
  char tmp[24];
  size_t n = 0;
  uint64_t u = (uint64_t)v;
  do {
    tmp[n++] = (char)('0' + (int)(u % 10));
    u /= 10;
  } while (u);
  for (size_t i = 0; i < n; i++) out[i] = tmp[n - 1 - i];
  return n;
}

// Builds one ThingsBoard Gateway API telemetry payload from up to maxReadings
// of the oldest pending ring entries:
//   {"<node>":[<rec>,<rec>,...], "<otherNode>":[...]}
// Ring entries are grouped by resolved node name (see Config.h's
// nbiotResolveNodeName) rather than assuming a single node.
//
// Two record shapes, and the choice is not free-form - ThingsBoard's gateway
// parser only accepts a "values" wrapper when it's paired with a "ts":
//   _haveNetTime : {"ts":<epoch_ms>,"values":{<keys>}}  - real per-reading time
//   otherwise    : {<keys>}                             - server stamps on receipt
// Each reading's ts is its LoRa-receive moment: netNowMs() rebased through the
// reading's own lastHeardMs (a millis() value), so a 2-min batch of 15s-spaced
// readings lands in ThingsBoard as a real 15s-resolution trend, not 8 points
// at one instant. The flat fallback (no clock from the network) still stores
// fine, just at batch-arrival resolution.
size_t ModemNBIoTMqtt::buildGatewayPayload(uint8_t *out, size_t cap, uint8_t maxReadings) {
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
  if (count > maxReadings) count = maxReadings;

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

      char keys[500];  // +3 keys for chargePct/capMv/alarmState
      snprintf(keys, sizeof(keys),
               "{"
               "\"temp\":%.2f,\"rh\":%.2f,\"pres\":%.2f,\"gas\":%.2f,"
               "\"pm1\":%.2f,\"pm25\":%.2f,\"pm10\":%.2f,\"co2\":%.2f,"
               "\"co\":%.2f,\"coTemp\":%.2f,\"windAngle\":%.2f,\"windSpeed\":%.2f,"
               "\"windValid\":%s,\"rssi\":%d,\"snr\":%d,"
               "\"chargePct\":%d,\"capMv\":%d,\"alarmState\":%d}",
               snap.temp, snap.hum, snap.pres, snap.gas, snap.pm1, snap.pm25, snap.pm10, snap.co2, snap.co,
               snap.coTemp, snap.windAngle, snap.windSpeed, snap.windValid ? "true" : "false", (int)snap.rssi,
               (int)snap.snr, (int)snap.chargePct, (int)snap.capMv, (int)snap.alarmState);

      if (_haveNetTime) {
        int64_t ts = _netEpochMsAtSync + (int64_t)(int32_t)(snap.lastHeardMs - _netSyncLocalMs);
        char tsField[40];
        size_t tn = 0;
        memcpy(tsField, "{\"ts\":", 6);
        tn = 6;
        tn += appendI64(tsField + tn, ts);
        memcpy(tsField + tn, ",\"values\":", 10);
        tn += 10;
        tsField[tn] = '\0';
        append(tsField);
        append(keys);
        append("}");
      } else {
        append(keys);
      }
    }
    append("]");
  }
  append("}");

  _lastPublishCount = count;
  return pos;
}
