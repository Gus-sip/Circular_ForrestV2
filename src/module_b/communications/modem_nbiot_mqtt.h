#pragma once

#include <Arduino.h>

#include "Config.h"
#include "nbiot/NbiotProtocol.h"
#include "telemetry/SensorSnapshot.h"

// Non-blocking driver for the Quectel BC660K-GL's native MQTT client,
// publishing Module C's telemetry into ThingsBoard over the Gateway API.
// Sibling to ModemNBIoT (src/modem_nbiot.*), not a replacement in place -
// the transport (MQTT vs. raw UDP) differs enough (chained QMTOPEN/QMTCONN
// with no gap allowed, a length+prompt+Ctrl-Z publish form, JSON instead of
// a packed binary frame, URC set) that swapping one method inside ModemNBIoT
// would have meant rewriting most of its transport-facing surface anyway.
// See project memory "project_thingsboard_mqtt_plan" for the reasoning.
//
// The boot/power/registration sequence below (POWERING through ATTACHING)
// is hardware-confirmed as of 2026-08-19 (see project memory
// "project_nbiot_uplink_bringup") and differs from what ModemNBIoT's own
// CONFIG state currently does - that class's sequence was never actually
// validated against real hardware timing (wrong pins, no channel/PWRKEY
// handling, AT+CFUN=1 issued after AT+CPIN? instead of before, AT+CGDCONT
// instead of AT+QCGDEFCONT). Not corrected there since it's being retired
// in favor of this class - corrected only here.
//
// State machine:
//   OFF -> POWERING -> WAIT_AT -> CONFIG -> ATTACHING -> MQTT_CONNECT -> IDLE
//   IDLE <-> PUBLISHING (batch ready, loops back to IDLE either way)
//   any state -> ERROR on an unrecoverable timeout, ERROR -> OFF after a
//   backoff.
//
// MQTT session policy: held open across sends, never proactively pinged
// between batches (see MQTT_KEEPALIVE_S in Config.h for why - reconnecting
// once per batch is far cheaper than a keepalive cadence tight enough to
// matter). If PUBLISHING finds the session not connected (never opened, or
// +QMTSTAT reported it dropped), it detours through MQTT_CONNECT first,
// then publishes - lazy reconnect, not proactive.
//
// Escalating recovery on failure, same shape as ModemNBIoT: retry the
// publish, then reconnect MQTT (QMTOPEN+QMTCONN again), then re-attach to
// the network, and only as an absolute last resort power-cycle the modem.
class ModemNBIoTMqtt {
public:
  enum class State : uint8_t {
    OFF,
    POWERING,
    WAIT_AT,
    CONFIG,
    ATTACHING,
    MQTT_CONNECT,
    IDLE,
    PUBLISHING,
    RPC_REPLY,
    ERROR,
  };

  ModemNBIoTMqtt(HardwareSerial &serial, uint8_t rxPin, uint8_t txPin, uint8_t enPin, uint8_t channelPin,
                 uint8_t pwrkeyPin, uint32_t baud = NBIOT_BAUD);

  // Sets pin modes, leaves the modem powered off. Call once from setup().
  void begin();

  // Drives the state machine - does a small bounded unit of work and
  // returns, never blocks. Call every loop() iteration.
  void tick();

  // Pushes one reading into the batching ring. Never blocks. If the ring is
  // already full, the oldest reading is dropped and droppedCount() goes up.
  void enqueue(const SensorSnapshot &snap);

  // ---------- Observability - surfaced on the dashboard ----------
  State state() const { return _state; }
  // The modem UART is deliberately NOT held open across power cycles - a driven
  // TX pin back-powers the module through its protection diodes and prevents it
  // ever cold-starting. See the comment on enterPowering().
  void releaseUart();
  void startUart();

  const char *stateName() const;
  // Names a state without needing an instance in that state - setState() must be
  // able to print the state it is ENTERING, before _state is assigned.
  static const char *stateNameOf(State st);
  const char *lastError() const { return _lastError; }
  int rssiDbm() const { return _rssiDbm; }
  bool attached() const { return _attached; }
  uint32_t attachedUptimeMs() const;
  bool mqttConnected() const { return _mqttConnected; }
  uint32_t packetsSent() const { return _packetsSent; }
  uint32_t packetsFailed() const { return _packetsFailed; }
  uint32_t packetsDropped() const { return _packetsDropped; }
  uint32_t mqttReconnects() const { return _mqttReconnects; }
  uint16_t batchReadingsTarget() const { return _batchReadingsTarget; }
  uint32_t batchSecondsTarget() const { return _batchSecondsTarget; }
  uint8_t ringCount() const { return _ringCount; }
  bool haveNetTime() const { return _haveNetTime; }

  // ---------- Runtime config, settable from Module A ----------
  // Both clamp to the MQTT_BATCH_*_MIN/MAX bounds in Config.h and return the
  // value actually applied, so a clamped command is visible to the caller (and
  // gets reported back in the RPC response) instead of silently assumed away.
  uint32_t setBatchSeconds(uint32_t seconds);
  uint16_t setBatchReadings(uint16_t readings);

  // ---------- Downlink relay to a child node ----------
  // A node-directed RPC can't be delivered on arrival: Module C only listens
  // for ~2s after each of its own transmissions. So one command is parked here
  // and main.cpp fires it over LoRa the instant it sees that node's uplink.
  // Only one is held at a time - a second arriving before the first is
  // answered is rejected with a "busy" RPC error rather than queued, since a
  // deep queue of stale config pushes helps nobody.
  bool nodeCommandPending() const { return _nodeCmd.pending && !_nodeCmd.awaitingAck; }
  const char *nodeCommandDevice() const { return _nodeCmd.device; }
  const char *nodeCommandCfg() const { return _nodeCmd.cfg; }
  // Called by main.cpp once the CFG string has gone out over LoRa.
  void onNodeCommandDelivered();
  // Called by main.cpp when an "ACK,..." packet comes back from the node.
  // Publishes the RPC response and clears the slot.
  void onNodeCommandAck(const char *ackPayload);

private:
  enum class CmdKind : uint8_t { PLAIN, QMTOPEN, QMTCONN, QMTPUB, QMTSUB };
  enum class CmdOutcome : uint8_t { NONE, OK, ERR, TIMEOUT };
  enum class ConfigStep : uint8_t {
    ATE0,
    CMEE,
    CTZU,  // AT+CTZU=1 - auto network time update, so AT+CCLK? works (best-effort)
    QSCLK,
    CFUN_ON_1,
    CPIN,
    CFUN_OFF,
    QCGDEFCONT,
    CFUN_ON_2,
    DONE
  };
  // SUB_DEVICE/SUB_GATEWAY run after CONN so downlink is live before IDLE.
  // Both are best-effort: a failed subscribe loses remote control but must not
  // stop telemetry, which is the module's actual job.
  enum class MqttConnectSub : uint8_t { CLOSE_FIRST, KEEPALIVE_CFG, OPEN, CONN, SUB_DEVICE, SUB_GATEWAY };
  enum class RecoveryLevel : uint8_t { PUBLISH, MQTT_CONNECT, ATTACH };

  struct PendingCmd {
    bool active = false;
    CmdKind kind = CmdKind::PLAIN;
    uint32_t sentAtMs = 0;
    uint32_t timeoutMs = 0;
    bool sawOk = false;           // QMTOPEN/QMTCONN/QMTPUB: immediate OK ack seen, still waiting on the async URC
    bool awaitingPrompt = false;  // QMTPUB: waiting on the bare '>' data prompt
    const uint8_t *sendPayload = nullptr;
    size_t sendPayloadLen = 0;
    char infoLine[96] = {0};
    bool hasInfoLine = false;
    CmdOutcome outcome = CmdOutcome::NONE;
  };

  // ---------- The one command path ----------
  void issueCommand(const char *cmd, uint32_t timeoutMs, CmdKind kind = CmdKind::PLAIN,
                     const uint8_t *sendPayload = nullptr, size_t sendPayloadLen = 0);
  void pumpSerial();
  void handleLine(const char *line, size_t len);
  void handleUrc(const char *line, size_t len);
  void completeCommand(CmdOutcome outcome);
  void checkCommandTimeout();
  CmdOutcome consumeOutcome();

  // ---------- State machine ----------
  void setState(State s);
  void enterPowering();
  void enterError(const char *reason);
  void powerCycle();
  void handleFailureAtLevel(RecoveryLevel level, const char *reason);
  void applyBackoffAndSetState(State s);
  uint32_t computeBackoff(uint32_t attempt) const;
  void setLastError(const char *msg);

  // ---------- Downlink handling ----------
  // Routes one inbound MQTT message by topic: the gateway's own RPC topic
  // means "reconfigure Module B", the gateway RPC topic means "relay this to
  // a child node".
  void handleInboundMqtt(const NbiotProtocol::MqttMessage &msg);
  void handleOwnRpc(const NbiotProtocol::MqttMessage &msg);
  void handleGatewayRpc(const NbiotProtocol::MqttMessage &msg);
  // Applies a flat {"KEY":value,...} params object to Module B's own runtime
  // config, writing what was actually applied (post-clamp) into applied.
  bool applyOwnConfig(const char *params, char *applied, size_t cap);
  // Queues an RPC response for RPC_REPLY to publish.
  void queueRpcReply(const char *topic, const char *payload);
  void tickNodeCommandTimeout();

  void tickOff();
  void tickPowering();
  void tickWaitAt();
  void tickConfig();
  void tickAttaching();
  void tickMqttConnect();
  void tickIdle();
  void tickPublishing();
  void tickRpcReply();
  void tickError();

  void onPublishSucceeded();
  void onPublishFailed();

  // ---------- Ring buffer ----------
  const SensorSnapshot &ringPeek(uint8_t i) const;
  void popSentReadings();
  // Builds one AT+QMTPUB payload from up to maxReadings oldest ring entries
  // (grouped by node name). Sets _lastPublishCount to how many it actually
  // encoded. Emits {"ts":<ms>,"values":{...}} per reading when network time
  // is known, else the flat no-"ts" form.
  size_t buildGatewayPayload(uint8_t *out, size_t cap, uint8_t maxReadings);

  // Epoch-ms "now" from the last AT+CCLK? sync + elapsed millis(). Only
  // meaningful when _haveNetTime.
  int64_t netNowMs() const;

  // ---------- Fixed config ----------
  HardwareSerial &_serial;
  uint8_t _rxPin, _txPin, _enPin, _channelPin, _pwrkeyPin;
  uint32_t _baud;
  bool _uartStarted = false;

  // ---------- Command engine ----------
  PendingCmd _cmd;
  char _lineBuf[96];
  uint8_t _lineLen = 0;
  bool _sawBootUrc = false;

  // ---------- State machine bookkeeping ----------
  State _state = State::OFF;
  uint32_t _stateEnteredMs = 0;
  uint32_t _backoffUntilMs = 0;
  uint32_t _powerCycleSettleUntilMs = 0;
  uint32_t _consecutiveFailures = 0;
  char _lastError[64] = "none";

  bool _pwrkeyReleased = false;  // POWERING sub-timing: pulse low, then release high, non-blocking
  uint32_t _lastAtPingMs = 0;

  ConfigStep _configStep = ConfigStep::ATE0;
  uint32_t _configStepEnteredMs = 0;  // lets QCGDEFCONT wait out CFUN=0's SIM-interface settle, non-blocking

  bool _cgpaddrChecked = false;
  bool _csqCheckedAfterAttach = false;  // one AT+CSQ right after attach - see tickAttaching()
  bool _cclkChecked = false;            // one AT+CCLK? right after the CSQ check - see tickAttaching()
  uint32_t _lastCeregPollMs = 0;
  uint8_t _attachRetries = 0;

  // ---------- Network time (per-reading uplink timestamps) ----------
  // Read once per attach via AT+CCLK? (needs AT+CTZU=1). _netEpochMsAtSync is
  // epoch-ms at the moment _netSyncLocalMs (a millis() value) was captured;
  // netNowMs() extrapolates from there. Left as "no time" if the network
  // never provides a plausible clock - the uplink then omits "ts".
  bool _haveNetTime = false;
  int64_t _netEpochMsAtSync = 0;
  uint32_t _netSyncLocalMs = 0;

  MqttConnectSub _mqttConnectSub = MqttConnectSub::KEEPALIVE_CFG;
  uint8_t _mqttConnectRetries = 0;
  bool _mqttConnected = false;
  uint32_t _mqttReconnects = 0;

  uint32_t _lastCsqPollMs = 0;  // dashboard RSSI refresh while IDLE - no PSM/sleep management in this class yet

  bool _publishInFlight = false;
  uint8_t _publishRetries = 0;
  uint8_t _publishPayload[MQTT_PAYLOAD_MAX_BYTES];
  size_t _publishPayloadLen = 0;
  uint8_t _lastPublishCount = 0;

  // ---------- Downlink ----------
  // One node-directed command in flight at a time - see nodeCommandPending().
  struct NodeCommand {
    bool pending = false;      // occupied
    bool awaitingAck = false;  // already sent over LoRa, waiting for the node's ACK
    char device[24] = {0};     // ThingsBoard child-device name, e.g. "NodoC-1"
    char cfg[128] = {0};       // "CFG,INTERVAL=300,BMV080=0" as Module C expects it
    long rpcId = 0;
    uint32_t queuedMs = 0;
  };
  NodeCommand _nodeCmd;

  // One queued RPC response, published by RPC_REPLY then cleared.
  bool _rpcReplyPending = false;
  char _rpcReplyTopic[NbiotProtocol::kMqttTopicLen] = {0};
  char _rpcReplyPayload[224] = {0};
  bool _rpcReplyInFlight = false;

  uint16_t _subMsgId = 1;  // AT+QMTSUB message id, incremented per subscribe
  uint32_t _downlinksReceived = 0;

  // ---------- Ring buffer of pending telemetry ----------
  SensorSnapshot _ring[NBIOT_RING_CAPACITY];
  uint8_t _ringHead = 0;
  uint8_t _ringCount = 0;
  uint32_t _oldestPendingMs = 0;

  // ---------- Batching (fixed for now - no downlink CFG yet, see project
  // memory "project_thingsboard_mqtt_plan" - the v1/gateway/attributes and
  // v1/gateway/rpc topics are the natural home for this later, deliberately
  // not built yet). PUBLISHING fires when the ring hits _batchReadingsTarget
  // OR _batchSecondsTarget elapses since the oldest pending reading, then
  // drains the whole ring in MQTT_PUB_CHUNK_READINGS-sized QMTPUBs. ----------
  uint16_t _batchReadingsTarget = MQTT_BATCH_MAX_READINGS;
  uint32_t _batchSecondsTarget = MQTT_BATCH_SECONDS;

  // ---------- Observability ----------
  bool _attached = false;
  uint32_t _attachedSinceMs = 0;
  int _rssiDbm = 0;
  uint32_t _packetsSent = 0;
  uint32_t _packetsFailed = 0;
  uint32_t _packetsDropped = 0;
};
