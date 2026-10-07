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

  // Publish at the next opportunity instead of waiting for the batch window.
  //
  // Batching exists to keep the NB-IoT link and the SIM's data budget sane, and
  // for telemetry that is right. A fire is not telemetry: a node reporting one
  // every 5s while Module B sits on the packet for the rest of a 60s window is a
  // minute of silence at the only moment the system exists for.
  void requestFlush() { _flushRequested = true; }

  // Publishes one commandLog record: the outcome of a command that was already
  // acknowledged "enviado". Uses the same single publish slot as RPC replies.
  void publishCommandLog(long requestId, const char *status);

  // Same record plus a "detail" field: the keys the node actually applied, or the
  // reason it refused. The protocol document defines requestId/status/timestamp;
  // this adds to that rather than changing it, so a dashboard reading only the
  // documented fields is unaffected.
  // `device` additionally mirrors the record onto that node's own telemetry, so
  // it is readable whether a dashboard watches the gateway or the node.
  void publishCommandLogApplied(long requestId, const char *status, const char *detail,
                                const char *device = nullptr);

private:
  void handleConfigurarModuloC(const NbiotProtocol::MqttMessage &msg, const char *idStr);

public:

  // Stamp a node's CURRENT fire state onto readings already waiting to go out.
  //
  // Without this the alarm flush publishes a stale state, which makes it theatre.
  // A node transmits its readings FIRST and its STAT about two seconds later, so
  // by the time Module B learns the node has gone into alarm, that cycle's reading
  // is already queued carrying the PREVIOUS state. requestFlush() would then rush
  // out a packet saying alarmState 0 - the one packet whose whole purpose was to
  // say otherwise - and the true state would arrive on the next batch, up to a
  // minute later.
  //
  // Confirmed on hardware 2026-09-28: all three nodes published alarmState -1 on
  // one batch and 0 on the next, one cycle behind their STATs throughout.
  void applyAlarmToQueued(uint16_t addr, int8_t state) {
    for (uint8_t i = 0; i < _ringCount; i++) {
      if (_ring[i].senderAddr == addr) _ring[i].alarmState = state;
    }
  }

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

  bool _flushRequested = false;
  uint16_t setBatchReadings(uint16_t readings);

  // ---------- Downlink relay to a child node ----------
  // A node-directed RPC can't be delivered on arrival: Module C only listens
  // for ~2s after each of its own transmissions. So one command is parked here
  // and main.cpp fires it over LoRa the instant it sees that node's uplink.
  // Only one is held at a time - a second arriving before the first is
  // answered is rejected with a "busy" RPC error rather than queued, since a
  // deep queue of stale config pushes helps nobody.
  // A PENDING LIST, as the protocol document specifies ("guardar comando en lista
  // pendiente"), not a single slot.
  //
  // A node can only hear during the ~2s window after it uplinks, so a command may
  // wait minutes for delivery. With one slot, every command queued during that
  // wait was refused as "busy" - and the Module A dashboard has one row per
  // magnitude, so setting temperature and then humidity meant the second was
  // rejected for a reason the operator could not see or act on.
  //
  // Addressed by device throughout: a command for C-2 must never block one for
  // C-3 just because C-2 has not woken up yet.
  const char *nodeCommandCfgFor(const char *device) const {
    const int i = deliverableIndexFor(device);
    return i < 0 ? nullptr : _nodeCmds[i].cfg;
  }
  // Called by main.cpp once that device's CFG string has gone out over LoRa.
  void onNodeCommandDelivered(const char *device);
  // Called by main.cpp when an "ACK,..." packet comes back from a node. Reports
  // the outcome and frees that device's slot.
  void onNodeCommandAck(const char *device, const char *ackPayload);

  // The node understood the command and applied NOTHING. Reported as the
  // protocol's "error" state immediately, rather than being left to time out as
  // though the node were unreachable.
  void onNodeCommandNack(const char *device, const char *reason);

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
  // ANNOUNCE comes last, after both subscriptions, and is not optional.
  //
  // ThingsBoard will not route an RPC to a gateway's child device until the
  // gateway has announced it on v1/gateway/connect. Telemetry does NOT require
  // this - publishing to v1/gateway/telemetry auto-creates the device - which is
  // exactly why the omission was invisible: the nodes appeared in ThingsBoard
  // with live readings while every command sent to them died on the server,
  // before it ever reached the modem.
  enum class MqttConnectSub : uint8_t {
    CLOSE_FIRST, KEEPALIVE_CFG, SESSION_CFG, OPEN, CONN, SUB_DEVICE, SUB_GATEWAY, ANNOUNCE
  };
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

  // The electrical half of a power cycle: TX released, VIN and the channel gate
  // dropped, PWRKEY parked inactive, and the off-settle timer armed. Shared by
  // powerCycle() and by the ERROR->OFF retry path, which MUST remove power
  // rather than just pulse PWRKEY again - see tickError().
  void powerDown();
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
  // MUST hold a whole +QMTRECV line, payload included.
  //
  // This was 96 and it silently broke every downlink. A gateway RPC line is
  //
  //   +QMTRECV: 0,0,"v1/devices/me/rpc/request/123",{...json...}
  //
  // and the umbral command's JSON alone runs past 150 characters, so the line was
  // always longer than the buffer. Worse than truncation: the overflow path reset
  // the buffer and kept going, so the FRONT of the message was discarded and the
  // TAIL was handed to the parser as though it were a complete line. parseQmtrecv
  // failed on it, and the leftovers surfaced as
  //
  //   [nbiot-mqtt] urc: n":50,"umbral_alarm_off":55,"umbral_alarm_on":60}}"
  //
  // which looks like line noise and is in fact the end of a command. Every
  // "nothing is arriving" conclusion drawn over several days came from grepping
  // for the success message, which only prints on a successful parse.
  //
  // Sized for the largest thing that can arrive: topic (72) + payload (320) plus
  // the +QMTRECV envelope, rounded up.
  static const size_t kLineBufLen = 512;
  char _lineBuf[kLineBufLen];
  bool _lineOverflowed = false;
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
  uint8_t _announceIdx = 0;  // which node is being announced on v1/gateway/connect
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
    uint8_t attempts = 0;      // delivery attempts so far - reported, and retried
    uint32_t lastSentMs = 0;   // when the last attempt went out, for the retry gap

    // Which protocol asked for this, and therefore how its OUTCOME is reported.
    //
    //   false - the gateway-RPC path: the RPC is held open and answered with the
    //           result, so ThingsBoard blocks until the node replies or Tmax.
    //   true  - the agreed A->C protocol: the RPC was already answered "enviado"
    //           the moment it arrived, and the outcome goes out later as a
    //           commandLog telemetry record carrying the same requestId.
    //
    // The distinction has to be carried on the command itself: by the time the ACK
    // or the timeout arrives, the original message is long gone.
    bool viaCommandLog = false;
  };
  // Six: enough for every magnitude on one node, or two each across three nodes.
  // A full queue is REPORTED, never silently dropped.
  // The CFG string is built here before a free slot is chosen, so a command that
  // turns out to be unqueueable never half-writes into a real slot.
  char _pendingCfgScratch[128] = {0};

  static const uint8_t kNodeCmdQueue = 6;
  NodeCommand _nodeCmds[kNodeCmdQueue];

  int undeliveredIndexFor(const char *device) const {
    for (uint8_t i = 0; i < kNodeCmdQueue; i++) {
      if (_nodeCmds[i].pending && !_nodeCmds[i].awaitingAck &&
          strcmp(_nodeCmds[i].device, device) == 0) {
        return (int)i;
      }
    }
    return -1;
  }

  // A SLOT THIS NODE SHOULD BE SENT RIGHT NOW - which includes one already sent
  // and still unacknowledged.
  //
  // The node uplinking is itself the evidence that a previous attempt failed: if
  // it had received and applied the command it would have ACKed inside that
  // window, and the slot would already be free. So an unacknowledged command is
  // re-sent on the next window rather than sat on until the deadline.
  //
  // This is what the 15-minute deadline was always documented to mean - "3 missed
  // windows at 5 min" - except nothing ever retried in those windows. One lost
  // frame cost the full fifteen minutes and then reported a timeout for a command
  // that had been delivered to a healthy node. Seen on requestId 7, 2026-10-05.
  //
  // The gap guard is load-bearing, not caution: see DOWNLINK_RETRY_MIN_GAP_MS.
  // Cuantos huecos quedan. Una accion global necesita uno POR NODO, y hay que
  // saberlo ANTES de encolar nada: encolar la mitad de un comando que va a todos
  // los nodos deja el sistema diciendo que lo aplico cuando no es cierto.
  uint8_t freeNodeCmdSlots() const {
    uint8_t n = 0;
    for (uint8_t i = 0; i < kNodeCmdQueue; i++) {
      if (!_nodeCmds[i].pending) n++;
    }
    return n;
  }

  int deliverableIndexFor(const char *device) const {
    // ONE COMMAND PER NODE PER WINDOW.
    //
    // main.cpp runs its downlink hook on every frame received, and a node sends
    // several per window - telemetry, then its ACK, then STAT. Checking only
    // whether THIS slot was recently sent meant the second queued command went
    // out on the node's ACK frame and the third on its STAT frame, by which
    // point the node had closed its listen window and was transmitting. Both
    // were lost and had to be retried a window later.
    //
    // Observed with five commands queued at once on 2026-10-05: it drained one
    // per window anyway, but burned two frames each time to do it.
    //
    // So if ANYTHING for this node went out within the gap, the node is mid-
    // transmission and deaf: hold the rest.
    for (uint8_t i = 0; i < kNodeCmdQueue; i++) {
      const NodeCommand &nc = _nodeCmds[i];
      if (!nc.pending || strcmp(nc.device, device) != 0) continue;
      if (nc.awaitingAck && millis() - nc.lastSentMs < DOWNLINK_RETRY_MIN_GAP_MS) return -1;
    }

    // OLDEST FIRST, so commands reach the node in the order Module A sent them.
    // Slots are handed out by first-free, so slot order is not arrival order -
    // and when two commands set the same threshold to different values, the one
    // that wins must be the one sent last. Age rather than the raw stamp, so a
    // millis() rollover compares correctly.
    int best = -1;
    uint32_t bestAge = 0;
    const uint32_t now = millis();
    for (uint8_t i = 0; i < kNodeCmdQueue; i++) {
      const NodeCommand &nc = _nodeCmds[i];
      if (!nc.pending || strcmp(nc.device, device) != 0) continue;
      const uint32_t age = now - nc.queuedMs;
      if (best < 0 || age > bestAge) {
        best = (int)i;
        bestAge = age;
      }
    }
    return best;
  }

  int awaitingAckIndexFor(const char *device) const {
    for (uint8_t i = 0; i < kNodeCmdQueue; i++) {
      if (_nodeCmds[i].pending && _nodeCmds[i].awaitingAck &&
          strcmp(_nodeCmds[i].device, device) == 0) {
        return (int)i;
      }
    }
    return -1;
  }

  int freeNodeCmdSlot() const {
    for (uint8_t i = 0; i < kNodeCmdQueue; i++) {
      if (!_nodeCmds[i].pending) return (int)i;
    }
    return -1;
  }

  // One queued RPC response, published by RPC_REPLY then cleared.
  // A QUEUE, NOT ONE SLOT.
  //
  // This was a single slot that logged "dropping the older one" and threw a
  // message away. Nothing had hit it yet only by luck - the modem happened to
  // drain each reply before the next was queued - but two commands arriving in
  // one tick is normal (requests 9 and 10 did exactly that on 2026-10-05), and
  // the one discarded would have been an "enviado" that Module A was waiting on.
  //
  // Eight deep. Each outcome now produces THREE publishes - commandLog on the
  // gateway (what the protocol document specifies), the same record mirrored on
  // the node, and the node's attributes - and two commands can land in one tick,
  // as requests 9 and 10 did on 2026-10-05.
  static const uint8_t kRpcReplyQueue = 8;
  struct RpcReply {
    char topic[NbiotProtocol::kMqttTopicLen] = {0};
    char payload[512] = {0};  // the node mirror carries commandLog + one field per threshold
  };
  RpcReply _rpcReplies[kRpcReplyQueue];
  uint8_t _rpcReplyHead = 0;   // index of the next one to send
  uint8_t _rpcReplyCount = 0;

  bool rpcReplyPending() const { return _rpcReplyCount > 0; }
  void popRpcReply() {
    if (_rpcReplyCount == 0) return;
    _rpcReplyHead = (uint8_t)((_rpcReplyHead + 1) % kRpcReplyQueue);
    _rpcReplyCount--;
  }
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
