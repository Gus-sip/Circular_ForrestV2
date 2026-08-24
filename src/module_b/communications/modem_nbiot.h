#pragma once

#include <Arduino.h>

#include "Config.h"
#include "nbiot/NbiotProtocol.h"
#include "telemetry/SensorSnapshot.h"

// Non-blocking driver for the Quectel BC660K-GL NB-IoT uplink ("Module B"'s
// cellular backhaul). Owns its own UART (must NOT be Serial1 - see the
// NB-IoT wiring comment in Config.h for why) plus a RAM ring of pending
// telemetry, and drives everything through tick(), which must be called
// every loop() iteration and never blocks - no delay(), no busy-waits, every
// state below has a bounded timeout.
//
// State machine:
//   OFF -> POWERING -> WAIT_AT -> CONFIG -> ATTACHING -> SOCKET_OPEN -> IDLE
//   IDLE <-> SENDING (batch ready, loops back to IDLE either way)
//   any state -> ERROR on an unrecoverable timeout, ERROR -> OFF after a
//   backoff (so a transient issue eventually retries on its own; nothing is
//   stuck forever, but nothing hammers the modem forever either)
//
// Escalating recovery on a send failure: retry the send, then close+reopen
// the socket, then re-attach, and only as an absolute last resort power
// cycle (EN pin) - a cold boot forces a full network attach, the slowest and
// most expensive thing this class ever does, so it must never be the first
// response to one bad send. Each rung applies capped exponential backoff.
class ModemNBIoT {
public:
  enum class State : uint8_t {
    OFF,
    POWERING,
    WAIT_AT,
    CONFIG,
    ATTACHING,
    SOCKET_OPEN,
    IDLE,
    SENDING,
    ERROR,
  };

  // Fired once a downlink CFG has been parsed, clamped, and applied.
  using ConfigAppliedCallback = void (*)(uint8_t version, uint16_t batchReadings, uint32_t batchSeconds);

  ModemNBIoT(HardwareSerial &serial, uint8_t rxPin, uint8_t txPin, uint8_t enPin, uint8_t rstPin,
             uint32_t baud = NBIOT_BAUD);

  // Sets pin modes, leaves the modem powered off. Doesn't touch the UART
  // yet - that happens on the way into POWERING. Call once from setup().
  void begin();

  // Drives the state machine. Does at most a small bounded unit of work
  // (drain whatever bytes are already sitting in the UART buffer, check a
  // timeout, issue the next command) and returns - never blocks. Call every
  // loop() iteration, same as radio.poll() and server.handleClient().
  void tick();

  // Pushes one reading into the batching ring. Never blocks. If the ring is
  // already full (an upload hasn't drained it in time), the oldest reading
  // is dropped and droppedCount() goes up rather than growing unbounded or
  // stalling the caller.
  void enqueue(const SensorSnapshot &snap);

  void setConfigAppliedCallback(ConfigAppliedCallback cb) { _configAppliedCb = cb; }

  // ---------- Observability - surfaced on the dashboard ----------
  State state() const { return _state; }
  const char *stateName() const;
  const char *lastError() const { return _lastError; }
  int rssiDbm() const { return _rssiDbm; }
  bool attached() const { return _attached; }
  uint32_t attachedUptimeMs() const;
  uint32_t packetsSent() const { return _packetsSent; }
  uint32_t packetsFailed() const { return _packetsFailed; }
  uint32_t packetsDropped() const { return _packetsDropped; }
  uint8_t appliedCfgVersion() const { return _appliedCfgVersion; }
  uint16_t batchReadingsTarget() const { return _batchReadingsTarget; }
  uint32_t batchSecondsTarget() const { return _batchSecondsTarget; }
  uint8_t ringCount() const { return _ringCount; }

private:
  enum class CmdKind : uint8_t { PLAIN, QIOPEN, QISEND };
  enum class CmdOutcome : uint8_t { NONE, OK, ERR, TIMEOUT };
  enum class ConfigStep : uint8_t { ATE0, CPIN, CGDCONT, CFUN, DONE };
  enum class SocketSub : uint8_t { CLOSE_FIRST, OPEN };
  enum class IdleSub : uint8_t { NORMAL, POLLING_CSQ, SLEEPING, WAKING };
  enum class RecoveryLevel : uint8_t { SEND, SOCKET, ATTACH };

  struct PendingCmd {
    bool active = false;
    CmdKind kind = CmdKind::PLAIN;
    uint32_t sentAtMs = 0;
    uint32_t timeoutMs = 0;
    bool sawOk = false;          // QIOPEN: immediate OK ack seen, still waiting on the async +QIOPEN URC
    bool awaitingPrompt = false; // QISEND: waiting on the bare '>' data prompt
    const uint8_t *sendPayload = nullptr;
    size_t sendPayloadLen = 0;
    char infoLine[96] = {0};     // last non-terminal line seen - the "value" line for queries
    bool hasInfoLine = false;
    CmdOutcome outcome = CmdOutcome::NONE;
  };

  // ---------- The one command path ----------
  // Every AT command in this class goes through issueCommand() to send it
  // and handleLine()/checkCommandTimeout() to resolve it to OK/ERR/TIMEOUT
  // (QIOPEN's real result is a URC that arrives after its OK ack; QISEND's
  // is "SEND OK"/"SEND FAIL" after a raw '>' data prompt - both still funnel
  // through this same pair of functions via `kind`, not a separate path).
  void issueCommand(const char *cmd, uint32_t timeoutMs, CmdKind kind = CmdKind::PLAIN,
                     const uint8_t *sendPayload = nullptr, size_t sendPayloadLen = 0);
  void pumpSerial();                          // drains _serial, feeds handleLine()/the prompt watcher
  void handleLine(const char *line, size_t len);
  void handleUrc(const char *line, size_t len);
  void completeCommand(CmdOutcome outcome);
  void checkCommandTimeout();
  CmdOutcome consumeOutcome();                // reads-and-clears _cmd.outcome

  // ---------- Downlink (ACK/CFG) ----------
  void serviceDownlinkIfPending();  // issues AT+QIRD when a +QIURC "recv" is waiting and nothing else is in flight
  void applyDownlink(const char *text);

  // ---------- State machine ----------
  void setState(State s);  // the only place _state changes - resets per-state fields on entry
  void enterPowering();
  void enterError(const char *reason);
  void powerCycle();
  void handleFailureAtLevel(RecoveryLevel level, const char *reason);
  void applyBackoffAndSetState(State s);
  uint32_t computeBackoff(uint32_t attempt) const;
  void setLastError(const char *msg);

  void tickOff();
  void tickPowering();
  void tickWaitAt();
  void tickConfig();
  void tickAttaching();
  void tickSocketOpen();
  void tickIdle();
  void tickSending();
  void tickError();

  void onSendSucceeded();
  void onSendFailed();

  // ---------- Ring buffer ----------
  const SensorSnapshot &ringPeek(uint8_t i) const;  // i=0 is the oldest
  void popSentReadings();
  size_t buildBatchPayload(uint8_t *out, size_t cap);

  // ---------- Fixed config ----------
  HardwareSerial &_serial;
  uint8_t _rxPin, _txPin, _enPin, _rstPin;
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
  char _lastError[48] = "none";

  uint32_t _lastAtPingMs = 0;

  ConfigStep _configStep = ConfigStep::ATE0;

  bool _cgpaddrChecked = false;
  uint32_t _lastCeregPollMs = 0;
  uint8_t _attachRetries = 0;

  SocketSub _socketSub = SocketSub::OPEN;
  uint8_t _socketRetries = 0;

  IdleSub _idleSub = IdleSub::NORMAL;
  uint32_t _idleSinceMs = 0;
  uint32_t _lastCsqPollMs = 0;

  bool _sendInFlight = false;
  uint8_t _sendRetries = 0;
  uint8_t _sendPayload[NBIOT_PAYLOAD_MAX_BYTES];
  size_t _sendPayloadLen = 0;
  uint8_t _lastSendCount = 0;
  uint16_t _uplinkSeq = 0;

  bool _downlinkPending = false;
  bool _downlinkReadInFlight = false;
  int _downlinkPendingLen = 0;

  // ---------- Ring buffer of pending telemetry ----------
  SensorSnapshot _ring[NBIOT_RING_CAPACITY];
  uint8_t _ringHead = 0;
  uint8_t _ringCount = 0;
  uint32_t _oldestPendingMs = 0;

  // ---------- Config (defaults, retunable within Config.h's clamp range by downlink CFG) ----------
  uint16_t _batchReadingsTarget = NBIOT_BATCH_DEFAULT_READINGS;
  uint32_t _batchSecondsTarget = NBIOT_BATCH_DEFAULT_SECONDS;
  uint8_t _appliedCfgVersion = 0;
  ConfigAppliedCallback _configAppliedCb = nullptr;

  // ---------- Observability ----------
  bool _attached = false;
  uint32_t _attachedSinceMs = 0;
  int _rssiDbm = 0;
  uint32_t _packetsSent = 0;
  uint32_t _packetsFailed = 0;
  uint32_t _packetsDropped = 0;
};
