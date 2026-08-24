// ---------------------------------------------------------------------------
// bc660k_tcp_test.cpp - one-shot bring-up test, nothing more.
//
// Goal: prove a BC660K-GL can register on 1NCE and deliver a packet to a TCP
// socket on your laptop (via an ngrok tunnel), and that you can see it land
// in nbiot_server.py's log. No LoRa. No state machine, no batching, no
// retries, no power management - all of that comes later, separately.
//
// Deliberately blocking: every AT command is sent, its response logged over
// USB serial, and checked before moving on. The moment something doesn't
// match what's expected, this stops dead with die("<step>", "<why>") naming
// exactly which step failed and what to check - not a generic "error".
//
// NOTHING about the AT sequence below has been bench-verified yet - it's
// built from the Quectel docs. Assume the wiring could be wrong and read the
// die() message when it (probably) stops at the first "UART" step.
// ---------------------------------------------------------------------------

#include <Arduino.h>
#include <string.h>

// ---- modem BC660K-GL wiring ----
// Same pins/UART the production ModemNBIoT class uses (Serial2/UART2) - not
// Serial1, which the LoRa radio owns on this board (GPIO4/5). A clean
// bring-up here directly validates the wiring that code will depend on.
#define MODEM_RX_PIN 8  // ESP RX <- breakout P14 TXD
#define MODEM_TX_PIN 7  // ESP TX -> breakout P14 RXD
#define MODEM_BAUD 115200

// ---- network ----
#define APN "iot.1nce.net"
#define CONTEXT_ID 0  // BC660K-GL uses 0, NOT 1 - the BC66 docs say 1, that's the known trap
#define SOCKET_ID 0

// ngrok's address changes every time the tunnel restarts - this is the only
// place you should need to touch. Left blank on purpose: an empty host fails
// loudly at boot instead of silently trying to open a stale address.
#define SRV_PROTO "TCP"  // "UDP" once there's a real (non-ngrok) server
#define SRV_HOST ""      // e.g. "0.tcp.eu.ngrok.io" - TODO fill in
#define SRV_PORT 0       // e.g. 12345               - TODO fill in

#define SEND_INTERVAL_MS 60000UL

HardwareSerial modem(2);  // UART2

static uint32_t g_seq = 0;

// ---------- the one command path for this file ----------
// Reads modem lines until one contains `expect`, a line contains "ERROR", or
// timeoutMs elapses. Every line is echoed to USB serial as it arrives -
// that's the whole diagnostic story here, no separate log statements needed.
// If `capture` is given, the matching line is copied into it (for the few
// steps that need to look at *what* came back, like CEREG's stat field).
static bool waitFor(const char *expect, uint32_t timeoutMs, char *capture = nullptr, size_t captureCap = 0) {
  char buf[256];
  size_t n = 0;
  uint32_t deadline = millis() + timeoutMs;

  while ((int32_t)(deadline - millis()) > 0) {
    while (modem.available()) {
      char c = (char)modem.read();
      if (c == '\r') continue;
      if (c == '\n') {
        if (n > 0) {
          buf[n] = '\0';
          Serial.print("  < ");
          Serial.println(buf);
          if (strstr(buf, expect)) {
            if (capture && captureCap > 0) {
              strncpy(capture, buf, captureCap - 1);
              capture[captureCap - 1] = '\0';
            }
            return true;
          }
          if (strstr(buf, "ERROR")) return false;
        }
        n = 0;
        continue;
      }
      if (n < sizeof(buf) - 1) buf[n++] = c;
    }
  }
  return false;  // timeout
}

static bool at(const char *cmd, const char *expect = "OK", uint32_t timeoutMs = 5000, char *capture = nullptr,
               size_t captureCap = 0) {
  while (modem.available()) modem.read();  // flush anything stray (a late URC) before issuing
  Serial.print("> ");
  Serial.println(cmd);
  modem.print(cmd);
  modem.print("\r\n");
  return waitFor(expect, timeoutMs, capture, captureCap);
}

static void die(const char *step, const char *why) {
  Serial.println();
  Serial.print("STOPPED at ");
  Serial.print(step);
  Serial.println(":");
  Serial.println(why);
  while (true) delay(1000);
}

// Two-step AT+QISEND: issue the header, wait for the bare '>' data prompt
// (no CRLF - can't go through waitFor's line reader), then write raw bytes
// with no terminator. Used when the one-shot inline-payload form is rejected.
static bool sendPayloadTwoStep(const char *payload) {
  size_t len = strlen(payload);
  char cmd[32];
  snprintf(cmd, sizeof(cmd), "AT+QISEND=%d,%u", SOCKET_ID, (unsigned)len);

  while (modem.available()) modem.read();
  Serial.print("> ");
  Serial.println(cmd);
  modem.print(cmd);
  modem.print("\r\n");

  bool sawPrompt = false;
  uint32_t deadline = millis() + 5000;
  while ((int32_t)(deadline - millis()) > 0 && !sawPrompt) {
    while (modem.available()) {
      if ((char)modem.read() == '>') {
        sawPrompt = true;
        break;
      }
    }
  }
  if (!sawPrompt) {
    Serial.println("  ! never saw the '>' data prompt");
    return false;
  }

  modem.write(reinterpret_cast<const uint8_t *>(payload), len);
  return waitFor("OK", 15000);  // matches "SEND OK" or a plain "OK", both contain "OK"
}

static void forward(const char *payload) {
  Serial.print("[up] ");
  Serial.println(payload);

  size_t len = strlen(payload);
  char cmd[300];
  snprintf(cmd, sizeof(cmd), "AT+QISEND=%d,%u,\"%s\"", SOCKET_ID, (unsigned)len, payload);

  bool ok = at(cmd, "OK", 10000);
  if (!ok) {
    Serial.println("  ! inline AT+QISEND was rejected - falling back to the two-step prompt form");
    ok = sendPayloadTwoStep(payload);
  }
  Serial.println(ok ? "[up] sent" : "[up] FAILED (server down, socket dropped, or tunnel expired?)");
}

void setup() {
  Serial.begin(115200);
  uint32_t waitStart = millis();
  while (!Serial && millis() - waitStart < 3000) delay(10);
  delay(500);

  Serial.println();
  Serial.println("=== BC660K-GL -> laptop over TCP (ngrok) - one-shot bring-up test ===");
  Serial.println("Blocking and verbose on purpose. Not production firmware.");

  if (SRV_HOST[0] == '\0' || SRV_PORT == 0) {
    die("config", "SRV_HOST/SRV_PORT are still blank - put the current ngrok address at the top of this file "
                   "(it changes every time the tunnel restarts).");
  }

  modem.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
  Serial.println("Waiting for the modem to settle after power-up...");
  delay(3000);

  Serial.println("\n--- UART / liveness ---");
  bool gotAt = false;
  for (int attempt = 1; attempt <= 5 && !gotAt; attempt++) {
    Serial.printf("AT attempt %d/5\n", attempt);
    gotAt = at("AT", "OK", 2000);
  }
  if (!gotAt) {
    die("UART", "modem never answered plain AT. Check, in this order: GPIO7 -> breakout P14 RXD and "
                "GPIO8 <- P14 TXD (not swapped), a common GND between the ESP32 and the breakout, that the "
                "breakout's VIN is actually powered from its separate supply, and baud 115200 8N1.");
  }

  if (!at("ATE0")) die("UART", "modem answered plain AT but rejected ATE0 - unexpected; re-check baud/framing.");
  at("AT+QSCLK=0");  // best-effort - only stops the modem sleeping mid-test, not fatal if rejected
  at("AT+CGSN=1", "OK", 2000);  // IMEI - shows up in the transcript above, nothing to gate on

  Serial.println("\n--- SIM ---");
  if (!at("AT+CPIN?", "READY", 5000)) {
    die("SIM", "AT+CPIN? did not report READY. Check the SIM is seated correctly, is the 1NCE SIM, "
               "and is activated on 1NCE's side.");
  }

  Serial.println("\n--- PDP / network config ---");
  if (!at("AT+CFUN=0", "OK", 10000)) die("network config", "AT+CFUN=0 was rejected.");
  char cmd[64];
  snprintf(cmd, sizeof(cmd), "AT+CGDCONT=%d,\"IP\",\"%s\"", CONTEXT_ID, APN);
  if (!at(cmd, "OK", 5000)) {
    die("network config", "AT+CGDCONT was rejected - check CONTEXT_ID (0, not 1, on this specific module) "
                           "and the APN spelling.");
  }
  if (!at("AT+CFUN=1", "OK", 10000)) die("network config", "AT+CFUN=1 was rejected.");
  at("AT+CEREG=1");  // enable the URC too; polling below is the real gate either way

  Serial.println("\n--- registration (can take minutes) ---");
  bool attached = false;
  uint32_t giveUpAt = millis() + 300000UL;  // 5 minutes
  while (!attached && (int32_t)(giveUpAt - millis()) > 0) {
    char cereg[64] = {0};
    // "+CEREG: <n>,<stat>" - matching ",1"/",5" anywhere in the line catches
    // the stat field whether unsolicited reporting (<n>) is 0 or 1, without
    // a full parser - fine for a one-shot bring-up script, not fine as a
    // general CEREG parser (see nbiot/NbiotProtocol.h for that).
    at("AT+CEREG?", "+CEREG", 3000, cereg, sizeof(cereg));
    at("AT+CSQ", "OK", 2000);  // just for visibility while this can take minutes
    attached = strstr(cereg, ",1") != nullptr || strstr(cereg, ",5") != nullptr;
    if (!attached) delay(5000);
  }
  if (!attached) {
    at("AT+QENG=\"servingcell\"", "OK", 5000);  // best-effort extra diagnostics before giving up
    die("network", "never saw CEREG stat 1 or 5 within 5 minutes. Check the antenna is actually connected "
                    "(not just present), the SIM is active on 1NCE's dashboard, and there's NB-IoT coverage "
                    "where you are.");
  }
  Serial.println("REGISTERED.");

  char ip[64] = {0};
  if (!at("AT+CGPADDR=0", "+CGPADDR", 5000, ip, sizeof(ip))) {
    die("network", "CEREG says attached, but AT+CGPADDR=0 gave no answer - the PDP context likely didn't "
                    "actually activate.");
  }

  Serial.println("\n--- socket ---");
  snprintf(cmd, sizeof(cmd), "AT+QIOPEN=%d,%d,\"%s\",\"%s\",%d,0,0", CONTEXT_ID, SOCKET_ID, SRV_PROTO, SRV_HOST,
           SRV_PORT);
  if (!at(cmd, "OK", 10000)) {
    die("socket", "AT+QIOPEN was rejected outright - check SRV_PROTO/SRV_HOST/SRV_PORT at the top of this "
                   "file are the CURRENT ngrok address (it expires on every tunnel restart).");
  }
  char urc[64] = {0};
  if (!waitFor("+QIOPEN:", 30000, urc, sizeof(urc))) {
    die("socket", "AT+QIOPEN was accepted but no +QIOPEN URC arrived within 30s - the server is likely "
                   "unreachable or something (a firewall, a dead tunnel) is dropping the connection.");
  }
  if (!strstr(urc, ",0")) {
    die("socket", "the +QIOPEN URC above reported a nonzero error code - the socket did not open. Common "
                   "causes: wrong SRV_PORT, the ngrok tunnel isn't running/has expired, or SRV_PROTO doesn't "
                   "match the ngrok tunnel type (TCP tunnel needs SRV_PROTO \"TCP\").");
  }

  Serial.println("\n=== socket open - sending a test packet every 60s ===");
  Serial.println("Watch nbiot_server.py's console log or http://<laptop-ip>:8080\n");
}

void loop() {
  static uint32_t lastSend = 0;
  if (millis() - lastSend < SEND_INTERVAL_MS) {
    delay(200);
    return;
  }
  lastSend = millis();

  // Dummy values, same 6-field shape as the real telemetry
  // (TelemetryParser's schema) - nbiot_server.py's FIELDS already expects
  // exactly this: tag,seq,temp_c,rh_pct,co2_ppm,uptime_s.
  float temp = 21.5f + (float)(g_seq % 10) * 0.1f;
  float rh = 45.0f + (float)(g_seq % 5);
  int co2 = 410 + (int)(g_seq % 20);
  uint32_t uptimeS = millis() / 1000;

  char payload[64];
  snprintf(payload, sizeof(payload), "TEST,%lu,%.2f,%.2f,%d,%lu", (unsigned long)g_seq, temp, rh, co2,
           (unsigned long)uptimeS);
  g_seq++;

  forward(payload);
}
