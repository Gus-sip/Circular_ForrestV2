/*
 * Calypso ULP PRO wind sensor probe - first test on the fabbed PCB.
 *
 * This sensor has never worked on this board, and the most likely reason is now
 * known to be nothing to do with the sensor: it runs off the 5V rail, and GPIO11
 * (the 5V gate) is ACTIVE-HIGH while the firmware drove it LOW for weeks - see
 * pins.h. So the rail it needs was switched off the entire time. Expect it to work
 * now; if it does not, this probe separates the remaining possibilities.
 *
 * Unlike every other sensor here, the Calypso is NOT request/response - it streams
 * NMEA continuously and unprompted at 38400. That makes passive listening the
 * primary test: we should see bytes without sending anything at all. If a pin/baud
 * combination is right, sentences appear on their own.
 *
 *   $IIMWV,<angle>,R,<speed>,N,A*<cs>
 *
 * Bench history worth not re-deriving: a sweep across GPIO5/6/8/9 x baud
 * 4800-115200 found a valid sentence ONLY on RX=GPIO8 @ 38400, and the unit was
 * silent on 3.3V and streamed the moment it moved to 5V. pins.h reflects that.
 *
 * Wiring:
 *   VCC    -> 5V rail (NOT 3.3V - it is silent there)
 *   GND    -> shared ground
 *   green  -> GPIO8  (sensor TX -> ESP RX)
 *   yellow -> GPIO9  (ESP TX, unused)
 */

#include <Arduino.h>
#include "pins.h"
#include "Config.h"

#define windSerial Serial1

// 38400 first - the confirmed rate, so the expected case costs one pass.
static const uint32_t kBaudSweep[] = {38400, 9600, 4800, 19200, 57600, 115200};

// Long enough to catch several sentences at this sensor's output rate.
#define LISTEN_MS 4000UL

// Counts printable NMEA-looking content, so a run of framing garbage at the wrong
// baud is not mistaken for a working link.
struct ListenResult {
  size_t bytes;
  size_t dollarSigns;
  bool sawMWV;
  char sample[120];
};

static ListenResult listen(uint32_t ms) {
  ListenResult r = {0, 0, false, {0}};
  size_t sampleLen = 0;
  char line[100];
  size_t lineLen = 0;

  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    if (!windSerial.available()) continue;
    char ch = (char)windSerial.read();
    r.bytes++;
    if (ch == '$') r.dollarSigns++;

    if (sampleLen < sizeof(r.sample) - 1 && ch >= 32 && ch < 127) {
      r.sample[sampleLen++] = ch;
      r.sample[sampleLen] = 0;
    }

    if (ch == '\n' || ch == '\r') {
      line[lineLen] = 0;
      if (strstr(line, "MWV") != nullptr) r.sawMWV = true;
      lineLen = 0;
    } else if (lineLen < sizeof(line) - 1) {
      line[lineLen++] = ch;
    }
  }
  return r;
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  delay(500);

  Serial.println();
  Serial.println("=== Calypso ULP PRO probe: first test on the fabbed PCB ===");

  // Rails first. GPIO11 (5V) is ACTIVE-HIGH - this sensor needs 5V, and this is
  // exactly the gate that was driven the wrong way for the life of this board.
  pinMode(PIN_PCB_EN_A, OUTPUT);
  digitalWrite(PIN_PCB_EN_A, PIN_PCB_EN_A_ACTIVE);
  pinMode(PIN_PCB_EN_B, OUTPUT);
  digitalWrite(PIN_PCB_EN_B, PIN_PCB_EN_B_ACTIVE);
  Serial.printf("Rails: GPIO%d=%s (3V3), GPIO%d=%s (5V)\n", PIN_PCB_EN_A,
                PIN_PCB_EN_A_ACTIVE == LOW ? "LOW" : "HIGH", PIN_PCB_EN_B,
                PIN_PCB_EN_B_ACTIVE == LOW ? "LOW" : "HIGH");
  delay(PIN_PCB_EN_SETTLE_MS);
  delay(1500);  // let the sensor boot and start streaming

  // ---- Phase A: prove our own pad before judging the sensor ----
  // RX and TX muxed onto one pad: the ESP reads back what it drove. A verdict from
  // an unverified rig is worthless, and this has caught a wiring fault before.
  Serial.println("\n--- phase A: pad self-check ---");
  {
    const int pads[2] = {PIN_CALYPSO_RX, PIN_CALYPSO_TX};
    for (int i = 0; i < 2; i++) {
      windSerial.begin(38400, SERIAL_8N1, pads[i], pads[i]);
      delay(60);
      while (windSerial.available()) windSerial.read();
      windSerial.print("TEST\r\n");
      windSerial.flush();
      size_t n = 0;
      uint32_t t = millis();
      while (millis() - t < 300 && n < 32) {
        if (windSerial.available()) { windSerial.read(); n++; }
      }
      Serial.printf("  GPIO%d self-echo: %u bytes %s\n", pads[i], (unsigned)n,
                    n >= 6 ? "-> pad OK" : "-> NO ECHO (our UART/pad is at fault)");
      windSerial.end();
    }
  }

  // ---- Phase B: passive listen, both pins x baud ----
  // The sensor streams unprompted, so this alone should find it. Nothing is
  // transmitted at any point.
  Serial.println("\n--- phase B: passive listen (nothing sent), RX pin x baud ---");
  const int rxCandidates[2] = {PIN_CALYPSO_RX, PIN_CALYPSO_TX};
  int liveRx = -1;
  uint32_t liveBaud = 0;

  for (int p = 0; p < 2 && liveRx < 0; p++) {
    Serial.printf("  RX = GPIO%d\n", rxCandidates[p]);
    for (size_t i = 0; i < sizeof(kBaudSweep) / sizeof(kBaudSweep[0]); i++) {
      windSerial.end();
      windSerial.begin(kBaudSweep[i], SERIAL_8N1, rxCandidates[p], PIN_CALYPSO_TX);
      delay(80);
      while (windSerial.available()) windSerial.read();

      ListenResult r = listen(LISTEN_MS);
      Serial.printf("    %6lu baud: %4u bytes, %u '$'%s", (unsigned long)kBaudSweep[i],
                    (unsigned)r.bytes, (unsigned)r.dollarSigns, r.sawMWV ? ", MWV seen" : "");
      if (r.bytes > 0) Serial.printf("  |  %s", r.sample);
      Serial.println();

      // Require NMEA structure, not just bytes - the wrong baud produces plenty of
      // garbage that would otherwise read as success.
      if (r.sawMWV || r.dollarSigns >= 2) {
        liveRx = rxCandidates[p];
        liveBaud = kBaudSweep[i];
        break;
      }
    }
  }

  Serial.println("\n=== VERDICT ===");
  if (liveRx >= 0) {
    Serial.printf("STREAMING - valid NMEA on RX=GPIO%d @ %lu baud.\n", liveRx,
                  (unsigned long)liveBaud);
    if (liveRx != PIN_CALYPSO_RX || liveBaud != CALYPSO_BAUD) {
      Serial.printf("*** pins.h says RX=GPIO%d @ %d - UPDATE IT ***\n", PIN_CALYPSO_RX,
                    CALYPSO_BAUD);
    }
    windSerial.end();
    windSerial.begin(liveBaud, SERIAL_8N1, liveRx, PIN_CALYPSO_TX);
  } else {
    Serial.println("SILENT on every pin/baud combination.");
    Serial.println("Check, in order: 5V present at the sensor's own VCC pin (GPIO11 must be");
    Serial.println("HIGH for that rail); shared ground; the green wire actually on GPIO8.");
    Serial.println("A missing shared ground has faked a dead peripheral twice on this board.");
    windSerial.end();
    windSerial.begin(CALYPSO_BAUD, SERIAL_8N1, PIN_CALYPSO_RX, PIN_CALYPSO_TX);
  }

  Serial.println("\n--- raw stream below ---");
}

void loop() {
  while (windSerial.available()) Serial.write(windSerial.read());
}
