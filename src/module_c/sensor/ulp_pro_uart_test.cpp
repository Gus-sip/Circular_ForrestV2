/*
 * Calypso ULP PRO Wind Sensor - UART diagnostic sweep
 * Target: ESP32-S3 Mini (PlatformIO, Arduino framework)
 *
 * 2026-08-28: rewritten as a sweep. The plain listener on GPIO9 @ 38400 saw
 * ZERO bytes with the sensor reportedly powered and wired, so this cycles
 * through every plausible (RX pin, baud) combination and reports the byte
 * count for each, plus a raw hex dump of whatever arrives. It also pokes the
 * TX line with a newline each round in case the unit is in poll mode.
 *
 * Wiring being tested (per the user): GREEN (sensor TX) -> GPIO9,
 * YELLOW (sensor RX) -> GPIO8, BROWN -> 3.3-18V, WHITE -> GND (common).
 */

#include <Arduino.h>

HardwareSerial WindSerial(1);  // UART1

// (rxPin, txPin, baud) combos to try, ~6s each.
struct Combo {
  uint8_t rx;
  uint8_t tx;
  uint32_t baud;
};
static const Combo kCombos[] = {
    {9, 8, 38400}, {8, 9, 38400},  // as-wired, then TX/RX swapped
    {9, 8, 9600},  {8, 9, 9600},
    {9, 8, 4800},  {8, 9, 4800},
    {9, 8, 115200},
    {9, 8, 19200},
};
static const size_t kNumCombos = sizeof(kCombos) / sizeof(kCombos[0]);

static size_t comboIdx = 0;
static uint32_t comboStartMs = 0;
static uint32_t bytesThisCombo = 0;
static uint32_t rawDumpCount = 0;

static void startCombo(size_t i) {
  const Combo &c = kCombos[i];
  WindSerial.end();
  delay(20);
  // Pull-up on RX in case the sensor's TX is open-drain / floating.
  pinMode(c.rx, INPUT_PULLUP);
  WindSerial.begin(c.baud, SERIAL_8N1, c.rx, c.tx);
  comboStartMs = millis();
  bytesThisCombo = 0;
  rawDumpCount = 0;
  Serial.printf("\n=== combo %u/%u : RX=GPIO%u  TX=GPIO%u  baud=%lu ===\n", (unsigned)(i + 1),
                (unsigned)kNumCombos, c.rx, c.tx, (unsigned long)c.baud);
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("=== Calypso ULP PRO UART diagnostic sweep ===");
  Serial.println("Sensor should be powered (BROWN 3.3-18V, WHITE common GND).");
  startCombo(0);
}

void loop() {
  const Combo &c = kCombos[comboIdx];

  while (WindSerial.available()) {
    uint8_t b = (uint8_t)WindSerial.read();
    bytesThisCombo++;
    if (rawDumpCount < 64) {
      Serial.printf("%02X%c", b, (b >= 32 && b < 127) ? (char)b : '.');
      rawDumpCount++;
      if (rawDumpCount % 16 == 0) Serial.println();
    }
  }

  // Poke the TX line once a second - a unit in poll mode may answer a CR/LF
  // or a generic NMEA query.
  static uint32_t lastPoke = 0;
  if (millis() - lastPoke >= 1000) {
    lastPoke = millis();
    WindSerial.print("\r\n");
  }

  if (millis() - comboStartMs >= 6000) {
    Serial.printf("\n[combo %u result] RX=GPIO%u baud=%lu -> %lu bytes\n", (unsigned)(comboIdx + 1), c.rx,
                  (unsigned long)c.baud, (unsigned long)bytesThisCombo);
    comboIdx = (comboIdx + 1) % kNumCombos;
    if (comboIdx == 0) Serial.println("\n---- sweep complete, looping ----");
    startCombo(comboIdx);
  }
}
