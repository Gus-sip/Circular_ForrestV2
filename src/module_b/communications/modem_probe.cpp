/*
 * BC660K-GL - the RST line, the last untried lever.
 *
 * Exhausted so far, with the module powered and held:
 *   RX on GPIO43/44 swapped                 0 bytes
 *   RX swept across all 18 free GPIOs       0 bytes (only GPIO44's line-settling 0x00)
 *   all seven baud rates                    0 bytes
 *   channel gate open AND closed            0 bytes
 *   EN and channel, every combination, 30s  0 bytes
 *   PWRKEY widths                           moot - NOT WIRED on this board
 *
 * So the pin map is right and the module is simply not talking. One control line
 * has still never been driven: NBIOT_RST_PIN (GPIO6), which Config.h describes as
 * "kept for a future explicit-reset recovery path, not currently wired into
 * POWERING".
 *
 * Why it is worth trying even though PWRKEY turned out to be unconnected: RST and
 * PWRKEY are different things. PWRKEY is a power-on REQUEST the module's own logic
 * can decline if its state is wedged; RST forces the core to restart regardless.
 * "Worked yesterday, silent today, unchanged firmware" is exactly what a latched
 * module looks like, and this is the only remaining way to unlatch it in software.
 *
 * If any strategy below yields bytes, that sequence goes straight into
 * ModemNBIoTMqtt::enterPowering() as the standard bring-up.
 */

#include <Arduino.h>

#include "Config.h"

#define LISTEN_MS 8000UL

static uint8_t buf[512];

static void uartUp() {
  Serial2.end();
  delay(20);
  Serial2.begin(NBIOT_BAUD, SERIAL_8N1, NBIOT_RX_PIN, NBIOT_TX_PIN);
  while (Serial2.available()) Serial2.read();
}

static bool listen(const char *what) {
  size_t n = 0;
  uint32_t t0 = millis();
  uint32_t lastAt = 0;
  while (millis() - t0 < LISTEN_MS) {
    if (millis() - lastAt >= 1200) {
      lastAt = millis();
      Serial2.print("AT\r\n");
    }
    while (Serial2.available() && n < sizeof(buf)) buf[n++] = (uint8_t)Serial2.read();
    delay(1);  // yield - task WDT is 5s with panic enabled
  }
  // A lone 0x00 is the line settling when the UART attaches, not data - it has
  // appeared in every previous run and must not be mistaken for success.
  bool real = (n > 1) || (n == 1 && buf[0] != 0x00);
  Serial.printf("  %-42s -> %u byte(s)", what, (unsigned)n);
  if (n) {
    Serial.print("  hex:");
    for (size_t i = 0; i < n && i < 32; i++) Serial.printf(" %02X", buf[i]);
    Serial.print("  ascii: \"");
    for (size_t i = 0; i < n && i < 32; i++) {
      char c = (char)buf[i];
      Serial.print((c >= 32 && c < 127) ? c : '.');
    }
    Serial.print("\"");
  }
  Serial.println(real ? "   *** REAL DATA ***" : "");
  Serial.flush();
  return real;
}

static void powerOn() {
  digitalWrite(NBIOT_CHANNEL_PIN, NBIOT_CHANNEL_ACTIVE);
  digitalWrite(NBIOT_EN_PIN, NBIOT_EN_ACTIVE);
}

static void powerOff(uint32_t ms) {
  digitalWrite(NBIOT_EN_PIN, NBIOT_DISABLE);
  digitalWrite(NBIOT_CHANNEL_PIN, !NBIOT_CHANNEL_ACTIVE);
  delay(ms);
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  delay(500);

  pinMode(NBIOT_EN_PIN, OUTPUT);
  pinMode(NBIOT_CHANNEL_PIN, OUTPUT);
  pinMode(NBIOT_RST_PIN, OUTPUT);
  pinMode(NBIOT_PWRKEY_PIN, INPUT);  // not wired - deliberately not driven
  digitalWrite(NBIOT_RST_PIN, !NBIOT_RST_ACTIVE);

  Serial.println();
  Serial.println("=== BC660K-GL: RST line trial ===");
  Serial.printf("RST=GPIO%d (active %s)  EN=GPIO%d  CH=GPIO%d  RX=GPIO%d TX=GPIO%d\n",
                NBIOT_RST_PIN, NBIOT_RST_ACTIVE == LOW ? "LOW" : "HIGH", NBIOT_EN_PIN,
                NBIOT_CHANNEL_PIN, NBIOT_RX_PIN, NBIOT_TX_PIN);
  Serial.println("PWRKEY is NOT wired on this board and is not driven.");
  Serial.flush();
}

void loop() {
  Serial.println("\n--- powering module, then trying RST every way ---");
  Serial.flush();
  powerOff(4000);
  powerOn();
  delay(2500);
  uartUp();

  if (listen("A: powered, no RST (baseline)")) return;

  digitalWrite(NBIOT_RST_PIN, NBIOT_RST_ACTIVE);
  delay(NBIOT_RST_PULSE_MS);
  digitalWrite(NBIOT_RST_PIN, !NBIOT_RST_ACTIVE);
  delay(1500);  // let the core come out of reset before expecting anything
  uartUp();
  if (listen("B: RST pulsed 200ms")) return;

  digitalWrite(NBIOT_RST_PIN, NBIOT_RST_ACTIVE);
  delay(1500);
  digitalWrite(NBIOT_RST_PIN, !NBIOT_RST_ACTIVE);
  delay(2500);
  uartUp();
  if (listen("C: RST held 1500ms")) return;

  // Reset asserted BEFORE the rail rises and released after, so the core starts
  // from a defined state rather than whatever it powered into.
  powerOff(4000);
  digitalWrite(NBIOT_RST_PIN, NBIOT_RST_ACTIVE);
  powerOn();
  delay(2500);
  digitalWrite(NBIOT_RST_PIN, !NBIOT_RST_ACTIVE);
  delay(2000);
  uartUp();
  if (listen("D: RST held across VIN rise, released after")) return;

  // Opposite polarity, in case the net is active-high on this board. Cheap to
  // test and the alternative is assuming it away.
  digitalWrite(NBIOT_RST_PIN, !NBIOT_RST_ACTIVE);
  delay(500);
  digitalWrite(NBIOT_RST_PIN, NBIOT_RST_ACTIVE);
  delay(1500);
  digitalWrite(NBIOT_RST_PIN, !NBIOT_RST_ACTIVE);
  delay(2000);
  uartUp();
  if (listen("E: RST inverted polarity")) return;

  Serial.println("\n=== RST produced nothing either - repeating in 10s ===");
  Serial.flush();
  delay(10000);
}
