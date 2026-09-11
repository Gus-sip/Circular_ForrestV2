/*
 * BC660K-GL isolation probe - why does the modem never answer AT?
 *
 * The main firmware retries forever and never gets a byte back. Raising the
 * WAIT_AT ceiling from 8s to 30s changed nothing, which rules out the power-on
 * race (the modem was NOT simply being cut off mid-boot). ARDUINO_USB_CDC_ON_BOOT
 * is 1, so the console is USB CDC and UART0's GPIO43/44 are genuinely free - no
 * pin conflict either.
 *
 * So this strips away the state machine, the OLED, WiFi and the radio, and tests
 * the few things that remain, one variable at a time:
 *
 *   PHASE 1  documented sequence, RX=44 TX=43     - the known-good configuration
 *   PHASE 2  same, but RX/TX SWAPPED             - a swapped pair is silent, not garbled
 *   PHASE 3  longer PWRKEY pulse (2500ms)        - in case 1000ms no longer triggers it
 *   PHASE 4  no channel gate (GP10 left closed)  - proves GP10 matters, or that it does not
 *
 * Every phase dumps RAW BYTES, not parsed replies. The distinction matters: a
 * parser that finds no "OK" and a UART that receives nothing at all look identical
 * from the main firmware, and they point at completely different faults. Any byte
 * arriving - even garbage - proves the modem is alive and the UART path works,
 * which would move the fault to baud rate or framing.
 */

#include <Arduino.h>

#include "Config.h"

#define LISTEN_MS 25000UL
#define AT_GAP_MS 3000UL

static void dumpRaw(const char *label, size_t n, const uint8_t *buf) {
  Serial.printf("  %s: %u byte(s)", label, (unsigned)n);
  if (n == 0) {
    Serial.println("  <- NOTHING AT ALL");
    return;
  }
  Serial.print("  hex:");
  for (size_t i = 0; i < n && i < 64; i++) Serial.printf(" %02X", buf[i]);
  Serial.print("  ascii: \"");
  for (size_t i = 0; i < n && i < 64; i++) {
    char c = (char)buf[i];
    Serial.print((c >= 32 && c < 127) ? c : '.');
  }
  Serial.println("\"");
}

// Power the module up per the sequence in Config.h, optionally varying it.
static void powerUp(uint32_t pwrkeyMs, bool useChannel) {
  pinMode(NBIOT_EN_PIN, OUTPUT);
  pinMode(NBIOT_CHANNEL_PIN, OUTPUT);
  pinMode(NBIOT_PWRKEY_PIN, OUTPUT);

  // Everything off first, and long enough for the module's rail to truly
  // collapse - a half-powered module comes back in an undefined state.
  digitalWrite(NBIOT_EN_PIN, NBIOT_DISABLE);
  digitalWrite(NBIOT_CHANNEL_PIN, !NBIOT_CHANNEL_ACTIVE);
  digitalWrite(NBIOT_PWRKEY_PIN, !NBIOT_PWRKEY_ACTIVE);
  Serial.println("  power down 3s...");
  Serial.flush();
  delay(3000);

  if (useChannel) {
    digitalWrite(NBIOT_CHANNEL_PIN, NBIOT_CHANNEL_ACTIVE);
    Serial.printf("  GP%d channel -> %s\n", NBIOT_CHANNEL_PIN,
                  NBIOT_CHANNEL_ACTIVE == LOW ? "LOW" : "HIGH");
  } else {
    Serial.printf("  GP%d channel LEFT CLOSED (deliberate)\n", NBIOT_CHANNEL_PIN);
  }
  digitalWrite(NBIOT_EN_PIN, NBIOT_EN_ACTIVE);
  Serial.printf("  GP%d VIN -> %s, settling 2s\n", NBIOT_EN_PIN,
                NBIOT_EN_ACTIVE == LOW ? "LOW" : "HIGH");
  Serial.flush();
  delay(2000);

  digitalWrite(NBIOT_PWRKEY_PIN, NBIOT_PWRKEY_ACTIVE);
  Serial.printf("  GP%d PWRKEY -> active for %lums\n", NBIOT_PWRKEY_PIN,
                (unsigned long)pwrkeyMs);
  Serial.flush();
  delay(pwrkeyMs);
  digitalWrite(NBIOT_PWRKEY_PIN, !NBIOT_PWRKEY_ACTIVE);
  Serial.println("  PWRKEY released");
  Serial.flush();
}

static void phase(const char *name, int rxPin, int txPin, uint32_t pwrkeyMs, bool useChannel) {
  Serial.println();
  Serial.printf("================ %s ================\n", name);
  Serial.printf("  UART RX=GPIO%d TX=GPIO%d @ %d baud\n", rxPin, txPin, NBIOT_BAUD);
  Serial.flush();

  Serial2.end();
  delay(50);
  powerUp(pwrkeyMs, useChannel);

  Serial2.begin(NBIOT_BAUD, SERIAL_8N1, rxPin, txPin);
  while (Serial2.available()) Serial2.read();

  static uint8_t buf[512];
  size_t n = 0;
  uint32_t t0 = millis();
  uint32_t lastAt = 0;
  while (millis() - t0 < LISTEN_MS) {
    if (millis() - lastAt >= AT_GAP_MS) {
      lastAt = millis();
      Serial2.print("AT\r\n");
      Serial.printf("  [%5lums] -> AT\n", (unsigned long)(millis() - t0));
      Serial.flush();
    }
    while (Serial2.available() && n < sizeof(buf)) {
      buf[n++] = (uint8_t)Serial2.read();
    }
  }
  dumpRaw("received", n, buf);
  Serial.flush();
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  delay(500);

  Serial.println();
  Serial.println("=== BC660K-GL isolation probe ===");
  Serial.printf("EN=GPIO%d(active %s)  CHANNEL=GPIO%d(active %s)  PWRKEY=GPIO%d(active %s)\n",
                NBIOT_EN_PIN, NBIOT_EN_ACTIVE == LOW ? "LOW" : "HIGH", NBIOT_CHANNEL_PIN,
                NBIOT_CHANNEL_ACTIVE == LOW ? "LOW" : "HIGH", NBIOT_PWRKEY_PIN,
                NBIOT_PWRKEY_ACTIVE == LOW ? "LOW" : "HIGH");
  Serial.println("Any byte received - even garbage - proves the modem is alive.");
  Serial.flush();
}

void loop() {
  phase("PHASE 1: documented sequence", NBIOT_RX_PIN, NBIOT_TX_PIN, NBIOT_PWRKEY_PULSE_MS, true);
  phase("PHASE 2: RX/TX SWAPPED", NBIOT_TX_PIN, NBIOT_RX_PIN, NBIOT_PWRKEY_PULSE_MS, true);
  phase("PHASE 3: longer PWRKEY 2500ms", NBIOT_RX_PIN, NBIOT_TX_PIN, 2500, true);
  phase("PHASE 4: channel gate left CLOSED", NBIOT_RX_PIN, NBIOT_TX_PIN, NBIOT_PWRKEY_PULSE_MS,
        false);

  Serial.println();
  Serial.println("=== all phases done - repeating in 10s ===");
  Serial.flush();
  delay(10000);
}
