/*
 * On-board LED finder.
 *
 * The activity LED added to the production firmware never lit. The cause could be
 * either of two things and guessing between them wastes flashes: the RGB_BUILTIN
 * macro may not be defined in this build (in which case the calls compiled away to
 * nothing, silently), or this module's LED may simply be on a different pin than
 * the esp32s3 variant's PIN_NEOPIXEL 48.
 *
 * So this sketch reports what the build actually defines, then drives every
 * plausible pin two ways - as an addressable WS2812 and as a plain GPIO - with a
 * spoken announcement before each. Watch the board, note which announcement is on
 * screen when it lights, and that identifies both the pin and the type.
 *
 * Candidates: 48 (esp32s3 variant default), 47, 38 (DevKitC-1 v1.1), 21, 2 and 13
 * cover the common ESP32-S3 module and devkit layouts. GPIO13 is skipped for the
 * plain-GPIO pass on a real Module C PCB, since there it gates the LoRa rail.
 */

#include <Arduino.h>

static const int kCandidates[] = {48, 47, 38, 21, 2};
#define HOLD_MS 2500UL

static void announce(const char *what, int pin) {
  Serial.printf("@@@ %s on GPIO%d - LOOK NOW @@@\n", what, pin);
  Serial.flush();
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  delay(600);

  Serial.println();
  Serial.println("=== On-board LED finder ===");

  // What does this build actually define? This alone may explain the silence.
  Serial.println();
  Serial.println("--- what the build defines ---");
#ifdef RGB_BUILTIN
  Serial.printf("  RGB_BUILTIN   defined, value %d\n", (int)RGB_BUILTIN);
#else
  Serial.println("  RGB_BUILTIN   NOT DEFINED  <-- this is why the LED never lit");
#endif
#ifdef PIN_NEOPIXEL
  Serial.printf("  PIN_NEOPIXEL  defined, value %d\n", (int)PIN_NEOPIXEL);
#else
  Serial.println("  PIN_NEOPIXEL  not defined");
#endif
#ifdef LED_BUILTIN
  Serial.printf("  LED_BUILTIN   defined, value %d\n", (int)LED_BUILTIN);
#else
  Serial.println("  LED_BUILTIN   not defined");
#endif
  Serial.println();
  Serial.println("Now sweeping pins. Watch the board and note which announcement");
  Serial.println("is showing when it lights up.");
  Serial.println();
}

void loop() {
  // Pass 1: addressable WS2812. Bright here purely so it is unmistakable during
  // the hunt - the production firmware uses a few percent of this.
  for (size_t i = 0; i < sizeof(kCandidates) / sizeof(kCandidates[0]); i++) {
    int pin = kCandidates[i];
    announce("WS2812 RED", pin);
    neopixelWrite(pin, 60, 0, 0);
    delay(HOLD_MS);
    announce("WS2812 GREEN", pin);
    neopixelWrite(pin, 0, 60, 0);
    delay(HOLD_MS);
    neopixelWrite(pin, 0, 0, 0);
    delay(300);
  }

  // Pass 2: plain on/off LED, both polarities, since some boards sink rather than
  // source and a plain LED would stay dark through the whole WS2812 pass.
  for (size_t i = 0; i < sizeof(kCandidates) / sizeof(kCandidates[0]); i++) {
    int pin = kCandidates[i];
    pinMode(pin, OUTPUT);
    announce("plain GPIO HIGH", pin);
    digitalWrite(pin, HIGH);
    delay(HOLD_MS);
    announce("plain GPIO LOW", pin);
    digitalWrite(pin, LOW);
    delay(HOLD_MS);
  }

  Serial.println();
  Serial.println("--- sweep complete, repeating ---");
  Serial.println();
}
