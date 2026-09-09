/*
 * Exhaustive on-board LED hunt.
 *
 * GPIO48 and GPIO38 have been ruled out, as WS2812 and as plain active-high GPIO.
 * So the LED is either on a different pin, or it is wired active-LOW (anode to
 * 3V3, cathode to the GPIO), where HIGH means off and every test so far would
 * have left it dark.
 *
 * This drives every safe GPIO in turn, HIGH then LOW, announcing each on serial.
 * Watch the board; when it lights, the announcement on screen names the pin and
 * the polarity. That is the answer, with no more guessing.
 *
 * Pin list is restricted to GPIOs the Module C PCB does not use, so this is SAFE
 * to run on a populated board as well as a bare module. Excluded and why:
 *
 *   1, 2         I2C bus (BME690, SEN0466, BMV080)
 *   3            CM1106 enable
 *   4, 5         RYLR998 LoRa UART
 *   6, 7         CM1106 UART
 *   8, 9         Calypso UART
 *   10, 11, 13   rail gates - driving these switches 3V3, 5V and the radio supply
 *   12           PIN_SPARE, reserved
 *   0            boot strapping
 *   19, 20       USB D-/D+ - would drop the serial console this reports on
 *   26-32        SPI flash
 *   33-37        octal PSRAM
 *   43, 44       UART0
 *
 * What is left is genuinely free on this design, so nothing here can switch a
 * rail, corrupt the I2C bus or talk over a sensor UART.
 */

#include <Arduino.h>

// Free on the Module C PCB - see the header for what is excluded and why.
static const int kPins[] = {14, 15, 16, 17, 18, 21, 38, 39, 40, 41, 42, 45, 46, 47, 48};

#define HOLD_MS 1300UL

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  delay(600);

  Serial.println();
  Serial.println("=== Exhaustive LED hunt ===");
  Serial.printf("Sweeping %d pins, HIGH then LOW on each, %lums per state.\n",
                (int)(sizeof(kPins) / sizeof(kPins[0])), (unsigned long)HOLD_MS);
  Serial.println("Watch the board. When the LED lights, the line on screen names");
  Serial.println("the pin and whether it lit on HIGH or on LOW.");
  Serial.println();

  // Park everything low first, so only the pin under test is ever driven high and
  // a lit LED cannot be attributed to the wrong pin.
  for (size_t i = 0; i < sizeof(kPins) / sizeof(kPins[0]); i++) {
    pinMode(kPins[i], OUTPUT);
    digitalWrite(kPins[i], LOW);
  }
  delay(500);
}

void loop() {
  const size_t n = sizeof(kPins) / sizeof(kPins[0]);

  for (size_t i = 0; i < n; i++) {
    int pin = kPins[i];

    Serial.printf(">>> GPIO%-2d  HIGH   <<<\n", pin);
    Serial.flush();
    digitalWrite(pin, HIGH);
    delay(HOLD_MS);

    Serial.printf(">>> GPIO%-2d  LOW    <<<\n", pin);
    Serial.flush();
    digitalWrite(pin, LOW);
    delay(HOLD_MS);
  }

  // Second pass: the same pins as addressable LEDs, in case one is a WS2812 that
  // the plain-GPIO pass could not light.
  Serial.println();
  Serial.println("--- now trying each pin as an addressable WS2812 ---");
  for (size_t i = 0; i < n; i++) {
    int pin = kPins[i];
    Serial.printf(">>> GPIO%-2d  WS2812 white <<<\n", pin);
    Serial.flush();
    neopixelWrite(pin, 120, 120, 120);
    delay(HOLD_MS);
    neopixelWrite(pin, 0, 0, 0);
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }

  Serial.println();
  Serial.println("--- sweep complete, repeating ---");
  Serial.println();
}
