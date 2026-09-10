/*
 * Status LED: find the correct colour order, definitively.
 *
 * What is known about node C2's LED so far:
 *   - it is populated, on GPIO21, and it lights
 *   - the colours are wrong, and NOT by a fixed channel permutation
 *
 * That second point is what matters. Driving single channels through
 * neopixelWrite() gave results no channel swap can explain - argument (255,0,0)
 * produced TEAL, and one channel cannot produce a two-channel colour. Earlier
 * single-channel readings also contradicted each other between runs. So the frame
 * itself is being decoded wrongly, not merely reordered, and guessing permutations
 * by eye cannot converge.
 *
 * neopixelWrite() hardcodes one bit timing and one colour order with no way to
 * change either. Adafruit_NeoPixel makes both explicit, so this sweeps every
 * colour order and asks for pure GREEN each time. Exactly one should look green -
 * that identifies the part's real order, and it is then a constant in the firmware
 * rather than a guess.
 *
 * Each order is announced before it shows, held 3s, and blanked after, so there is
 * no ambiguity about which one is on screen.
 */

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#include "pins.h"

#define HOLD_MS 3000
#define BLANK_MS 900

struct OrderCase {
  neoPixelType type;
  const char *name;
};

// All six orderings, at the standard 800kHz. If none of these shows green, the
// part is not an 800kHz WS2812-family device and the 400kHz variants come next.
static const OrderCase kOrders[] = {
    {NEO_GRB + NEO_KHZ800, "NEO_GRB"},  // the usual WS2812B
    {NEO_RGB + NEO_KHZ800, "NEO_RGB"},
    {NEO_BRG + NEO_KHZ800, "NEO_BRG"},
    {NEO_RBG + NEO_KHZ800, "NEO_RBG"},
    {NEO_GBR + NEO_KHZ800, "NEO_GBR"},
    {NEO_BGR + NEO_KHZ800, "NEO_BGR"},
};

static void showGreen(const OrderCase &oc) {
  Adafruit_NeoPixel px(1, PIN_STATUS_LED, oc.type);
  px.begin();
  px.setBrightness(255);
  px.setPixelColor(0, px.Color(0, 255, 0));  // asking for pure green
  px.show();
  Serial.printf("  %-8s -> asking for GREEN ... is it green?\n", oc.name);
  Serial.flush();
  delay(HOLD_MS);
  px.clear();
  px.show();
  delay(BLANK_MS);
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  delay(500);

  Serial.println();
  Serial.println("=== Status LED: colour ORDER sweep (GPIO21) ===");
  Serial.println("Each order asks for pure GREEN. Exactly one should look green.");
  Serial.println("Note which name is on the line when the LED is actually green.");
  Serial.flush();

  // Rails up and left up - the LED lit with rails on, so hold that constant.
  pinMode(PIN_PCB_EN_A, OUTPUT);
  digitalWrite(PIN_PCB_EN_A, PIN_PCB_EN_A_ACTIVE);
  delay(300);
  pinMode(PIN_PCB_EN_B, OUTPUT);
  digitalWrite(PIN_PCB_EN_B, PIN_PCB_EN_B_ACTIVE);
  delay(300);
}

void loop() {
  Serial.println();
  Serial.println("---- sweeping colour orders, 3s each ----");
  for (size_t i = 0; i < sizeof(kOrders) / sizeof(kOrders[0]); i++) {
    showGreen(kOrders[i]);
  }
  Serial.println("---- end of sweep, 3s pause then repeat ----");
  Serial.flush();
  delay(3000);
}
