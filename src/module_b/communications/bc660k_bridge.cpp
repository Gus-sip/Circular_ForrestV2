/*
 * Quectel BC660K-GL (NB-IoT Cat NB2) - AT bring-up, stage 1: liveness check +
 * transparent bridge. Same purpose as rylr998_bridge.cpp / radio_bridge_test.cpp
 * on the other boards in this system: confirm the module answers AT at all,
 * then act as a plain USB<->UART passthrough so SIM/network registration can
 * be driven and observed by hand from the serial monitor. No AT parsing, no
 * driver abstraction yet - this comes after responses have been seen firsthand.
 *
 * Wiring - DELIBERATELY DIFFERENT from the "PCB_Concentrador" schematic's
 * GPIO43/44 ("native RX/TX") routing. Real hardware testing showed the ESP32-S3's
 * masked boot ROM - not configurable, baked into silicon, runs before any of our
 * code including the bootloader - always prints its reset banner
 * ("ESP-ROM:esp32s3-...") on the physical GPIO43/GPIO44 pads on every reset. With
 * the module's RXD wired there, it received that banner as garbage AT commands
 * and replied ERROR to it on every boot. There is no firmware fix for this - it's
 * silicon behavior - so the only real fix is not wiring anything modem-critical to
 * those two physical pins. Moved to GP5/GP10 instead, which the boot ROM never
 * touches:
 *   Module RXD (pin 1) <- ESP TX  -> GP10 (was GPIO43)
 *   Module TXD (pin 2) -> ESP RX  -> GP5  (was GPIO44)
 *   Module VIN/3V3 (pins 6, 9)    -> switched 3V3 rail, gated by Q3 (P-MOSFET)
 *   Module PWR (pin 8)            -> UNCONFIRMED, see below
 *   Module GND (pin 7)            -> common GND
 *
 * If this ever becomes a real fabricated PCB, the schematic's UARTDB_TX/RX nets
 * feeding U4 need to be rerouted to GP5/GP10 (or another pair that isn't
 * GPIO43/44) instead of the board's native RX/TX pins.
 *
 * POWER GATING - not always-on like the bench RYLR998 setup. Q3 is an IRLML6401
 * P-channel MOSFET, gate pulled up to 3V3 through R2 (100k), same topology as the
 * LoRa module's own power switch (Q4/LORA_VCC_EN). For a high-side P-FET switch
 * wired this way, pulling the gate LOW turns the switch ON - so NBIOT_EN_PIN below
 * is driven LOW to power the module up. Inferred from the schematic topology and
 * confirmed on real hardware (module powers and produces boot URCs).
 *
 * NOT YET CONFIRMED - flag these, don't assume:
 *   - PWR (pin 8, PWRKEY): no net label reaches it in the schematic text - it
 *     may be left NC (module auto-boots once VIN is present, consistent with
 *     what's been observed so far) or tied to GND off-sheet.
 *   - Baud rate: defaulted to 9600 8N1 below (Quectel BC66/BC68/BC65-family
 *     default), not verified against this specific module. Try 115200 next
 *     if 9600 gets nothing.
 *   - Current draw: NB-IoT TX bursts can pull several hundred mA momentarily -
 *     if the module resets mid-command, suspect bench supply/decoupling, not
 *     firmware.
 *
 * This is a real hardware UART (UART1 peripheral), not SoftwareSerial.
 */

#include <Arduino.h>

#define BC660K_RX_PIN 5    // ESP RX <- module TXD
#define BC660K_TX_PIN 10   // ESP TX -> module RXD
#define BC660K_BAUD 9600   // UNCONFIRMED - see header comment
#define NBIOT_EN_PIN 9     // 3V3_NBIoT_EN - LOW enables power via Q3 (P-FET), see header comment

// Observed on real hardware: the module doesn't respond to AT the instant
// power is applied - it prints its own boot URCs first (seen so far: "RDY"
// on a cold boot, "+QNBIOTEVENT: \"EXIT DEEPSLEEP\"" when it was already
// woken from deep sleep by a prior power cycle), and AT gets a plain ERROR
// if sent before that finishes. Rather than guess a fixed delay, wait for
// either URC (or a timeout) before treating the module as ready, then retry
// AT a few times - this makes the sketch self-synchronizing regardless of
// which boot path the module takes or when a terminal happens to connect.
#define BOOT_WAIT_MS 6000
#define AT_RETRY_COUNT 6
#define AT_RETRY_GAP_MS 1000

HardwareSerial ModemSerial(1);  // UART1 - GP5/GP10, well away from the boot-ROM pins

// Reads ModemSerial into buf for windowMs, echoing everything to the USB
// console as it arrives. Returns true if needle1 (required) or needle2
// (optional - pass nullptr to skip) showed up anywhere in what was read.
static bool waitFor(const char *needle1, uint32_t windowMs, const char *needle2 = nullptr) {
  char buf[256];
  size_t len = 0;
  uint32_t start = millis();
  while (millis() - start < windowMs) {
    while (ModemSerial.available()) {
      char c = ModemSerial.read();
      Serial.write(c);
      if (len < sizeof(buf) - 1) buf[len++] = c;
    }
    buf[len] = '\0';
    if (strstr(buf, needle1)) return true;
    if (needle2 && strstr(buf, needle2)) return true;
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  uint32_t waitStart = millis();
  while (!Serial && millis() - waitStart < 3000) delay(10);
  delay(500);

  Serial.println();
  Serial.println("=== BC660K-GL AT bring-up: liveness check + transparent bridge ===");
  Serial.println("(now on GP5/GP10, not GPIO43/44 - see header comment on why)");

  pinMode(NBIOT_EN_PIN, OUTPUT);
  digitalWrite(NBIOT_EN_PIN, LOW);  // enable power switch - see header comment on polarity
  Serial.println("Driving NBIOT_EN_PIN (GP9) LOW to enable module power via Q3...");

  ModemSerial.begin(BC660K_BAUD, SERIAL_8N1, BC660K_RX_PIN, BC660K_TX_PIN);

  Serial.printf("Waiting up to %dms for a boot URC (RDY / EXIT DEEPSLEEP / etc)...\n", BOOT_WAIT_MS);
  bool sawBootUrc = waitFor("RDY", BOOT_WAIT_MS, "QNBIOTEVENT");
  // waitFor already echoed everything it saw either way, so a miss here just
  // means neither URC arrived within the window, not that nothing happened.
  Serial.println();
  Serial.println(sawBootUrc ? "Saw a boot URC." : "No boot URC seen within the window - trying AT anyway.");

  bool gotOk = false;
  for (int attempt = 1; attempt <= AT_RETRY_COUNT && !gotOk; attempt++) {
    Serial.printf("Sending AT (attempt %d/%d)...\n", attempt, AT_RETRY_COUNT);
    ModemSerial.print("AT\r\n");
    gotOk = waitFor("OK", AT_RETRY_GAP_MS);
    Serial.println();
  }

  if (!gotOk) {
    Serial.println("Never got OK - before suspecting firmware, check:");
    Serial.println("  1) VIN actually reads ~3V3 with a multimeter (confirms Q3/NBIOT_EN_PIN polarity)");
    Serial.println("  2) PWR (pin 8) state - may need a pulse this sketch doesn't send yet");
    Serial.println("  3) TXD/RXD not swapped, common GND present, wired to GP5/GP10 (not 43/44)");
    Serial.println("  4) baud rate - try 115200 if 9600 gets nothing");
  } else {
    Serial.println("Module responded OK - ready for AT commands.");
  }

  Serial.println();
  Serial.println("--- Transparent bridge: type AT commands below, Enter to send ---");
  Serial.println("--- Try: AT+CPIN? / AT+CIMI / AT+CSQ / AT+CEREG? / AT+CGATT?    ---");
}

void loop() {
  // Module -> USB
  while (ModemSerial.available()) {
    Serial.write(ModemSerial.read());
  }

  // USB -> module: raw passthrough, byte for byte.
  while (Serial.available()) {
    ModemSerial.write(Serial.read());
  }
}
