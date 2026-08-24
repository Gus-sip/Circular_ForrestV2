/*
 * RYLR998 stage-2 demo: two-role range test over the driver in radio/RYLR998.*
 *
 * Flash one board with ROLE_TX defined below, the other with ROLE_RX - TX sends
 * an incrementing counter every SEND_INTERVAL_MS, RX prints each packet with
 * sender address, RSSI and SNR. That's the range test: walk the RX board away
 * from the TX board and watch RSSI/SNR degrade before packets start dropping.
 *
 * Wiring is identical to rylr998_bridge.cpp (GPIO4/5 crossover - see that file
 * for the net-name-swap explanation).
 *
 * Region: EU868 (confirmed with the project owner - do not change without
 * re-checking legal ISM limits for wherever this actually deploys). The
 * AT+PARAMETER values below (SF9 / BW7 / CR1 / preamble 12) are NOT the
 * generic "commonly documented" RYLR998 defaults this file originally used
 * (7,7,1,4) - that combination came back +ERR=18 from the actual module.
 * These were read directly off the module via AT+PARAMETER?
 * (rylr998_param_probe.cpp) instead of guessed a second time. Higher SF or
 * lower BW buys range at the cost of airtime (and therefore duty-cycle
 * budget) per packet; retune with care, and re-verify with that same probe.
 * See radio/README.md for more.
 *
 * EU863-870 SRD regulations cap the duty cycle on this band (as low as 1% in
 * the most commonly used 868.0-868.6MHz sub-band) - that's a legal limit, not
 * a firmware knob. At SF9/BW125kHz/CR 4/5/preamble 12 with this demo's small
 * counter payload (~10 bytes), time-on-air works out to roughly 160ms per
 * packet (Semtech's public airtime formula, not measured on this exact
 * module), so a 1% duty cycle allows one packet roughly every 16s at the
 * legal limit. SEND_INTERVAL_MS below (30s) keeps a comfortable margin under
 * that; don't shrink it without recomputing for whatever parameters/payload
 * size this ends up tuned to.
 */

#include <Arduino.h>
#include "radio/RYLR998.h"

// Uncomment exactly one of these before building:
#define ROLE_TX
// #define ROLE_RX

#if defined(ROLE_TX) && defined(ROLE_RX)
#error "Define only one of ROLE_TX or ROLE_RX, not both"
#elif !defined(ROLE_TX) && !defined(ROLE_RX)
#error "Define ROLE_TX or ROLE_RX before building this demo"
#endif

#define LORA_RX_PIN 4  // module TXD -> ESP RX
#define LORA_TX_PIN 5  // module RXD <- ESP TX

#define LORA_NETWORK_ID 5
#define LORA_BAND_HZ 868500000UL  // EU868

#if defined(ROLE_TX)
#define LORA_MY_ADDR 1
#define LORA_PEER_ADDR 2
#else
#define LORA_MY_ADDR 2
#define LORA_PEER_ADDR 1
#endif

#define SEND_INTERVAL_MS 30000  // duty-cycle headroom, see file header - don't shrink casually

// spreadingFactor, bandwidth, codingRate, preamble - see file header / README.
RYLR998Params params = {9, 7, 1, 12};

RYLR998 radio(Serial1, LORA_RX_PIN, LORA_TX_PIN);

void setup() {
  Serial.begin(115200);
  uint32_t waitStart = millis();
  while (!Serial && millis() - waitStart < 3000) delay(10);
  delay(500);

  Serial.println();
#if defined(ROLE_TX)
  Serial.println("=== RYLR998 range demo: TX role ===");
#else
  Serial.println("=== RYLR998 range demo: RX role ===");
#endif

  bool ok = radio.begin(LORA_MY_ADDR, LORA_NETWORK_ID, LORA_BAND_HZ, params);
  Serial.println(ok ? "Radio init OK"
                     : "Radio init FAILED - check wiring/baud with rylr998_bridge.cpp first");
}

#if defined(ROLE_TX)

void loop() {
  static uint32_t lastSend = 0;
  static uint32_t counter = 0;

  if (millis() - lastSend >= SEND_INTERVAL_MS) {
    lastSend = millis();
    char payload[16];
    int len = snprintf(payload, sizeof(payload), "%lu", (unsigned long)counter);
    bool sent = radio.send(LORA_PEER_ADDR, payload, (uint8_t)len);
    Serial.printf("TX #%lu: %s\n", (unsigned long)counter, sent ? "sent" : "send FAILED");
    counter++;
  }

  LoRaMessage msg;
  while (radio.poll(msg)) {
    Serial.printf("  (unexpected RX on TX role) from %u: \"%.*s\"  RSSI=%d  SNR=%d\n", msg.senderAddr,
                  msg.length, msg.payload, msg.rssi, msg.snr);
  }
}

#else

void loop() {
  LoRaMessage msg;
  while (radio.poll(msg)) {
    Serial.printf("RX from %u: \"%.*s\"  RSSI=%d  SNR=%d\n", msg.senderAddr, msg.length, msg.payload,
                  msg.rssi, msg.snr);
  }
}

#endif
