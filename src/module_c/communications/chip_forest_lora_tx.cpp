/*
 * CHIP FOREST bring-up + LoRa TX: all five sensors sampled continuously into a
 * last-known-good cache, and that cache transmitted over the RYLR998 to the
 * ground-station receiver (Module B) once every LORA_TX_PERIOD_MS.
 *
 * Payload schema (must match Module B's TelemetryParser exactly):
 *   temp,hum,pres,gas,pm1,pm25,pm10,co2,co,coTemp,windAngle,windSpeed,windValid
 * See src/module_b/communications/telemetry/TelemetryParser.h for the
 * authoritative field list/order - provisional on both ends; if either side
 * changes this encoding, update the other to match.
 *
 * ---- Cadence (2026-09-01) --------------------------------------------------
 * Sample-then-send is split into two independent rates:
 *   - loop() re-reads EVERY sensor every SAMPLE_GAP_MS (~2s) and updates its
 *     cached value whenever the read succeeds. A sensor that hasn't produced a
 *     valid reading yet just keeps being retried; once it has, the cache holds
 *     the freshest good value.
 *   - The radio transmits that whole cached snapshot to Module B once every
 *     LORA_TX_PERIOD_MS (default 5 min; first packet 30s after boot). Module B
 *     in turn batches to Module A / ThingsBoard on its own ~10-min cadence
 *     (MQTT_BATCH_SECONDS in module_b/communications/Config.h).
 * So by the time a packet goes out, each sensor has had ~150 read attempts -
 * even a flaky sensor almost always has a fresh value in it.
 *
 * ---- LoRa parameters (SF7/BW7/CR1/preamble12) ----------------------------
 * SF7 (not the generic "7,7,1,4" default, which returned +ERR=18 from this
 * module - preamble had to be 12). At SF7/BW125/CR4-5/preamble12 with an
 * ~75-byte payload one packet is ~137ms on air. The EU868 1% duty cycle
 * allows ~36s airtime/hour; at a 5-min send period that's ~12 packets/hour
 * (~1.6s/hour) - three orders of magnitude under the cap, so the duty cycle
 * is never the binding constraint at this cadence. LORA_TX_PERIOD_MIN_MS is
 * kept at 15s purely as a sanity floor. SF7 costs ~5-6dB link budget vs SF9
 * (~half the range, worse through foliage) - no field range test has
 * confirmed the deployment closes at SF7 yet. MUST match Module B's
 * Config.h LORA_PARAM_SF.
 *
 * ---- No sleep yet -------------------------------------------------------
 * The node runs a plain delay(SAMPLE_GAP_MS) between sample passes - NOT
 * light or deep sleep. Light sleep was removed 2026-08-28: on the ESP32-S3 it
 * kills USB-Serial/JTAG (port re-enumerates every cycle, wedges the host) and
 * halts the Calypso UART ISR. delay() keeps USB + all UART ISRs serviced and
 * preserves driver state, so the BMV080 5s / CM1106 3s warmups are paid once
 * at boot. The cost is power - this firmware does NOT meet the supercap
 * budget in notes/power_budget.md. The deployment version (deep sleep,
 * switched rails, per-sensor sample periods, Normal/Prealarma/Alarma states)
 * is a separate build on top of this - see that doc.
 *
 * The post-TX LORA_POST_TX_LISTEN_MS window is still the only moment Module B
 * can reach this node with a CFG downlink - whatever relays a command down
 * must hold it and send it the instant it sees this node's uplink.
 *
 * Downlink command grammar (Module B -> this node), styled to match
 * pp1-lora-receiver's own Module A->B grammar (NbiotProtocol.h) so a future
 * relay is a thin translation rather than two incompatible formats:
 *   CFG,<key>=<value>[,<key>=<value>...]
 * Recognized keys: INTERVAL (the LoRa send period, seconds, clamped to
 * [LORA_TX_PERIOD_MIN_MS/1000, LORA_TX_PERIOD_MAX_MS/1000]), and one per sensor -
 * BME690/SEN0466/BMV080/CM1106/CALYPSO (0/1) - to mute/unmute it without a
 * reflash (e.g. BME690, which is known-not-detected on this board per
 * STATUS.md). Unrecognized keys are ignored, not fatal. Applied config is
 * volatile only - it resets to these compiled-in defaults on every reboot,
 * by design (no NVS persistence for this pass). Only commands from
 * LORA_RX_ADDR (Module B) are processed; this is basic sender-address
 * filtering, not authentication - no auth layer exists anywhere in this
 * system yet. On success this node replies ACK,<key>=<value>,... with the
 * actually-applied (post-clamp) values, so a clamp is visible to the sender
 * instead of silently assumed away. A command that doesn't parse as CFG, or
 * applies nothing recognized, gets no ACK - airtime isn't spent acking noise.
 *
 * Missing/not-yet-ready sensor readings: each field keeps its last
 * successfully read value rather than snapping to zero on a transient miss,
 * so one bad cycle doesn't look identical to the sensor genuinely reading
 * zero. A sensor that has never read successfully sends 0. This means the
 * CSV payload alone can't distinguish "sensor briefly busy" from "sensor
 * stuck since boot" - only whether packets are arriving at all, which the
 * receiver's own staleness timer already covers. Fine for this milestone.
 */

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <string.h>
#include <stdlib.h>
#include "../sensor/pins.h"
#include "../sensor/Config.h"
#include "../sensor/PowerManager.h"
#include "../sensor/sensors/Bme690Sensor.h"
#include "../sensor/sensors/Sen0466Sensor.h"
#include "../sensor/sensors/Cm1106Sensor.h"
#include "../sensor/sensors/CalypsoSensor.h"
#include "../sensor/sensors/Bmv080Sensor.h"
#include "radio/RYLR998.h"
#include <esp_system.h>
#include <esp_sleep.h>
#include <Preferences.h>
#include <driver/gpio.h>

// ---------- RYLR998 wiring - GPIO4/5 are unused by pins.h's sensor map ----------
#define LORA_RX_PIN 4  // module TXD -> ESP RX
#define LORA_TX_PIN 5  // module RXD <- ESP TX

// ---------- LoRa link parameters - MUST match pp1-lora-receiver's Config.h ----------
#define LORA_BAND_HZ 868000000UL  // EU868, Spain deployment (confirmed)
#define LORA_NETWORK_ID 5
// SF7 (was SF9): halves airtime enough to make the 15s TX_INTERVAL_MS below
// legal under the EU868 1% duty cycle - see the file-header airtime math.
// Costs ~5-6dB link budget vs SF9. MUST match Module B's Config.h
// (LORA_PARAM_SF) exactly. Re-verify AT+PARAMETER=7,7,1,12 comes back +OK on
// real hardware - this SF/BW/CR/preamble combo has not been confirmed on
// this module (an earlier 7,7,1,4 attempt returned +ERR=18).
#define LORA_PARAM_SF 7
#define LORA_PARAM_BW 7
#define LORA_PARAM_CR 1
#define LORA_PARAM_PREAMBLE 12
// Transmit power, dBm (AT+CRFOP, 0..22). The module's 22 dBm default draws
// ~120mA in the TX burst, and this PCB's supply cannot source that step - the
// board reset on every single transmit, 3 for 3, immediately after "[tx] sending".
// 14 dBm cuts the burst substantially while staying a normal LoRa link budget.
// This is a mitigation, not a cure: the real fix is bulk capacitance on the rail
// (see the note in rylr998_bridge.cpp). Raise it once the supply can take it.
#define LORA_TX_POWER_DBM 14

// This node's AT+ADDRESS - the number Module B sees as "+RCV addr=<n>", and the
// ONLY thing distinguishing one node from another on the air. Every node shipped
// with 1, so Module B could not tell them apart and two nodes' readings
// interleaved under one identity in the log.
//
// Overridable per build so one firmware serves every node - see the
// env:module-c-node* environments. Address 2 is Module B itself and must never be
// used by a node. Keep these in step with NBIOT_NODE_NAMES in Module B's Config.h,
// and never renumber a node already known to ThingsBoard.
//
//   1 = C1 (NodoC-1)   2 = Module B (receiver)   3 = C2   4 = C3
#ifndef LORA_MY_ADDR
#define LORA_MY_ADDR 1
#endif
#define LORA_RX_ADDR 2  // pp1-lora-receiver's AT+ADDRESS

// LoRa send period - Module C -> Module B. Sampling runs continuously
// (SAMPLE_GAP_MS); this is only how often the cached snapshot is transmitted.
// Runtime-tunable via CFG,INTERVAL=<seconds>. At this cadence the EU868 duty
// cycle is a non-issue (see file header); the MIN floor is just a sanity guard.
#define LORA_TX_PERIOD_MS 300000UL      // 5 min - now the FALLBACK deadline, not the cadence
#define LORA_TX_PERIOD_MIN_MS 15000UL   // sanity floor - CFG,INTERVAL can't go below this
#define LORA_TX_PERIOD_MAX_MS 86400000UL  // 24h ceiling - guards a fat-fingered CFG
#define FIRST_TX_DELAY_MS 30000UL       // retained for reference; the readiness gate supersedes it

// ---------- Readiness-gated transmit ----------
// A packet goes out when every ENABLED and PRESENT sensor has produced at least
// one fresh reading since the previous packet - so Module B receives a genuinely
// coherent snapshot rather than a mix of new values and stale cache. Two guards
// bound that, and both are load-bearing:
//
//   MIN_GAP  - sampling runs every SAMPLE_GAP_MS, so "all fresh" can be satisfied
//              within seconds. Without a floor the radio would transmit almost
//              continuously, blowing both the EU868 duty cycle and the power
//              budget. Nothing is ever sent sooner than this after the last TX.
//   MAX_WAIT - a sensor that never returns (disconnected Calypso, a sensor that
//              fails mid-deployment) must not silence the node forever. On this
//              deadline we transmit whatever we have and name the missing sensors
//              in the log, so a stalled sensor degrades the data instead of
//              stopping it. This is what g_txPeriodMs / CFG,INTERVAL now sets.
#define LORA_TX_MIN_GAP_MS 15000UL      // never transmit more often than this

// While in pre-alarm or alarm the node transmits far more often - a fire is the
// one situation where latency matters more than airtime.
//
// DUTY CYCLE, STATED PLAINLY: at SF7 a ~116-byte packet is roughly 0.2 s on air.
// At 5 s that is about 4%, which is OVER the 1% EU868 limit for this band. It is
// deliberate and it is bounded - it only happens while a node is reporting a fire,
// not in normal operation, and a node in alarm that nobody hears is useless. If
// that trade is not acceptable for a given deployment, raise this; the alarm still
// works, it just reports less often.
#define ALARM_TX_MIN_GAP_MS 5000UL

// ---------- BMV080 duty cycling ----------
// The laser draws ~68mA - the largest load in the system by an order of magnitude,
// and notes/power_budget.md budgets it at 20s per 30 min, not continuously. This
// firmware ran it continuously (roughly 90x its energy allowance), and the board
// reset on every boot where it was measuring. It is now started only around an
// actual sample and stopped the moment a frame lands.
#define BMV080_MEASURE_TIMEOUT_MS 30000UL  // give up waiting for a frame after this
#define BMV080_SETTLE_MS 2000UL            // laser preheat before frames are meaningful

// ---------- Fire detection thresholds ----------
// ---- Fire thresholds ----
//
// The real figures from the project's alarma/prealarma spreadsheet were
// transcribed on 2026-09-28 and live in kAlarmRules[], next to the state machine
// that uses them - they have to be declared after the reading globals they point
// at. See that table for the numbers, the per-magnitude direction, and the reason
// each disabled rule is disabled.

// Consecutive reads with every rule clear before a level auto-clears. More than
// one, so a single dip cannot end an alarm during a real fire.
#define ALARM_CLEAR_CONSECUTIVE 5

// ---- PRE-ALARM CONFIRMATION WINDOW ----
//
// Pre-alarm is a question, not a verdict. The node holds it for this long and then
// decides: anything still over its threshold when the window expires escalates to
// full ALARM, because a condition that lasts ten minutes is not sensor noise.
// Everything back under clears to NORMAL before then, via the usual clean-read run.
//
// An ALARM-level reading still escalates IMMEDIATELY and does not wait for this.
// Waiting ten minutes on a 60 C reading would be absurd: the window exists to
// confirm a doubtful signal, not to delay an obvious one.
#define PREALARM_CONFIRM_MS 600000UL  // 10 minutes

// ---------------------------------------------------------------------------
// MEASUREMENT BUILD ONLY - REMOVE BEFORE DEPLOYMENT
//
// Pins the alarm state so each of the three power profiles can be measured on a
// PPK2 for as long as the meter needs, instead of waiting for real smoke:
//
//   0  not forced - normal firmware, the state machine runs as designed
//   1  pinned PRE-ALARM
//   2  pinned ALARM
//
// When forced, updateAlarmState() returns immediately: the state can neither
// clear nor escalate, so a pinned PRE-ALARM stays pre-alarm instead of being
// promoted by the 10-minute confirmation window.
//
// WHY THIS IS DANGEROUS TO LEAVE IN. A node built with 1 or 2 is DEAF TO ITS OWN
// SENSORS - it reports a fire that is not there and, worse, could never report one
// that was, because the state is frozen. It also never sleeps, so on a battery it
// is flat within hours. Every boot prints a banner saying so, and loop() repeats
// it, precisely so a test build cannot be mistaken for a real one.
#ifndef FORCE_ALARM_STATE
#define FORCE_ALARM_STATE 0
#endif

// The status LED is a measurement confound in the raised states.
//
// ledOff() only runs on the way into deep sleep, and the raised states never
// sleep - so the WS2812 stays lit from the last transmit for the whole test and
// its draw lands in the numbers. Set to 1 to keep it dark throughout.
//
// Worth knowing beyond this test: that is REAL behaviour, not a test artefact. A
// node genuinely in alarm leaves its LED lit indefinitely, which is wasted current
// at exactly the moment the battery matters most.
#ifndef FORCE_LED_OFF
#define FORCE_LED_OFF 0
#endif
// ---------------------------------------------------------------------------

// Defined further down, beside kAlarmRules which they depend on. Declared here
// because the CFG parser and the boot-seed block both run long before that.
static void seedThresholds();
static bool setThresholdByName(const char *key, float value);
static float *thresholdSlot(const char *key);

// ---------- Activity LED ----------
// The ESP32-S3 module's own RGB LED (WS2812 on GPIO48, PIN_NEOPIXEL in the
// esp32s3 variant). Lit only while the node is actually doing something: from the
// moment a tick decides it has work, through rail bring-up, sensor reads and the
// transmit, and switched off immediately before deep sleep.
//
// Idle ticks never light it, which makes it a useful health indicator at a glance:
//   brief flash every third tick  -> healthy, cycle running
//   solid on                      -> stuck awake, something is blocking
//   never lights                  -> not completing ticks at all
//
// Deliberately DIM. A WS2812 at full white draws ~60mA - comparable to the BMV080
// laser and more than the ESP32 itself - which would undo the power work this
// firmware exists to do. The values below are a few percent of full scale, still
// clearly visible indoors, and cost on the order of a milliamp.
// Status LED: addressable WS2812 on PIN_STATUS_LED (GPIO21) - see pins.h for why
// it is that pin and not the variant's PIN_NEOPIXEL 48.
//
// Three states, so the node's behaviour is readable from across the room:
//
//   GREEN flash   every wake from deep sleep, including idle ticks. Proof the
//                 timer fired and the node came back - a missing green flash is
//                 the node failing to wake at all.
//   YELLOW        held while actually working: rails up, sensors reading, radio
//                 transmitting. Its duration shows how long a working tick takes.
//   OFF           asleep.
//
// At the default 3 read / 4 transmit counters a healthy node reads as: green wave,
// dark, green wave, dark, green wave then a yellow stretch (the read), then next
// tick a green wave and a shorter yellow stretch (the transmit). Green with no
// yellow ever means ticks are being counted but work is not happening; yellow that
// never goes out means it is stuck awake.
//
// Note the read and transmit schedules are INDEPENDENT counters, each resetting on
// its own threshold - 3 and 4 share no common factor below 12, so which tick of a
// frame carries the transmit drifts, and every 12th tick does both at once.
//
// BRIGHTNESS IS A POWER DECISION HERE, not a cosmetic one.
//
// These were 120 and 120/80, described in a comment as "a few percent of full
// scale". That was simply wrong - 120/255 is nearly half, on two channels at once.
// It went unnoticed for as long as it did because the LED was not lighting AT ALL:
// the colour order was wrong (see ledWrite), so the pixel was being driven with a
// frame it never accepted and drew almost nothing.
//
// The moment the colour order was fixed and the LED actually lit, the board began
// taking POWERON resets - a true supply collapse, not a software fault. A WS2812
// at full white is ~60mA, rivalling the BMV080 laser, and the yellow state is HELD
// for the whole working tick including the radio's ~120mA transmit burst. That is
// precisely the concurrent-load pattern that this firmware's sequential slots exist
// to avoid.
//
// So: low enough to be cheap, high enough to see across a room. A WS2812 is very
// bright - 30/255 is comfortably visible indoors and costs single-digit mA.
#define LED_GREEN_R 0
#define LED_GREEN_G 30
#define LED_GREEN_B 0
#define LED_RED_R 30
#define LED_RED_G 0
#define LED_RED_B 0
#define LED_BLUE_R 0
#define LED_BLUE_G 0
#define LED_BLUE_B 30
// White = transmitting. All three channels, so it draws roughly three times a
// single-channel colour at the same per-channel value - kept at 30 for that
// reason. Safe here because the LED is blanked across the radio burst itself
// (see transmitStore), so it is never lit at the tightest moment for the supply.
#define LED_WHITE_R 30
#define LED_WHITE_G 30
#define LED_WHITE_B 30
#define LED_FLASH_MS 120

// Wake indicator shape: a smooth ramp up and back down rather than a hard blink.
//
// Only the WS2812 path actually fades - the brightness is carried in the RGB
// values sent to it. The plain-level fallback in ledWrite() is a bare digital pin
// with no PWM behind it, so on a plain LED this still reads as a flash, just a
// longer one. GPIO21 on these boards is a WS2812, so the wave is what shows.
//
// COST, because this fires on EVERY wake including idle ticks: an idle tick was
// otherwise almost free - a few hundred milliseconds awake at ~40mA. A 500ms wave
// roughly doubles that, which on solar is a real number, not a rounding error.
// Worth it on the bench where the point is to SEE the node working; turn
// LED_WAVE_MS down (or back to ledFlashWake) for deployment.
#define LED_WAVE_MS 500
#define LED_WAVE_STEPS 24

// This LED is NEO_RGB, NOT the WS2812B-standard NEO_GRB that neopixelWrite()
// hardcodes. Established on node C2 by sweeping all six colour orders and asking
// each for pure green - only NEO_RGB actually showed green.
//
// That mismatch is why every colour was wrong, and why it could not be fixed by
// swapping channels: neopixelWrite() puts the green byte first on the wire, this
// part reads the first byte as red, so the error is in the FRAME rather than in
// the arguments. Single-channel probing gave results no permutation could explain
// (one channel producing teal) and two runs contradicted each other. A driver with
// an explicit colour order settles it; do not "simplify" this back to
// neopixelWrite().
//
// The old implementation also did a plain digitalWrite() after the pixel write, to
// cover the LED being a plain one rather than addressable. That is gone: the type
// is now known, and holding the data line HIGH between frames corrupts the timing
// of the next frame.
static Adafruit_NeoPixel g_statusLed(1, PIN_STATUS_LED, NEO_RGB + NEO_KHZ800);
static bool g_ledBegun = false;

static void ledWrite(uint8_t r, uint8_t g, uint8_t b, bool level) {
  (void)level;  // legacy plain-LED argument, no longer meaningful
#if FORCE_LED_OFF
  // Forced dark for a power measurement - see FORCE_LED_OFF. Done HERE rather
  // than at the call sites so no colour can slip through a path I missed.
  r = g = b = 0;
#endif
  if (!g_ledBegun) {
    g_statusLed.begin();
    g_ledBegun = true;
  }
  g_statusLed.setPixelColor(0, g_statusLed.Color(r, g, b));
  g_statusLed.show();
}

static void ledOff() {
  ledWrite(0, 0, 0, false);
}

// One wave: fade up to full colour and back down. Blocking, like everything else
// on the wake path.
//
// The triangle is computed as a level 0..half..0 and scaled into each channel, so
// a colour with a zero channel (green is 0,120,0) stays that colour throughout
// rather than drifting hue as it fades.
static void ledWave(uint8_t r, uint8_t g, uint8_t b) {
  const int half = LED_WAVE_STEPS / 2;
  const int stepMs = LED_WAVE_MS / LED_WAVE_STEPS;
  for (int i = 0; i <= LED_WAVE_STEPS; i++) {
    int level = (i <= half) ? i : (LED_WAVE_STEPS - i);  // 0 -> half -> 0
    ledWrite((uint8_t)((int)r * level / half), (uint8_t)((int)g * level / half),
             (uint8_t)((int)b * level / half), level > 0);
    delay(stepMs);
  }
  ledOff();
}

// Green wave: "I woke up". Fires on every wake, idle ticks included, so a missing
// wave is the node failing to wake at all.
static void ledFlashWake() {
  ledWave(LED_GREEN_R, LED_GREEN_G, LED_GREEN_B);
}

// White, held for the whole of a transmit, not blinked, so its length is
// meaningful - a transmit is ~1s against the read tick's ~19s of blue.
static void ledWorking() {
  ledWrite(LED_WHITE_R, LED_WHITE_G, LED_WHITE_B, true);
}

// Red: something is wrong that the node cannot fix itself - a sensor that was
// expected and did not answer, or safe mode. Deliberately the SAME brightness as
// the others so it is a colour change rather than an attention-grabbing flash;
// the point is to be readable from a distance, not to be loud.
//
// This matters specifically for untethered testing: with no serial cable, the LED
// and the radio are the only two things that can tell you the node is unhappy.
static void ledTrouble() {
  ledWrite(LED_RED_R, LED_RED_G, LED_RED_B, true);
}

// Blue, held for the whole of a READ tick. Distinguishing reading from
// transmitting matters at a glance: a read tick is ~19s (every sensor plus the
// post-heater rail cycle) while a transmit is barely a second, so the two are
// unmistakable by duration alone once they are different colours.
static void ledReading() {
  ledWrite(LED_BLUE_R, LED_BLUE_G, LED_BLUE_B, true);
}

// ---------- Deep sleep ----------
// The ESP keeps its own 3V3 rail and sleeps itself on a timer (~10-15uA),
// retaining RTC memory; the power MCU is not involved in waking it. Each wake is
// a complete cycle: rails on -> init -> sample until every expected sensor is
// fresh -> transmit -> rails off -> sleep.
//
// SEN0466 is deliberately left OUT of the sleeping cycle for now. It needs a 210s
// settle per notes/power_budget.md, which would dominate every wake; sleeping
// through that window is a later change (it needs a multi-phase wake). Until then
// including it would simply stall each cycle for three and a half minutes.
// Set to 0 for bench testing: the node stays awake, samples continuously and
// transmits on a plain timer. Useful when you need the serial port to stay put and
// the LED to be watchable, rather than the port vanishing every 10 seconds.
// Overridable from platformio.ini (-DSLEEP_ENABLED=0) so a bench or measurement
// build needs no source edit - see the medir-tiempos environment.
#ifndef SLEEP_ENABLED
#define SLEEP_ENABLED 1
#endif

// Transmit cadence when SLEEP_ENABLED is 0. The tick counters cannot drive it in
// this mode - they advance once per boot, and without sleep there are no reboots,
// so they would sit at 1/3 forever and nothing would ever be sent.
#define NOSLEEP_TX_GAP_MS 15000UL
#define SLEEP_CYCLE_SECONDS 10UL        // one tick; work is scheduled in ticks, not seconds
// Were the sensor rails brought up on this wake?
//
// Tracked with a flag rather than read back off the pins: these are outputs, and
// on an idle or hibernation wake nothing has driven them since deep sleep latched
// them off, so a pin read would describe the latch rather than this boot.
static bool g_railsUp = false;
static bool railsArePowered() { return g_railsUp; }


// Below this, the reading is not a flat pack - it is a missing divider.
//
// The board cannot run from a pack this low (the rail collapses around 1.9V), so
// a node that is transmitting this packet while reporting 0.5V is telling us about
// its wiring, not its energy. An older PCB with no divider fitted floats at
// ~100 mV, which is ~0.12V through the ratio and lands well under this.
#define SUPERCAP_V_SANITY_FLOOR 0.8f

// ---- LOW-BATTERY HIBERNATION ----
//
// Below HIB_ENTER_V the node stops being a sensor node and becomes a voltmeter
// that sleeps: no sensor reads, both rails off, the radio off and silent. It wakes
// only to look at the pack, and resumes normal operation once charging has brought
// it back to HIB_EXIT_V.
//
// THE POINT IS THE CELLS, NOT THE UPTIME. A 3.8V supercapacitor pack of this kind
// is damaged by being run down hard, so hibernation is a protective stop: better a
// node that pauses and comes back than one that flattens its pack and needs the
// hardware replaced.
//
// The gap between the two thresholds is hysteresis and is not optional. Without
// it, a pack hovering at the limit would resume, draw current, sag below the limit,
// hibernate, recover, resume... thrashing between the two at exactly the moment it
// has least energy to waste. 0.3V is wide enough that only real charging crosses it.
#define HIB_ENTER_V 2.2f
#define HIB_EXIT_V 2.5f

// How often hibernation wakes to look at the pack.
//
// There is no analog wake-on-threshold on the ESP32-S3 - nothing can interrupt the
// chip when a voltage crosses a level - so "wait until charged" has to be polling.
// A check is a boot, one ADC read and back to sleep with no rail powered: roughly
// 400ms. At 10 minutes that is about 0.8 mAh per day against a ~176 mAh pack.
//
// Longer saves little and costs responsiveness: the node would sit hibernating
// through the first part of a sunny morning with a full pack.
#define HIB_CHECK_SECONDS 600UL

// One STAT as the node enters hibernation, before it goes quiet.
//
// Hibernation itself transmits NOTHING - that is the whole point. But a node that
// simply stops, with no last word, is indistinguishable at Module A from a node
// that has died, and the cost of telling those apart is someone walking into a
// forest. One packet on the way down turns a mystery into a status.
//
// Set to 0 for true silence from the moment the threshold is crossed.
#define HIB_ANNOUNCE_ENTRY 1

// How long to hold before sleeping so the host can drain the USB CDC buffer.
//
// Serial.flush() is NOT sufficient on the ESP32-S3's USB-Serial/JTAG: it waits
// only until the buffer is handed to the USB stack, and if the host is not polling
// at that moment the bytes sit in the endpoint FIFO and die with the peripheral.
// At 50ms the entire tail of every read tick was lost - [read], [store] and
// [sleep] all vanished while the earlier [bmv080] and [cm1106] lines survived,
// which is what a tail-loss looks like as opposed to a crash.
//
// This is BENCH instrumentation. It is pure waste in the field (the awake time it
// adds is spent doing nothing at all), so drop it to ~50ms for deployment - by
// which point there is no USB host listening anyway.
// 400ms was for watching the log over USB. In the field there is no host draining
// the buffer, so it is pure awake time on every single wake - dropped for
// untethered running. Raise it again when debugging on a cable.
#define SLEEP_USB_DRAIN_MS 60

// ---------- Tick scheduling ----------
// Every wake is one tick. Two counters advance on EVERY tick and each resets when
// it reaches its own threshold, so the two schedules are independent:
//
//   sensor_read -> read the FAST sensors (BME690, CM1106, BMV080) and store the
//                  reading. Default 3 ticks = 30s.
//   lora_trans  -> transmit everything stored since the last transmit, then clear
//                  the store. Default 3 ticks = 30s.
//
// Both thresholds are settable from Module A (CFG,SENSOR_READ / CFG,LORA_TRANS).
// A tick where neither is due does no sensor or radio initialisation at all and
// goes straight back to sleep - that is what makes a 10s tick affordable, since
// the BMV080's startup alone would otherwise cost 5s of every 10s tick.
// --- "Dueto Centinela": two sensor groups on two schedules ---
//
// SENTINEL, every tick: BME690 temperature/humidity/pressure with the GAS HEATER
// OFF, plus CO. Both are fast and cheap, and both are the signals that move first
// in a fire. This is the group that justifies a 10s tick.
//
// BURST, every BURST_EVERY_DEFAULT ticks: the expensive sensors - BME690 gas
// (10.8s of heater), BMV080 (68mA laser), CM1106 (power-cycle plus warm-up) and
// the wind sensor. Particulates and CO2 change on the timescale of a spreading
// fire, not of seconds, so 15 minutes matches the phenomenon and costs a
// seventieth of what sampling them every 30s costs.
//
// Both counters live in RTC memory and both are settable from Module A, so the
// balance between detection latency and battery life can be retuned in the field
// without a reflash.
#define BURST_EVERY_DEFAULT 90   // ticks; 90 x 10s = 15 minutes

// NOTE ON THE SENTINEL CADENCE. The sentinels are cheap in SENSOR terms, but on
// this board they are not free: the BME690 sits behind the gated 3V3 rail, so
// reading it means powering that rail and re-initialising the sensor - roughly a
// second, against ~0.15s for a tick that powers nothing.
//
// So a true 10s sentinel cadence (SENSOR_READ=1) costs ~10% duty on rail bring-up
// alone. It becomes genuinely cheap only once the sentinel sensors are permanently
// powered, which is a board change (the 3V3 rail currently gates all of them
// together).
//
// Until then SENSOR_READ stays at 3 - sentinels every 30s - which keeps the idle
// ticks free. Set CFG,SENSOR_READ=1 for 10s sentinels when the power budget allows
// it or the hardware changes.

#define SENSOR_READ_EVERY_DEFAULT 3
#define LORA_TRANS_EVERY_DEFAULT 4
#define COUNTER_EVERY_MIN 1
#define COUNTER_EVERY_MAX 360           // 1 hour at a 10s tick

// The RYLR998 accepts at most 240 bytes per packet. One reading in the compact
// batch format runs ~56 bytes typically and up to ~70 in the worst case (every
// field wide and negative), so three readings plus the "B,n," header is the most
// that reliably fits. Rather than split a batch across packets, LORA_TRANS is
// limited so a batch always fits in one - which is why the ratio, not LORA_TRANS
// alone, is what gets validated:
//
//     readings per packet = ceil(LORA_TRANS / SENSOR_READ)
//
// So with SENSOR_READ=3, LORA_TRANS may go up to 9 (3 readings). Raising
// SENSOR_READ raises the permitted LORA_TRANS in step.
#define READINGS_PER_PACKET_MAX 3
#define BATCH_READING_WORST_BYTES 70

// One more than the packet maximum, purely as headroom so a mid-cycle change to
// the counters cannot overrun the array before the next transmit drains it.
#define READING_STORE_MAX (READINGS_PER_PACKET_MAX + 1)

// Readings a given counter pair would accumulate between transmits.
static uint16_t readingsPerPacket(uint16_t sensorReadEvery, uint16_t loraTransEvery) {
  if (sensorReadEvery == 0) return 0xFFFF;
  return (uint16_t)((loraTransEvery + sensorReadEvery - 1) / sensorReadEvery);
}


// Longest a single scheduled read may spend waiting for every fast sensor to go
// fresh. Bounds the awake time of a reading tick; whatever is still stale is sent
// as its last known value and named in the log.
// Ceiling for one read tick. Must now cover every slot PLUS the post-heater rail
// cycle and the BMV080's startup delay that follows it, so it is far wider than
// when the slots merely ran back to back.
#define SENSOR_READ_WINDOW_MS 45000UL
#define SLEEP_SKIP_SEN0466 1            // 210s settle: out of the cycle until we sleep through it

// A power-cycled board must always give a window to reflash in. Deep sleep drops
// the USB-Serial/JTAG, so without this the port would vanish seconds after boot
// and reappear only briefly each cycle - the same class of problem that got light
// sleep removed from this firmware once before. On a COLD boot (power-on or
// reset, i.e. not a timer wake) the node refuses to sleep for this long.
#define SLEEP_COLD_BOOT_AWAKE_MS 30000UL

// Safety net: never stay awake indefinitely because one sensor never goes fresh
// and the TX deadline is long. Bounds the worst-case energy of a single wake.
#define SLEEP_MAX_AWAKE_MS 120000UL

// Rail gates are held through deep sleep. Left un-held they go Hi-Z, and a
// floating P-FET gate can drift back to conducting - leaving the sensor rails
// powered for the whole sleep and wasting exactly what sleeping was meant to
// save. These are all GPIO<=21, so they are RTC-capable and can be held.
static const gpio_num_t kHeldGates[] = {(gpio_num_t)PIN_PCB_EN_A, (gpio_num_t)PIN_PCB_EN_B,
                                        (gpio_num_t)PIN_LORA_EN};

// ---------- CM1106 freeze recovery ----------
// This unit keeps answering with valid frames whose counter byte increments while
// the CO2 value stays pegged (see Cm1106Sensor::frozenValueRun). It has only ever
// recovered from a full power cycle, so past this many consecutive identical
// values, cycle EN rather than keep publishing a dead number.
#define CM1106_FREEZE_LIMIT 20

#define SAMPLE_GAP_MS 2000UL   // re-read every sensor into its cache this often
#define READ_LOG_GAP_MS 10000UL  // throttle the [read] diagnostic line to at most this rate

#define LORA_POST_TX_LISTEN_MS 2000  // window after each TX to receive a queued CFG command - see file header

// The Calypso streams NMEA continuously. flushInput() drops whatever's stale in
// the RX buffer, then read() is polled for up to this long for a fresh
// checksum-valid $--MWV. With the UART ISR running (no sleep) this usually
// returns almost immediately.
#define CALYPSO_READ_WINDOW_MS 1500

PowerManager power;

Bme690Sensor bme690(BME690_I2C_ADDR);
Sen0466Sensor sen0466(SEN0466_I2C_ADDR);
Bmv080Sensor bmv080(BMV080_I2C_ADDR);
Cm1106Sensor cm1106(Serial2, PIN_CM1106_RX, PIN_CM1106_TX, PIN_CM1106_EN, CM1106_WARMUP_MS, CM1106_BAUD);
CalypsoSensor calypso(Serial1, PIN_CALYPSO_RX, PIN_CALYPSO_TX, CALYPSO_BAUD);
RYLR998 radio(Serial0, LORA_RX_PIN, LORA_TX_PIN);  // UART0 is free - UART1/UART2 taken above

// Survives resets and is cleared only by true power loss. A count > 1 after
// leaving the board completely alone proves it resets on its own, rather than
// being reset by a host touching DTR/RTS over USB - a distinction this project
// could not measure before, and one that decides whether the resets are a power
// fault or a tooling artefact.
// RTC_NOINIT_ATTR, deliberately NOT RTC_DATA_ATTR: the latter lives in
// .rtc.data, which is reloaded from the flash image on every reset, so a counter
// there reads 1 forever and proves nothing. NOINIT survives any reset that keeps
// the RTC domain powered, and is only lost on genuine power loss - which is
// exactly the distinction being measured. It starts as garbage at true power-on,
// hence the magic word.
// BUMPED TO 05 because g_hibernating is a NEW RTC_NOINIT variable. RTC_NOINIT
// memory survives both deep sleep and a reset, and is only re-seeded when this word
// changes - so without the bump a board with existing state would come up with
// g_hibernating holding whatever junk was in that address, and could hibernate
// immediately on a full pack. This trap has cost three debugging sessions already.
#define BOOTCOUNT_MAGIC 0xC0FFEE07UL  // bumped: seeds g_thr[][] from kAlarmRules
RTC_NOINIT_ATTR uint32_t g_bootMagic;
RTC_NOINIT_ATTR uint32_t g_bootCount;

// Survives deep sleep (and any reset that keeps the RTC domain). Lets each wake
// report where it sits in the run, and carries the downlink-tuned send period
// across sleeps - without this, CFG,INTERVAL would silently revert to the
// compiled-in default on every single wake, which would look like the downlink
// being ignored.
RTC_NOINIT_ATTR uint32_t g_wakeCount;

// Set immediately before esp_deep_sleep_start(), cleared on the next boot. This is
// what identifies a wake, NOT esp_sleep_get_wakeup_cause(): a chip reset clears the
// wakeup cause, and on the bench a reset is routine - waking drops USB, the host
// re-enumerates and opening the port resets the ESP32-S3. Relying on the cause made
// every wake look like a cold boot, so the node held awake 30s each time and never
// really slept. Observed on node C2, 2026-09-08. This flag survives that reset
// because RTC_NOINIT does.
RTC_NOINIT_ATTR uint32_t g_sleepFlag;
#define SLEEP_FLAG_MAGIC 0x5EEDBEEFUL

// True while the node is in low-battery hibernation. RTC_NOINIT so it survives the
// deep sleep between checks - the whole state machine is one bool plus the ADC.
RTC_NOINIT_ATTR bool g_hibernating;
RTC_NOINIT_ATTR uint32_t g_txPeriodMsPersist;

// Tick counters and their end-user thresholds. Counters advance every tick and
// reset on reaching their threshold; thresholds arrive by downlink from Module A.
RTC_NOINIT_ATTR uint16_t g_sensorReadCount;
RTC_NOINIT_ATTR uint16_t g_loraTransCount;
// Which sensor the next read tick will service. Survives sleep in RTC memory -
// if it reset every wake the node would read slot 0 forever and never touch the
// others.
RTC_NOINIT_ATTR uint16_t g_slotCursor;
RTC_NOINIT_ATTR uint16_t g_burstCount;
RTC_NOINIT_ATTR uint16_t g_burstEvery;
RTC_NOINIT_ATTR uint16_t g_sensorReadEvery;
RTC_NOINIT_ATTR uint16_t g_loraTransEvery;

// One stored reading. Kept in RTC memory so the batch survives deep sleep between
// the tick that recorded it and the tick that finally transmits it.
struct StoredReading {
  float temp, hum, pres, gas;
  float pm1, pm25, pm10;
  float co2, co, coTemp;
  float windAngle, windSpeed;
  uint8_t windValid;
  uint32_t tickAge;  // ticks before the transmit that this was taken - lets the
                     // receiver reconstruct when, since the node has no clock
};
RTC_NOINIT_ATTR StoredReading g_store[READING_STORE_MAX];
RTC_NOINIT_ATTR uint16_t g_storeCount;
RTC_NOINIT_ATTR uint32_t g_storeDropped;  // readings lost to a full store

// Pre-alarm. Continuous all-sensor operation until the readings settle or Module A
// clears it. Held in RTC so an alarm survives the sleeps it will mostly prevent.
// Three levels now, matching the spreadsheet. ALARM_FIRE is new - the firmware
// previously had no full-alarm state at all, only pre-alarm.
//
// Ordered so "at least pre-alarm" is a >= test: several places must treat both
// raised states the same way (stay awake, do not sleep), and >= cannot silently
// miss the new state the way a chain of == would.
enum AlarmState : uint8_t { ALARM_NORMAL = 0, ALARM_PREALARM = 1, ALARM_FIRE = 2 };
RTC_NOINIT_ATTR uint8_t g_alarmState;
RTC_NOINIT_ATTR uint16_t g_alarmClearRun;   // consecutive clean reads while in pre-alarm

// Milliseconds accumulated in pre-alarm, and why it is not just a millis() stamp.
//
// g_alarmState is RTC_NOINIT and survives a reset, so a node that browns out
// mid-pre-alarm comes back still in pre-alarm - but millis() restarts at zero. A
// stored start-time would therefore hand it a fresh ten minutes on every reset,
// and a node resetting under the 35mA continuous load that pre-alarm imposes could
// postpone a real fire alarm indefinitely without anything looking wrong.
//
// Accumulating instead means the window survives whatever the node does.
RTC_NOINIT_ATTR uint32_t g_prealarmMs;
RTC_NOINIT_ATTR float g_gasBaseline;        // BME690 gas resistance in clean air

// Which sensors read OK on the most recent READ tick, as a 5-bit field
// (bme, bmv, co2, co, wind - MSB first).
//
// Must be RTC_NOINIT and must be SEPARATE from the g_*St status variables. The
// status packet goes out on a TRANSMIT tick, where by definition no sensor was
// read, so the live statuses are still at their power-on defaults and report every
// sensor dead:
//
//   [stat] STAT,23,8,0,00000,17467      <- all five "dead"
//   [read] bme=OK bmv=OK co2=OK ...     <- the very next tick, all fine
//
// On an untethered node that is worse than no status at all: it would cry wolf on
// every single packet and make a genuine failure unnoticeable.
RTC_NOINIT_ATTR uint8_t g_lastReadHealth;
RTC_NOINIT_ATTR uint32_t g_lastReadWake;  // which wake produced it, so age is visible

bool g_wokeFromTimer = false;   // this boot was a deep-sleep wake, not a cold boot
uint32_t g_setupDoneMs = 0;

// BMV080 duty-cycle state - see BMV080_MEASURE_TIMEOUT_MS.
enum class BmvPhase { Idle, Measuring };
BmvPhase g_bmvPhase = BmvPhase::Idle;
uint32_t g_bmvPhaseStartedMs = 0;

bool bmeReady = false;
bool sen0466Ready = false;
bool bmvReady = false;
bool radioReady = false;

// Last-known-good readings. A field stays at its last good value rather than
// snapping to 0 on a transient miss.
//
// THESE MUST BE RTC_NOINIT, NOT PLAIN GLOBALS. Now that each wake services only
// ONE sensor, every other field in a packet comes from a previous wake - and deep
// sleep wipes ordinary RAM. As plain globals the cache reset to zero on every
// wake, so a packet carried one fresh value and twelve zeros:
//
//   bme=NotInit(T0.0 H0.0 gas0) bmv=OK(pm2.5=0.0) ... STALE: BME690,CM1106
//
// which looks exactly like three dead sensors. In RTC memory they survive the
// sleep, so the packet carries a full set whose fields are merely staggered in
// age by up to one rotation.
//
// Seeded to zero in the BOOTCOUNT_MAGIC block below, like every other RTC_NOINIT
// value - they hold garbage on a true power-on until then.
RTC_NOINIT_ATTR float g_temp, g_hum, g_pres, g_gas;
RTC_NOINIT_ATTR float g_pm1, g_pm25, g_pm10;
RTC_NOINIT_ATTR float g_co2;
RTC_NOINIT_ATTR float g_co, g_coTemp;
RTC_NOINIT_ATTR float g_windAngle, g_windSpeed;
RTC_NOINIT_ATTR bool g_windValid;

// Set when a sensor returns a good reading; cleared for all sensors immediately
// after each transmit. These - not a timer - decide when the next packet goes out.
bool g_bmeFresh = false;
bool g_bmvFresh = false;
bool g_co2Fresh = false;
bool g_coFresh = false;
bool g_calFresh = false;

// Status of each sensor's most recent read attempt - for the [read] diag line.
ReadingStatus g_bmeSt = ReadingStatus::NotInitialized;
ReadingStatus g_coSt = ReadingStatus::NotInitialized;
ReadingStatus g_bmvSt = ReadingStatus::NotInitialized;
ReadingStatus g_co2St = ReadingStatus::NotInitialized;
ReadingStatus g_calSt = ReadingStatus::NotInitialized;

// Runtime-mutable config, pushed via downlink CFG - see file header. Volatile
// only: resets to these compiled-in defaults on every reboot, by design.
uint32_t g_txPeriodMs = LORA_TX_PERIOD_MS;  // CFG,INTERVAL=<seconds>
bool g_bmeEnabled = true;
bool g_sen0466Enabled = true;
bool g_bmvEnabled = true;
bool g_cm1106Enabled = true;
bool g_calypsoEnabled = true;

// Parses/applies a "CFG,<key>=<value>[,<key>=<value>...]" downlink command
// into the g_* runtime globals above. Unrecognized keys are logged and
// skipped, not fatal. Returns true (with ackPayload filled, comma-joined
// "<key>=<value>" tokens of what was actually applied post-clamp) only if at
// least one key was recognized - see file header for why a no-op command
// gets no ACK.
static bool applyConfigCommand(const char *payload, uint8_t len, char *ackPayload, size_t ackCap) {
  static const char kPrefix[] = "CFG,";
  static const size_t kPrefixLen = sizeof(kPrefix) - 1;
  if (len <= kPrefixLen || memcmp(payload, kPrefix, kPrefixLen) != 0) return false;

  char buf[241];
  uint8_t bodyLen = len - (uint8_t)kPrefixLen;
  if (bodyLen >= sizeof(buf)) bodyLen = sizeof(buf) - 1;
  memcpy(buf, payload + kPrefixLen, bodyLen);
  buf[bodyLen] = '\0';

  ackPayload[0] = '\0';
  size_t ackLen = 0;
  bool appliedAny = false;

  char *saveptr = nullptr;
  char *tok = strtok_r(buf, ",", &saveptr);
  while (tok != nullptr) {
    char *eq = strchr(tok, '=');
    if (eq) {
      *eq = '\0';
      const char *key = tok;
      const char *valueStr = eq + 1;
      char appliedKv[24] = {0};
      bool matched = true;

      if (strcmp(key, "INTERVAL") == 0) {
        long seconds = atol(valueStr);
        // Also persisted to RTC at sleep, so a downlink-set period survives.
        const long minSeconds = LORA_TX_PERIOD_MIN_MS / 1000;
        const long maxSeconds = LORA_TX_PERIOD_MAX_MS / 1000;
        if (seconds < minSeconds) {
          Serial.printf("[cfg] INTERVAL=%ld below floor, clamped to %ld\n", seconds, minSeconds);
          seconds = minSeconds;
        } else if (seconds > maxSeconds) {
          Serial.printf("[cfg] INTERVAL=%ld above ceiling, clamped to %ld\n", seconds, maxSeconds);
          seconds = maxSeconds;
        }
        g_txPeriodMs = (uint32_t)seconds * 1000UL;
        snprintf(appliedKv, sizeof(appliedKv), "INTERVAL=%ld", seconds);
      } else if (strcmp(key, "SENSOR_READ") == 0) {
        long n = atol(valueStr);
        if (n < COUNTER_EVERY_MIN || n > COUNTER_EVERY_MAX) {
          Serial.printf("[downlink] SENSOR_READ=%ld out of range [%d..%d] - ignored\n", n,
                        COUNTER_EVERY_MIN, COUNTER_EVERY_MAX);
          matched = false;
        } else if (readingsPerPacket((uint16_t)n, g_loraTransEvery) > READINGS_PER_PACKET_MAX) {
          // Lowering SENSOR_READ raises the readings per packet just as surely as
          // raising LORA_TRANS does, so it has to pass the same check.
          Serial.printf("[downlink] SENSOR_READ=%ld would put %u readings in one packet\n"
                        "           (max %d with LORA_TRANS=%u) - ignored\n",
                        n, (unsigned)readingsPerPacket((uint16_t)n, g_loraTransEvery),
                        READINGS_PER_PACKET_MAX, (unsigned)g_loraTransEvery);
          matched = false;
        } else {
          g_sensorReadEvery = (uint16_t)n;
          // Reset the counter too: leaving it above a newly lowered threshold would
          // fire immediately and then look like the setting was ignored.
          g_sensorReadCount = 0;
          snprintf(appliedKv, sizeof(appliedKv), "SENSOR_READ=%ld", n);
        }
      } else if (strcmp(key, "LORA_TRANS") == 0) {
        long n = atol(valueStr);
        if (n < COUNTER_EVERY_MIN || n > COUNTER_EVERY_MAX) {
          Serial.printf("[downlink] LORA_TRANS=%ld out of range [%d..%d] - ignored\n", n,
                        COUNTER_EVERY_MIN, COUNTER_EVERY_MAX);
          matched = false;
        } else if (readingsPerPacket(g_sensorReadEvery, (uint16_t)n) > READINGS_PER_PACKET_MAX) {
          Serial.printf("[downlink] LORA_TRANS=%ld would put %u readings in one packet\n"
                        "           (max %d; with SENSOR_READ=%u the limit is %u) - ignored\n",
                        n, (unsigned)readingsPerPacket(g_sensorReadEvery, (uint16_t)n),
                        READINGS_PER_PACKET_MAX, (unsigned)g_sensorReadEvery,
                        (unsigned)(g_sensorReadEvery * READINGS_PER_PACKET_MAX));
          matched = false;
        } else {
          g_loraTransEvery = (uint16_t)n;
          g_loraTransCount = 0;
          snprintf(appliedKv, sizeof(appliedKv), "LORA_TRANS=%ld", n);
        }
      } else if (strcmp(key, "BURST") == 0) {
        // Ticks between runs of the EXPENSIVE sensor group (PM, CO2, gas, wind).
        // This is the main power/latency dial in the field: 90 ticks = 15 min at
        // the default 10s tick. Bounded by the same COUNTER_EVERY_* limits as the
        // other schedules so a fat-fingered value cannot silence the node.
        long n = strtol(valueStr, nullptr, 10);
        if (n < COUNTER_EVERY_MIN || n > COUNTER_EVERY_MAX) {
          Serial.printf("[downlink] BURST=%ld out of range %d..%d - ignored\n", n,
                        COUNTER_EVERY_MIN, COUNTER_EVERY_MAX);
          matched = false;
        } else {
          g_burstEvery = (uint16_t)n;
          g_burstCount = 0;
          Serial.printf("[downlink] burst group now every %ld ticks (%lds)\n", n,
                        n * (long)SLEEP_CYCLE_SECONDS);
          snprintf(appliedKv, sizeof(appliedKv), "BURST=%ld", n);
        }
      } else if (strcmp(key, "ALARM") == 0) {
        // Force-clear from Module A. Setting it is deliberately NOT supported:
        // an alarm should be raised by the sensors, not asserted remotely.
        if (atoi(valueStr) == 0) {
          g_alarmState = ALARM_NORMAL;
          g_alarmClearRun = 0;
          Serial.println("[downlink] pre-alarm force-cleared by Module A");
          snprintf(appliedKv, sizeof(appliedKv), "ALARM=0");
        } else {
          Serial.println("[downlink] ALARM can only be cleared (ALARM=0), not set - ignored");
          matched = false;
        }
      } else if (strcmp(key, "BME690") == 0) {
        g_bmeEnabled = atoi(valueStr) != 0;
        snprintf(appliedKv, sizeof(appliedKv), "BME690=%d", g_bmeEnabled ? 1 : 0);
      } else if (strcmp(key, "SEN0466") == 0) {
        g_sen0466Enabled = atoi(valueStr) != 0;
        snprintf(appliedKv, sizeof(appliedKv), "SEN0466=%d", g_sen0466Enabled ? 1 : 0);
      } else if (strcmp(key, "BMV080") == 0) {
        g_bmvEnabled = atoi(valueStr) != 0;
        snprintf(appliedKv, sizeof(appliedKv), "BMV080=%d", g_bmvEnabled ? 1 : 0);
      } else if (strcmp(key, "CM1106") == 0) {
        g_cm1106Enabled = atoi(valueStr) != 0;
        snprintf(appliedKv, sizeof(appliedKv), "CM1106=%d", g_cm1106Enabled ? 1 : 0);
      } else if (strcmp(key, "CALYPSO") == 0) {
        g_calypsoEnabled = atoi(valueStr) != 0;
        snprintf(appliedKv, sizeof(appliedKv), "CALYPSO=%d", g_calypsoEnabled ? 1 : 0);

      // ---- Fire thresholds, e.g. CFG,temp_pre_on=50 ----
      //
      // Tried LAST, after every fixed key, so a threshold name can never shadow a
      // config key. The names are the rule names from kAlarmRules suffixed with
      // _pre_on / _pre_off / _alarma_on / _alarma_off, which is exactly what the
      // Module A dashboard sends.
      //
      // The ACK echoes back the value the node STORED, so a dashboard showing a
      // threshold is showing what the node is really using rather than what it was
      // asked to use.
      // THE ACK REPORTS WHAT WAS STORED, NOT WHAT ARRIVED.
      //
      // This used to echo valueStr - the characters Module A sent - which proves
      // only that the key matched a rule. The comment above has always claimed it
      // echoed the stored value; now it does. Module A uses this to display the
      // threshold the node is really using, so an echo of the request would make
      // the dashboard agree with itself by construction and never reveal a
      // disagreement.
      //
      // Formatted with %.2f and then trimmed, rather than %g: this toolchain's
      // nano-newlib snprintf cannot be relied on for the rarer conversions - the
      // same reason appendI64() exists instead of %lld - and a format specifier
      // that silently emits nothing would put a malformed ACK on the air.
      } else if (float *slot = thresholdSlot(key)) {
        *slot = (float)atof(valueStr);
        char num[20];
        snprintf(num, sizeof(num), "%.2f", (double)*slot);
        char *dot = strchr(num, '.');
        if (dot) {
          char *end = num + strlen(num) - 1;
          while (end > dot && *end == '0') *end-- = '\0';
          if (end == dot) *end = '\0';
        }
        snprintf(appliedKv, sizeof(appliedKv), "%s=%s", key, num);

      } else {
        matched = false;
      }

      if (matched) {
        appliedAny = true;
        size_t kvLen = strlen(appliedKv);
        size_t sep = (ackLen > 0) ? 1 : 0;
        if (ackLen + sep + kvLen < ackCap) {
          if (sep) ackPayload[ackLen++] = ',';
          memcpy(ackPayload + ackLen, appliedKv, kvLen);
          ackLen += kvLen;
          ackPayload[ackLen] = '\0';
        }
      } else {
        Serial.printf("[cfg] unrecognized key \"%s\", ignored\n", key);
      }
    }
    tok = strtok_r(nullptr, ",", &saveptr);
  }

  return appliedAny;
}

static const char *statusName(ReadingStatus s) {
  switch (s) {
    case ReadingStatus::NotInitialized: return "NotInit";
    case ReadingStatus::NotReady:       return "NotReady";
    case ReadingStatus::Timeout:        return "Timeout";
    case ReadingStatus::NoAck:          return "NoAck";
    case ReadingStatus::InvalidFrame:   return "BadFrame";
    case ReadingStatus::Ok:             return "OK";
  }
  return "?";
}

// Handles one inbound LoRa message drained during the post-TX listen window -
// either a recognized CFG downlink (apply + ACK) or anything else (logged
// only; no command protocol beyond CFG exists yet).
static void handleInboundMessage(const LoRaMessage &msg) {
  Serial.printf("[downlink] from addr %u (%d bytes, rssi %d, snr %d): %.*s\n", msg.senderAddr, msg.length,
                msg.rssi, msg.snr, msg.length, msg.payload);

  if (msg.senderAddr != LORA_RX_ADDR) {
    Serial.println("[downlink] ignored - not from Module B (LORA_RX_ADDR)");
    return;
  }

  // SILENCE IS THE WRONG ANSWER TO A COMMAND WE UNDERSTOOD BUT COULD NOT APPLY.
  //
  // This used to just return. Module B cannot tell silence-because-rejected from
  // silence-because-out-of-range, so it waited out the full downlink timeout and
  // reported "timeout" - the wrong one of the protocol's four states, a quarter of
  // an hour after the fact. The operator is told the node is unreachable when it
  // is sitting there having refused the command.
  //
  // A CFG we recognised but applied nothing from now gets an explicit NACK, which
  // Module B turns into status "error" immediately.
  const bool looksLikeCfg = (msg.length >= 4 && strncmp(msg.payload, "CFG,", 4) == 0);

  char kv[200];
  if (!applyConfigCommand(msg.payload, msg.length, kv, sizeof(kv))) {
    if (looksLikeCfg && radioReady) {
      const char *nack = "NACK,no recognized keys";
      bool sent = radio.send(LORA_RX_ADDR, nack, (uint8_t)strlen(nack));
      Serial.printf("[cfg] nothing applied - NACK sent (%s)\n", sent ? "ok" : "FAILED");
    } else if (looksLikeCfg) {
      Serial.println("[cfg] nothing applied, and the radio is down - cannot NACK");
    }
    return;
  }

  if (!radioReady) {
    Serial.println("[cfg] applied, but ACK skipped - radio not initialized");
    return;
  }

  char ack[220];
  int ackLen = snprintf(ack, sizeof(ack), "ACK,%s", kv);
  bool sent = radio.send(LORA_RX_ADDR, ack, (uint8_t)ackLen);
  Serial.printf("[cfg] applied, ACK sent: %s (%s)\n", ack, sent ? "ok" : "FAILED");
}

// Bring-up instrumentation. Each begin() below is a candidate for the reset seen
// on 2026-09-04 (board reboots right after "BMV080: OK"), so every step is
// announced BEFORE it runs and flushed - USB CDC buffers, and anything still
// queued is lost when the chip drops, which is exactly the case we're chasing.
static void step(const char *what) {
  Serial.printf("[step] %s ...\n", what);
  Serial.flush();
  delay(20);
}

// ---------------- Boot guard (NVS) ----------------
// The RTC boot counter reports "true power-on" on EVERY reset, which means the RTC
// domain is being lost - the supply is actually collapsing, not a task crashing.
// RTC RAM cannot measure that, because the event being measured destroys it.
//
// NVS lives in flash and survives a power loss, so it can. If the NVS count climbs
// 1, 2, 3, 4 while the RTC counter reads 1 every boot, the supply theory is
// confirmed outright - two counters, same resets, only one of them survives.
//
// It also drives safe mode: after three consecutive boots with no run long enough
// to be called good, the rails are left OFF so the port stays usable for
// reflashing instead of the board looping forever.
static Preferences bootStore;
static uint32_t nvsBoots = 0;

// COUNTS CRASHES, NOT WAKES. The distinction is the whole point.
//
// This used to increment on EVERY boot, with bootGuardMarkGood() clearing it only
// after a run lasted 15s. That is incompatible with deep sleep: an idle tick wakes
// and sleeps again in well under 15s, so the counter was never cleared, climbed on
// every single wake, and tripped safe mode after 12 of them. EVERY healthy
// sleeping node did this, within about two minutes of being switched on.
//
// It was proved by the node's own distress packet: "SAFE,12,8,17" - twelve boots,
// reset reason 8 = DEEPSLEEP, seventeen wakes. Twelve PERFECTLY NORMAL timer wakes
// were being counted as twelve failures.
//
// A deep-sleep wake is the firmware working exactly as designed and must never
// count against it. Only a boot that did NOT come from our own sleep - a crash, a
// brownout, a watchdog, a power cut - is evidence of trouble, and that is what the
// guard now counts.
static void bootGuardBegin(bool wokeFromTimer) {
  bootStore.begin("boot", false);
  nvsBoots = bootStore.getUInt("n", 0);

  if (wokeFromTimer) {
    // Reaching a scheduled wake means the previous cycle completed and slept on
    // purpose. That is a healthy run however short it was, so the counter is
    // cleared here rather than waiting for loop()'s 15s timer - which a short tick
    // never reaches.
    if (nvsBoots != 0) {
      bootStore.putUInt("n", 0);
      Serial.printf("Boot guard: timer wake - counter cleared (was %lu)\n",
                    (unsigned long)nvsBoots);
      nvsBoots = 0;
    }
  } else {
    nvsBoots++;
    bootStore.putUInt("n", nvsBoots);
    Serial.printf("NVS boot count (non-sleep boots since last good run): %lu\n",
                  (unsigned long)nvsBoots);
  }
  Serial.flush();
}

// Called once a run has lasted long enough to count as healthy.
static void bootGuardMarkGood() {
  bootStore.putUInt("n", 0);
  Serial.println("Boot guard: run marked good, NVS counter cleared");
  Serial.flush();
}

// Raw chip-ID read, usable at any point in setup() and independent of the sensor
// driver. Every previous look at this register happened AFTER bme690.begin() had
// already failed, which cannot distinguish "the device never transacts" from
// "something between the bus scan and begin() breaks it". Calling this at two
// different moments does distinguish them.
static void probeBmeChipId(const char *when) {
  const uint8_t addrs[2] = {0x76, 0x77};
  for (int i = 0; i < 2; i++) {
    uint8_t addr = addrs[i];

    // Try the REGISTER READ even when the bare address probe fails.
    //
    // This used to `continue` on a failed zero-length write, which assumes a
    // device that will not ACK an empty transaction cannot answer a real one.
    // That is not true of every I2C part, and it meant a sensor could be declared
    // absent without ever being asked a question. Report both results instead.
    Wire.beginTransmission(addr);
    uint8_t probe = Wire.endTransmission();

    Wire.beginTransmission(addr);
    Wire.write(0xD0);  // BME69X_REG_CHIP_ID
    uint8_t wr = Wire.endTransmission(false);
    if (wr != 0) {
      Serial.printf("[bme probe %s] 0x%02X addr-probe=%u, register write NACKed "
                    "(endTransmission=%u) - no answer either way\n",
                    when, addr, (unsigned)probe, (unsigned)wr);
      continue;
    }
    if (Wire.requestFrom((int)addr, 1) != 1) {
      Serial.printf("[bme probe %s] 0x%02X accepted the register write but returned "
                    "no data\n",
                    when, addr);
      continue;
    }
    uint8_t id = (uint8_t)Wire.read();
    Serial.printf("[bme probe %s] 0x%02X (addr-probe=%u) chip ID = 0x%02X %s\n", when, addr,
                  (unsigned)probe, id,
                  id == 0x61 ? "(0x61 = BME69x, CORRECT)" : "(expected 0x61)");
  }
  Serial.flush();
}

// Bus scan. The BME690 and BMV080 share this bus, so a scan separates "sensor
// gone from the bus" from "sensor present but not answering" - and proves the bus
// itself by whatever else replies on the same wires at the same instant. Node C1
// lost its BME690 mid-session and this was what identified it as physical in one
// line rather than an afternoon.
static void scanI2C(const char *when) {
  Serial.printf("--- I2C scan (%s) ---\n", when);
  int found = 0;
  for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  ACK at 0x%02X\n", addr);
      found++;
    }
  }
  if (found == 0) Serial.println("  (nothing acked)");
  Serial.flush();
}

// Rails brought up one at a time, with USB given time to enumerate first.
//
// The soft-start ramp that used to live here has been removed. It was written for
// the inrush theory, which the bisect disproved - it made the board die EARLIER,
// mid-ramp, and the real cause turned out to be steady-state sensor overlap. Plain
// writes now, identical to the minimal rail sketch that holds these same rails for
// 80+ seconds with zero resets.
static void enableRails(bool safeMode) {
  pinMode(PIN_PCB_EN_A, OUTPUT);
  pinMode(PIN_PCB_EN_B, OUTPUT);
  // Both OFF first - each gate's own inactive level, since the two are opposite.
  digitalWrite(PIN_PCB_EN_A, (PIN_PCB_EN_A_ACTIVE == LOW) ? HIGH : LOW);
  digitalWrite(PIN_PCB_EN_B, (PIN_PCB_EN_B_ACTIVE == LOW) ? HIGH : LOW);

  if (safeMode) {
    Serial.println("SAFE MODE: rails stay off, port is yours");
    Serial.println("  (3 boots without a good run - clear it by flashing again)");
    Serial.flush();
    return;
  }

  delay(2000);  // let USB enumerate before anything loads VBUS
  // A print before AND after each write. "about to" surviving but "asserted" not
  // means it dies in the write itself; both surviving but not "settled" means it
  // dies during the delay that follows. Those are different faults.
  Serial.println("3V3: about to assert");
  Serial.flush();
  g_railsUp = true;  // so hibernation knows whether the sensors can still be told to sleep
  digitalWrite(PIN_PCB_EN_A, PIN_PCB_EN_A_ACTIVE);
  Serial.println("3V3: asserted");
  Serial.flush();
  delay(500);
  Serial.println("3V3: settled");
  Serial.flush();

  // Plain write, exactly as the minimal rail sketch does it - that sketch holds
  // this same rail for 80+ seconds with zero resets, so making this identical
  // removes soft-start as a variable. (Soft-start made it die EARLIER, mid-ramp,
  // which argued against inrush being the cause anyway.)
  Serial.println("5V: about to assert (plain write)");
  Serial.flush();
  digitalWrite(PIN_PCB_EN_B, PIN_PCB_EN_B_ACTIVE);
  Serial.println("5V: asserted");
  Serial.flush();
  delay(500);
  Serial.println("5V: settled");
  Serial.flush();

  Serial.println("LoRa: about to assert");
  Serial.flush();
  pinMode(PIN_LORA_EN, OUTPUT);
  digitalWrite(PIN_LORA_EN, PIN_LORA_EN_ACTIVE);
  Serial.println("LoRa: asserted");

  // BMV080 enable - see PIN_BMV080_EN in pins.h. Without it the sensor opens but
  // will not start its laser.
  pinMode(PIN_BMV080_EN, OUTPUT);
  digitalWrite(PIN_BMV080_EN, PIN_BMV080_EN_ACTIVE);
  Serial.printf("BMV080 EN: GPIO%d -> %s\n", PIN_BMV080_EN,
                PIN_BMV080_EN_ACTIVE == LOW ? "LOW" : "HIGH");

  // Supercap sense is an INPUT. Left as an input explicitly, because this pin was
  // briefly driven as an output during the BMV080 enable hunt.
  pinMode(PIN_SUPERCAP_SENSE, INPUT);
  Serial.flush();
  delay(500);

  Serial.println("rails up");
  Serial.flush();
}

static bool g_safeMode = false;

// How long a safe-mode boot holds before restarting into normal operation. Long
// enough to be caught on serial and to reflash, short enough that a node is not
// out of service for long. Safe mode is a pause, not a terminal state.
#define SAFE_MODE_RECOVER_MS 60000UL

// Brings up ONLY the LoRa rail and radio - no sensors, since sensor load is the
// usual reason for being in safe mode in the first place - and sends one packet
// saying so. Without this a node in safe mode is indistinguishable from a dead
// one, which is exactly how node C1 cost days of investigation.

// One packet on the way into hibernation, then silence.
//
// Modelled on sendSafeModeDistress() and for the same reason: a node that simply
// stops looks exactly like a node that has died. Deliberately NOT the telemetry
// format, so it can never be mistaken for a reading - Module B logs an
// unrecognised payload verbatim, so it surfaces as kind=other rather than being
// dropped.
//
// Brings up ONLY the LoRa rail. Sensors stay off; the pack is already low and the
// whole purpose of this state is to stop drawing from it.
static void sendHibernationNotice(float volts) {
  Serial.println("[HIBERNATE] sending one notice, then going silent");
  Serial.flush();

  pinMode(PIN_LORA_EN, OUTPUT);
  digitalWrite(PIN_LORA_EN, PIN_LORA_EN_ACTIVE);
  delay(PIN_LORA_EN_SETTLE_MS);

  if (!radio.begin(LORA_MY_ADDR, LORA_NETWORK_ID, LORA_BAND_HZ,
                   {LORA_PARAM_SF, LORA_PARAM_BW, LORA_PARAM_CR, LORA_PARAM_PREAMBLE},
                   &Serial)) {
    Serial.println("[HIBERNATE] radio init failed - hibernating unannounced");
    Serial.flush();
    return;
  }

  char msg[64];
  int len = snprintf(msg, sizeof(msg), "HIB,%d,%d,%lu", (int)(volts * 1000.0f),
                     (int)(HIB_EXIT_V * 1000.0f), (unsigned long)g_wakeCount);
  if (len < 0) len = 0;
  if (len >= (int)sizeof(msg)) len = (int)sizeof(msg) - 1;

  bool ok = radio.send(LORA_RX_ADDR, msg, (uint8_t)len);
  Serial.printf("[HIBERNATE] %s (%s)\n", msg, ok ? "sent" : "FAILED");
  Serial.flush();

  digitalWrite(PIN_LORA_EN, (PIN_LORA_EN_ACTIVE == LOW) ? HIGH : LOW);
}

static void sendSafeModeDistress() {
  Serial.println("[safe mode] sending distress packet (radio only, sensors stay off)");
  Serial.flush();

  pinMode(PIN_LORA_EN, OUTPUT);
  digitalWrite(PIN_LORA_EN, PIN_LORA_EN_ACTIVE);
  delay(PIN_LORA_EN_SETTLE_MS);

  if (!radio.begin(LORA_MY_ADDR, LORA_NETWORK_ID, LORA_BAND_HZ,
                   {LORA_PARAM_SF, LORA_PARAM_BW, LORA_PARAM_CR, LORA_PARAM_PREAMBLE},
                   &Serial)) {
    Serial.println("[safe mode] radio init failed - cannot report");
    Serial.flush();
    return;
  }

  // Deliberately NOT the telemetry format: this must not be mistaken for a
  // reading. Module B logs an unrecognised payload verbatim rather than dropping
  // it, so it will surface in porthole_data.csv as kind=other.
  char msg[96];
  int len = snprintf(msg, sizeof(msg), "SAFE,%lu,%d,%lu", (unsigned long)nvsBoots,
                     (int)esp_reset_reason(), (unsigned long)g_wakeCount);
  if (len < 0) len = 0;
  if (len >= (int)sizeof(msg)) len = (int)sizeof(msg) - 1;

  bool ok = radio.send(LORA_RX_ADDR, msg, (uint8_t)len);
  Serial.printf("[safe mode] distress \"%s\" (%s)\n", msg, ok ? "sent" : "FAILED");
  Serial.flush();
}

// Defined below, but setup() must be able to sleep before it ever powers a rail.
static void enterDeepSleep(const char *why, uint32_t seconds = SLEEP_CYCLE_SECONDS);

// Reads the supercapacitor divider. Declared here because the hibernation check
// runs at the very top of setup(), long before the definition further down.
// Defined further down, next to the supercap helpers it depends on.
static void enterHibernation(float volts, bool announce);
static int readSupercapMv();
static float supercapVolts();

// A cold boot always initialises fully and stays awake, whatever the counters say:
// that is the window in which the board can be reflashed, and skipping init there
// would leave a freshly powered node looking dead for its first few ticks.
static bool isColdBoot();

// What this tick owes. Decided BEFORE anything is powered, so an idle tick can
// return to sleep without bringing up a single rail.
static bool g_readDue = false;
static bool g_txDue = false;
// The expensive sensor group is due this tick - see BURST_EVERY_DEFAULT.
static bool g_burstDue = false;

void setup() {
  Serial.begin(115200);

  // WAIT FOR A USB HOST ONLY ON A COLD BOOT - never on a timer wake.
  //
  // This used to wait unconditionally:
  //
  //     while (!Serial && millis() - waitStart < 3000) delay(10);
  //     delay(500);
  //
  // In the field there IS no USB host, so `!Serial` stays true and every wake -
  // including the idle ticks that are supposed to be nearly free - burned the full
  // 3s, plus another 500ms unconditionally. At a 10s tick that is 3.5s of every
  // 10s: a 35% duty cycle at ~40mA, roughly 14mA average, spent waiting for a
  // serial port that will never appear.
  //
  // It also hid itself during development: on the bench, USB IS attached, `Serial`
  // goes true within milliseconds and the wait collapses - so every measurement
  // taken with a cable understated the real field consumption.
  //
  // g_sleepFlag is RTC_NOINIT and survives deep sleep, so it can be read here
  // before anything else has run. A timer wake means nobody is watching: skip
  // straight to work. A cold boot means someone may have just plugged it in and
  // wants to see the banner, so keep the wait there.
  if (g_sleepFlag != SLEEP_FLAG_MAGIC) {
    uint32_t waitStart = millis();
    while (!Serial && millis() - waitStart < 3000) delay(10);
    delay(500);
  }

  Serial.println();
  Serial.println("=== CHIP FOREST + LoRa TX: sensors -> RYLR998 -> ground station ===");
#if FORCE_ALARM_STATE
  g_alarmState = (FORCE_ALARM_STATE >= 2) ? ALARM_FIRE : ALARM_PREALARM;
  g_alarmClearRun = 0;
  g_prealarmMs = 0;
  Serial.println("********************************************************");
  Serial.printf("*** TEST BUILD - ALARM STATE FORCED TO %-14s ***\n",
                (FORCE_ALARM_STATE >= 2) ? "ALARM" : "PRE-ALARM");
  Serial.println("*** The node is DEAF to its own sensors and NEVER     ***");
  Serial.println("*** sleeps. NOT FOR DEPLOYMENT. Rebuild without       ***");
  Serial.println("*** FORCE_ALARM_STATE before this board goes out.     ***");
  Serial.println("********************************************************");
  Serial.flush();
#endif
  Serial.flush();

  esp_reset_reason_t rr = esp_reset_reason();
  const char *rrName = rr == ESP_RST_POWERON    ? "POWERON"
                       : rr == ESP_RST_EXT      ? "EXT"
                       : rr == ESP_RST_SW       ? "SW"
                       : rr == ESP_RST_PANIC    ? "PANIC (crash)"
                       : rr == ESP_RST_INT_WDT  ? "INT_WDT"
                       : rr == ESP_RST_TASK_WDT ? "TASK_WDT"
                       : rr == ESP_RST_WDT      ? "WDT"
                       : rr == ESP_RST_BROWNOUT ? "BROWNOUT (rail collapsed)"
                       : rr == ESP_RST_DEEPSLEEP ? "DEEPSLEEP"
                                                 : "OTHER";
  esp_sleep_wakeup_cause_t wakeCause = esp_sleep_get_wakeup_cause();

  // The flag is authoritative; the cause is only reported for diagnosis.
  g_wokeFromTimer = (g_sleepFlag == SLEEP_FLAG_MAGIC);
  g_sleepFlag = 0;

  // The boot guard needs to know whether this was our own timer wake, so it runs
  // HERE and not earlier - a deep-sleep wake must not be counted as a failure.
  // (It still runs before any rail is touched, which was the original intent.)
  bootGuardBegin(g_wokeFromTimer);

  if (g_bootMagic != BOOTCOUNT_MAGIC) {
    g_bootMagic = BOOTCOUNT_MAGIC;
    g_bootCount = 0;
    g_wakeCount = 0;
    g_sleepFlag = 0;
    g_txPeriodMsPersist = LORA_TX_PERIOD_MS;
    g_sensorReadCount = 0;
    g_loraTransCount = 0;
    g_slotCursor = 0;
    // The reading cache lives in RTC_NOINIT too, so it holds garbage on a true
    // power-on and must be seeded here with everything else.
    g_temp = g_hum = g_pres = g_gas = 0.0f;
    g_pm1 = g_pm25 = g_pm10 = 0.0f;
    g_co2 = 0.0f;
    g_co = g_coTemp = 0.0f;
    g_windAngle = g_windSpeed = 0.0f;
    g_windValid = false;
    // Start DUE, not at zero, so the first working tick runs a burst immediately.
    // Otherwise a freshly powered node spends its first BURST_EVERY_DEFAULT ticks
    // (15 minutes by default) transmitting gas/PM/CO2 as zeros, which is
    // indistinguishable from three dead sensors at the receiving end.
    g_burstEvery = BURST_EVERY_DEFAULT;
    // A freshly seeded node starts awake. If the pack really is low, the check in
    // setup() puts it straight back into hibernation on this same boot.
    g_hibernating = false;
    g_burstCount = g_burstEvery;
    g_sensorReadEvery = SENSOR_READ_EVERY_DEFAULT;
    g_loraTransEvery = LORA_TRANS_EVERY_DEFAULT;
    g_storeCount = 0;
    g_storeDropped = 0;
    g_alarmState = ALARM_NORMAL;
    g_alarmClearRun = 0;
    g_prealarmMs = 0;
    seedThresholds();
    g_gasBaseline = 0.0f;
    g_lastReadHealth = 0;
    g_lastReadWake = 0;
    Serial.println("Boot counter initialised (true power-on, or RTC domain lost)");
  }
  if (g_wokeFromTimer) {
    g_wakeCount++;
    Serial.printf("Deep-sleep wake #%lu (cause=%d)\n", (unsigned long)g_wakeCount,
                  (int)wakeCause);
  } else {
    Serial.printf("COLD boot (cause=%d) - staying awake at least %lums so the port\n"
                  "  stays open long enough to reflash\n",
                  (int)wakeCause, (unsigned long)SLEEP_COLD_BOOT_AWAKE_MS);
  }
  // Carry the downlink-tuned send period across sleeps.
  g_txPeriodMs = g_txPeriodMsPersist;

  // Every boot is one tick, so both counters advance here - this is the only place
  // they do. loop() then compares each against its own threshold to decide what
  // this tick owes. Counting in setup() rather than loop() matters: loop() runs
  // many times per wake, so incrementing there would race through the thresholds
  // in milliseconds instead of once per 10s tick.
  if (g_burstCount < 0xFFFF) g_burstCount++;
  if (g_sensorReadCount < 0xFFFF) g_sensorReadCount++;
  if (g_loraTransCount < 0xFFFF) g_loraTransCount++;
  // Reset reason FIRST - before any bail-out can skip it. This costs one line per
  // wake and is the difference between diagnosing a reset and guessing at it.
  Serial.printf("Last reset reason: %d = %s\n", (int)rr, rrName);
  Serial.printf("Node address: %d (Module B sees this as +RCV addr=%d)\n", LORA_MY_ADDR,
                LORA_MY_ADDR);
  Serial.printf("Tick: sensor_read %u/%u, lora_trans %u/%u\n",
                (unsigned)g_sensorReadCount, (unsigned)g_sensorReadEvery,
                (unsigned)g_loraTransCount, (unsigned)g_loraTransEvery);

  // Green: woke up. Before the idle bail-out below, so every wake flashes -
  // including the two ticks in three that go straight back to sleep.
  ledFlashWake();

  // ---- LOW-BATTERY HIBERNATION: decided before anything is powered ----
  //
  // Measured here, at the top of the wake, for a specific reason: the rail SAGS
  // under load. A reading taken after a LoRa burst or a sensor slot would be a
  // measurement of the dip, not of the pack, and would park a healthy node for
  // hours. At this point in the wake nothing has been switched on since deep sleep
  // latched it all off, so this is the pack at rest - the fairest number available.
  //
  // The divider sits directly across the pack and needs no rail of its own, so
  // this costs one ADC read.
  float packV = supercapVolts();

  if (g_hibernating) {
    // In hibernation the ONLY job is to decide whether charging has brought it
    // back. No sensors, no radio, no rails - just look and sleep again.
    if (packV >= HIB_EXIT_V) {
      Serial.printf("[HIBERNATE] pack recovered to %.2fV (>= %.2fV) - resuming normal operation\n",
                    packV, (double)HIB_EXIT_V);
      Serial.flush();
      g_hibernating = false;
      // Falls through into a normal tick from here.
    } else {
      Serial.printf("[HIBERNATE] pack %.2fV, waiting for %.2fV - back to sleep for %lus\n",
                    packV, (double)HIB_EXIT_V, HIB_CHECK_SECONDS);
      Serial.flush();
      enterDeepSleep("still hibernating", HIB_CHECK_SECONDS);
      return;  // unreachable
    }
  } else if (packV <= HIB_ENTER_V && packV >= SUPERCAP_V_SANITY_FLOOR && !isColdBoot()) {
    // SANITY FLOOR GUARD: an older PCB has no divider and reads ~0.12V, which would
    // otherwise hibernate every such board permanently on its first wake. A reading
    // that low means "no divider fitted", not "flat pack" - see the same guard in
    // transmitStatus().
    //
    // COLD BOOT EXCLUDED so a node with a flat pack can still be reflashed. Deep
    // sleep drops the USB port; hibernating immediately on power-up would leave no
    // window to reprogram it, and a node you cannot reprogram is worse than one
    // that drains a little faster. The 30s cold-boot window runs first, then loop()
    // hibernates at the end of it.
    enterHibernation(packV, true);
    return;  // unreachable
  }

  g_readDue = (g_sensorReadCount >= g_sensorReadEvery);
  g_txDue = (g_loraTransCount >= g_loraTransEvery);
  g_burstDue = (g_burstCount >= g_burstEvery);

  // ---- Idle tick: power nothing, go straight back to sleep ----
  // This decision used to live in loop(), which meant every wake first switched on
  // all three rails, paid the BMV080's 5s preheat and initialised the radio - only
  // to discover it had nothing to do. Two ticks in three were pure waste, and the
  // measured cadence was 76s instead of the configured 30s because of it.
  //
  // It matters for more than energy. Bringing up three rails and a laser is a large
  // simultaneous load step, and doing it three times more often than necessary is
  // three times the opportunity to brown out on a supply with little headroom.
  //
  // The rails are still off here (deep sleep latched them off and nothing has
  // driven them since), so there is nothing to undo - just sleep again.
#if SLEEP_ENABLED
  // EVERY schedule must be listed here. Omitting one does not merely skip a tick -
  // that work never happens at all, because the bail-out sleeps before any rail is
  // powered. g_burstDue was missed when the burst schedule was added, which would
  // have meant the expensive sensor group never ran on a tick that owed nothing
  // else.
  if (!g_readDue && !g_txDue && !g_burstDue && !isColdBoot()) {
    enterDeepSleep("nothing due - no rails powered");
    return;  // unreachable: enterDeepSleep does not return
  }
#else
  // Continuous mode initialises everything every time - there is no idle tick to
  // skip, and bailing out here would leave setup() half-done.
  g_readDue = true;
  g_txDue = true;
#endif

  // Colour says WHICH kind of work this tick is doing, held until sleep:
  //   BLUE   a read tick - rails and sensors
  //   WHITE  a transmit-only tick - radio
  // A tick that does both starts blue and switches to white at the transmit.
  if (g_readDue) {
    ledReading();
  } else {
    ledWorking();
  }
  g_bootCount++;
  Serial.printf("Boot #%lu since last power loss\n", (unsigned long)g_bootCount);
  // (reset reason already printed above, before the idle bail-out)
  Serial.flush();

  // Deep sleep left these pads latched (see kHeldGates). Nothing can drive them
  // until the hold is released, so this must happen before the pinMode calls
  // below - otherwise the writes are silently ignored and the rails stay off.
  // Re-enabled: this was disabled during the reset bisect, which has since found
  // the real cause (concurrent sensor load, see the sequential slots below). It is
  // REQUIRED for sleep to work at all - the gates are latched through deep sleep,
  // so without releasing the hold the pinMode/digitalWrite calls below are
  // silently ignored and the rails never come back on after the first wake.
  for (size_t i = 0; i < sizeof(kHeldGates) / sizeof(kHeldGates[0]); i++) {
    gpio_hold_dis(kHeldGates[i]);
  }
  gpio_deep_sleep_hold_dis();

  // Rails one at a time, with USB given time to enumerate first - see enableRails().
  // Safe mode after three boots without a good run leaves them off entirely, so a
  // board that cannot hold its supply still gives back a usable serial port.
  // Threshold raised from 3: while the failure is being reproduced deliberately,
  // a 3-boot trigger hijacks every test run before the interesting output appears.
  g_safeMode = (nvsBoots >= 12);
  enableRails(g_safeMode);

  // (The bisect stop that used to sit here is gone - it did its job. The answer
  // was that the fault is in what comes AFTER: activating the sensors all at once,
  // not powering the rails. See the sequential slots below.)
  if (g_safeMode) {
    Serial.println("Rails off - skipping sensor and radio init this boot.");
    Serial.flush();
    return;  // nothing below can work without power
  }

  power.enable3V3Sensors();
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  scanI2C("after rails, before sensor init");
  probeBmeChipId("right after scan");

  Serial.printf("Waiting %dms for BMV080 startup...\n", BMV080_STARTUP_DELAY_MS);
  delay(BMV080_STARTUP_DELAY_MS);

  probeBmeChipId("after BMV080 wait, before begin");
  step("bme690.begin");
  bmeReady = bme690.begin();
  // begin() tries the configured address then the alternate, so print which one it
  // settled on - "OK" alone cannot tell 0x76 from 0x77.
  if (bmeReady) {
    Serial.printf("BME690: OK (bound to 0x%02X)\n", bme690.address());
  } else if (bme690.chipIdValid()) {
    // The driver refused it but the register still reads. 0x61 means the part is
    // present and healthy and the DRIVER is at fault; anything else means the bus
    // is corrupting a multi-byte read while still ACKing a bare address probe.
    Serial.printf("BME690: NOT FOUND at 0x%02X - chip ID 0x%02X via mode %d (expect 0x61) -> %s\n",
                  bme690.address(), bme690.lastChipId(), bme690.chipIdMode(),
                  bme690.lastChipId() == 0x61 ? "part is FINE, driver init failed"
                                              : "BUS CORRUPTION or wrong part");
  } else {
    Serial.printf("BME690: NOT FOUND at 0x%02X - chip ID register does not respond\n",
                  bme690.address());
  }

#if SLEEP_ENABLED && SLEEP_SKIP_SEN0466
  // Out of the cycle for now: its 210s settle would dominate every wake. Disabled
  // rather than merely unread, so the readiness gate does not wait on it.
  g_sen0466Enabled = false;
  sen0466Ready = false;
  Serial.println("SEN0466: SKIPPED (210s settle - excluded from the sleeping cycle)");
#else
  step("sen0466.begin");
  sen0466Ready = sen0466.begin();
  Serial.println(sen0466Ready ? "SEN0466: OK" : "SEN0466: NOT FOUND");
#endif

  // ---- INIT ORDER SWAPPED (diagnostic, 2026-09-04) ----
  // Original order was bmv080 then cm1106, and the board reset at cm1106.begin()
  // on every boot where the BMV080 came up - never when it didn't. Swapping tells
  // us which kind of fault that is:
  //   reset moves to bmv080.begin()  -> cumulative load, both together exceed the supply
  //   reset disappears entirely      -> an inrush/settling timing bug in the sequencing
  // Revert this once the answer is in; it is a probe, not a fix.
  step("cm1106.begin (SWAPPED: now before bmv080)");
  cm1106.begin();
  step("cm1106.begin returned");

  step("bmv080.begin (SWAPPED: now after cm1106)");
  bmvReady = bmv080.begin();
  if (bmvReady) {
    Serial.println("BMV080: OK");
  } else {
    // Say WHICH stage failed. "NOT FOUND" alone cannot distinguish a sensor that
    // is absent from one that is present but would not open - and those point at
    // the bus and the supply respectively.
    Serial.printf("BMV080: NOT FOUND - open failed at every strap address "
                  "(0x57/0x56/0x55/0x54), last SDK status %d\n",
                  bmv080.lastOpenStatus());
  }
  // begin() leaves it measuring in order to prove presence; park it straight away
  // so the laser stays off until a sample is actually wanted.
  if (bmvReady) {
    bmv080.stopMeasurement();
    g_bmvPhase = BmvPhase::Idle;
  }

  step("calypso.begin next");
  calypso.begin();
  step("calypso.begin returned - radio next");

  Serial.println("Configuring RYLR998 (link parameters must match pp1-lora-receiver):");
  radioReady = radio.begin(LORA_MY_ADDR, LORA_NETWORK_ID, LORA_BAND_HZ,
                            {LORA_PARAM_SF, LORA_PARAM_BW, LORA_PARAM_CR, LORA_PARAM_PREAMBLE}, &Serial);
  if (radioReady) {
    bool powerSet = radio.setTxPower(LORA_TX_POWER_DBM, &Serial);
    Serial.printf("LoRa TX power -> %d dBm: %s\n", LORA_TX_POWER_DBM,
                  powerSet ? "OK" : "REJECTED (still at module default, likely 22)");
  }
  Serial.println(radioReady ? "Radio init OK"
                             : "Radio init FAILED - not transmitting until this is fixed "
                               "(see the AT exchange above for which step was rejected)");

  Serial.println("Setup complete.\n");
  g_setupDoneMs = millis();
}

// Powers everything down and sleeps. Rails are switched OFF and their gates
// latched, so nothing downstream draws during the sleep - that gating, not the
// MCU's own ~10uA, is where the saving actually comes from.
static void enterDeepSleep(const char *why, uint32_t seconds) {
#if !SLEEP_ENABLED
  (void)why;
  return;
#else
  // Off before sleeping. A WS2812 latches its last value, so without this it would
  // stay lit through the entire sleep - burning current and making the indicator
  // meaningless.
  ledOff();

  Serial.printf("\n[sleep] %s - sleeping %lus\n", why, (unsigned long)seconds);

  // Quiesce each device before its rail disappears, rather than yanking power
  // from underneath a laser or a warming heater.
  if (bmvReady) bmv080.stopMeasurement();
  cm1106.sleep();

  // Persist anything a wake needs, while RAM still exists.
  g_txPeriodMsPersist = g_txPeriodMs;

  // Rails off. The two gates have OPPOSITE polarity, so each needs the inverse of
  // its own active level - a single shared offLevel would switch one rail off and
  // the other ON, powering a rail for the whole sleep.
  digitalWrite(PIN_PCB_EN_A, (PIN_PCB_EN_A_ACTIVE == LOW) ? HIGH : LOW);
  digitalWrite(PIN_PCB_EN_B, (PIN_PCB_EN_B_ACTIVE == LOW) ? HIGH : LOW);
  digitalWrite(PIN_LORA_EN, (PIN_LORA_EN_ACTIVE == LOW) ? HIGH : LOW);

  // Latch those levels for the duration of the sleep - see kHeldGates.
  for (size_t i = 0; i < sizeof(kHeldGates) / sizeof(kHeldGates[0]); i++) {
    gpio_hold_en(kHeldGates[i]);
  }
  gpio_deep_sleep_hold_en();

  Serial.flush();
  delay(SLEEP_USB_DRAIN_MS);

  g_sleepFlag = SLEEP_FLAG_MAGIC;  // so the next boot knows it came from sleep
  esp_sleep_enable_timer_wakeup((uint64_t)seconds * 1000000ULL);
  esp_deep_sleep_start();
  // never returns
#endif
}

// A cold-booted node must stay awake long enough to be reflashed. Deep sleep drops
// the USB-Serial/JTAG, and at a 10s tick the port would otherwise be present for
// only a second or two at a time - practically impossible to catch. So after a
// power-on or reset (as opposed to a timer wake) the node refuses to sleep until
// this window has passed. A timer wake skips it entirely and sleeps as soon as its
// tick work is done.
// A cold boot is any start that did not come from our own deep sleep - power-on,
// or a reset. g_wokeFromTimer is set from the RTC flag, not the wakeup cause,
// because a chip reset clears the cause (see the flag's declaration).
static bool isColdBoot() { return !g_wokeFromTimer; }

static bool coldBootWindowOpen() {
  if (g_wokeFromTimer) return false;
  uint32_t awake = millis() - g_setupDoneMs;
  if (awake >= SLEEP_COLD_BOOT_AWAKE_MS) return false;
  static uint32_t lastNoticeMs = 0;
  if (millis() - lastNoticeMs >= 5000) {
    lastNoticeMs = millis();
    Serial.printf("[boot] holding awake %lus more for reflashing (cold boot)\n",
                  (unsigned long)((SLEEP_COLD_BOOT_AWAKE_MS - awake) / 1000));
  }
  return true;
}

// One pass over every enabled sensor, updating its cached g_* value on a
// successful read and its g_*St status either way. Called every SAMPLE_GAP_MS.
// A sensor is "expected" only if it is enabled AND actually present. A sensor
// that failed to initialise (bmvReady == false) or is switched off by downlink
// must never be waited on, or the readiness gate below would never open and the
// node would go permanently silent. The Calypso has no ready-flag of its own, so
// it counts as expected only once it has EVER produced a reading - an unplugged
// wind sensor must not hold up every other sensor's data.
static bool calypsoEverSeen = false;

static uint8_t pendingSensors(char *out, size_t outCap) {
  uint8_t pending = 0;
  if (out && outCap) out[0] = 0;
  struct { bool expected; bool fresh; const char *name; } checks[] = {
      {bmeReady && g_bmeEnabled, g_bmeFresh, "BME690"},
      {bmvReady && g_bmvEnabled, g_bmvFresh, "BMV080"},
      {sen0466Ready && g_sen0466Enabled, g_coFresh, "SEN0466"},
      {g_cm1106Enabled, g_co2Fresh, "CM1106"},
      {g_calypsoEnabled && calypsoEverSeen, g_calFresh, "Calypso"},
  };
  for (size_t i = 0; i < sizeof(checks) / sizeof(checks[0]); i++) {
    if (checks[i].expected && !checks[i].fresh) {
      pending++;
      if (out && outCap) {
        if (out[0]) strncat(out, ",", outCap - strlen(out) - 1);
        strncat(out, checks[i].name, outCap - strlen(out) - 1);
      }
    }
  }
  return pending;
}

static void clearFreshFlags() {
  g_bmeFresh = g_bmvFresh = g_co2Fresh = g_coFresh = g_calFresh = false;
}

// ---------------- Sequential sensor slots ----------------
// ONE sensor active at a time, never overlapping. This is the fix for the resets
// that blocked this project for days, and it is the single most important
// property of this file - do not "optimise" it back into a single pass.
//
// The measurements that settled it, on this hardware:
//
//   all sensors active together      31 resets / 140s
//   BMV080 disabled, rest together    1 reset  / 140s
//   sequential slots, everything on   2 resets / 150s   (both were the reflash)
//
// The BMV080's laser alone is ~68mA - an order of magnitude more than anything
// else here - and the old code left it measuring while the BME690, CM1106 and
// SEN0466 were all read. That overlap is what browned the board out. Staggering
// the rails and soft-starting the gate both failed because they addressed INRUSH,
// and the problem was steady-state overlap.
//
// notes/power_budget.md specified this from the beginning: "switched rails so the
// 68mA BMV080 and 5mA/210s SEN0466 only draw during their windows".
enum Slot {
  SLOT_BME690 = 0,
  SLOT_BMV080,
  SLOT_CM1106,
  SLOT_SEN0466,
  SLOT_CALYPSO,
  SLOT_COUNT
};

// Names the slot in the log. With one sensor serviced per wake, the log is the
// only way to see which one a given tick actually touched.
static const char *slotName(int slot) {
  switch (slot) {
    case SLOT_BME690: return "BME690";
    case SLOT_BMV080: return "BMV080";
    case SLOT_CM1106: return "CM1106";
    case SLOT_SEN0466: return "SEN0466";
    case SLOT_CALYPSO: return "Calypso";
  }
  return "?";
}

// Longest the BMV080's laser may stay on waiting for a frame. Frames normally
// arrive ~2-4s after the settle. This bounds one slot; BMV080_MEASURE_TIMEOUT_MS
// is far too long to spend inside a sequential pass.
#define BMV080_SLOT_MS 8000UL

// Gap between ordinary slots - enough for one small load to stop drawing before
// the next starts.
#define SLOT_GAP_MS 1500UL

// How long the sensor rails are held OFF after the BME690's gas heater runs.
// A gap alone was not enough at 250ms or at 1500ms; only an actual rail cycle let
// the BMV080 start afterwards. This is that cycle's off-time.
#define HEATER_RECOVERY_OFF_MS 800UL

// Settling time after the sensor rails are cut and before the radio transmits.
// Long enough for the rails to actually discharge their loads and for whatever
// bulk capacitance is on the supply to recover before the burst.
#define TX_RAIL_SETTLE_MS 400UL

// Sensor RAILS off - not merely the sensors idled.
//
// quiesceAll() stops the BMV080 measuring and drops the CM1106's EN line, but
// leaves both sensor rails powered, so every part on them keeps drawing its idle
// current through the transmit. Measured consequence: the board takes a POWERON
// reset - a true supply collapse - immediately after "Setup complete." on a
// transmit tick, i.e. exactly at radio.send().
//
// The radio needs neither rail. So the transmit gets a genuinely exclusive slot:
// 3V3 and 5V off, LoRa rail left up, settle, then send. This is the same principle
// that fixed the sensor resets, applied to the one load that was still sharing.
//
// The rails are NOT brought back afterwards - a transmit is the last thing a tick
// does before sleeping, and enterDeepSleep() switches them off anyway. On a tick
// that both reads and transmits, the read has already finished by this point.
static void powerDownSensorRails() {
  g_railsUp = false;
  digitalWrite(PIN_PCB_EN_A, (PIN_PCB_EN_A_ACTIVE == LOW) ? HIGH : LOW);
  digitalWrite(PIN_PCB_EN_B, (PIN_PCB_EN_B_ACTIVE == LOW) ? HIGH : LOW);
  Serial.println("[tx] sensor rails off for the transmit");
  Serial.flush();
  delay(TX_RAIL_SETTLE_MS);
}

// Everything off. Called between slots and before every transmit, so the radio's
// ~120mA burst never lands on top of a laser or a warming heater.
static void quiesceAll() {
  if (bmvReady && g_bmvPhase != BmvPhase::Idle) {
    bmv080.stopMeasurement();
    g_bmvPhase = BmvPhase::Idle;
  }
  cm1106.sleep();  // EN low
}

// Runs exactly one sensor's slot. Blocking BY DESIGN: the whole point is that
// nothing else is drawing while this one is. Each slot bounds its own time.
static void runSlot(int slot) {
  switch (slot) {
    case SLOT_BME690: {
      if (!bmeReady || !g_bmeEnabled) break;
      Reading env = bme690.read();
      g_bmeSt = env.status;
      if (env.ok()) {
        g_bmeFresh = true;
        g_temp = env.values[0];
        g_hum = env.values[1];
        g_pres = env.values[2];
        g_gas = env.values[3];
      }
      break;
    }

    case SLOT_BMV080: {
      if (!bmvReady || !g_bmvEnabled) break;
      // Laser on only for this slot, off before the slot ends - on every path.
      if (!bmv080.startMeasurement()) {
        Serial.printf("[bmv080] start FAILED - SDK status %d (see bmv080_status_code_t)\n",
                      bmv080.lastStartStatus());
        Serial.flush();

        // NOTE: a GPIO12/GPIO14 sweep used to sit here, from before the board was
        // described. It has been REMOVED and must not come back in that form:
        // GPIO12 goes to the Cubic CM1106 and GPIO14 is the analog supercap sense,
        // so that loop was driving a working sensor's line and fighting an analog
        // divider. The enable turned out to be GPIO15 (PIN_BMV080_EN), asserted
        // once with the rail gates.
        g_bmvSt = ReadingStatus::NoAck;
        break;
      }
      g_bmvPhase = BmvPhase::Measuring;
      uint32_t t0 = millis();
      delay(BMV080_SETTLE_MS);  // preheat: frames before this are not meaningful
      while (millis() - t0 < BMV080_SLOT_MS) {
        Reading pm = bmv080.read();
        g_bmvSt = pm.status;
        if (pm.ok()) {
          g_bmvFresh = true;
          g_pm1 = pm.values[0];
          g_pm25 = pm.values[1];
          g_pm10 = pm.values[2];
          break;
        }
        delay(50);
      }
      bmv080.stopMeasurement();
      g_bmvPhase = BmvPhase::Idle;
      Serial.printf("[bmv080] laser off after %lums (pm2.5=%.1f, %s)\n",
                    (unsigned long)(millis() - t0), g_pm25,
                    g_bmvFresh ? "frame" : "no frame");
      break;
    }

    case SLOT_CM1106: {
      if (!g_cm1106Enabled) break;
      // Single-shot part: it measures on power-up and then repeats that value for
      // the rest of the boot, so a FRESH number costs an EN power cycle every
      // time. That is not a workaround for a fault - it is how the part behaves.
      cm1106.powerCycle();
      Reading co2 = cm1106.read();
      g_co2St = co2.status;
      if (co2.ok()) {
        g_co2Fresh = true;
        g_co2 = co2.values[0];
      }

      // Wire bytes, not the parsed ppm. A ppm that never moves is ambiguous;
      // byte-identical frames are not. Note resp[6] is a counter that increments
      // every read, so frozenValueRun compares the VALUE bytes, never whole frames.
      Serial.print("[cm1106 raw] ");
      if (cm1106.lastRawLen() == 0) {
        Serial.print("(nothing received)");
      } else {
        for (uint8_t i = 0; i < cm1106.lastRawLen(); i++)
          Serial.printf("%02X ", cm1106.lastRaw()[i]);
      }
      Serial.printf("| status=%s frozenValueRun=%u\n", statusName(co2.status),
                    (unsigned)cm1106.frozenValueRun());

      cm1106.sleep();  // EN back off before the next slot starts
      break;
    }

    case SLOT_SEN0466: {
      if (!sen0466Ready || !g_sen0466Enabled) break;
      Reading co = sen0466.read();
      g_coSt = co.status;
      if (co.ok()) {
        g_coFresh = true;
        g_co = co.values[0];
        g_coTemp = co.values[1];
      }
      break;
    }

    case SLOT_CALYPSO: {
      if (!g_calypsoEnabled) break;
      // Streams NMEA unprompted - drop the stale buffer, then poll briefly for a
      // fresh checksum-valid $--MWV.
      calypso.flushInput();
      Reading wind;
      uint32_t t0 = millis();
      do {
        wind = calypso.read();
        if (wind.ok()) break;
        delay(10);
      } while (millis() - t0 < CALYPSO_READ_WINDOW_MS);
      g_calSt = wind.status;
      if (wind.ok()) {
        g_calFresh = true;
        calypsoEverSeen = true;  // only now does it join the readiness gate
        g_windAngle = wind.values[0];
        g_windSpeed = wind.values[1];
        g_windValid = wind.values[2] > 0.5f;
      }
      break;
    }

    default:
      break;
  }
}

// Builds the CSV payload from the cache and transmits it to Module B, then
// holds a short window open for a CFG downlink.
// Copies the current cached values into the RTC store. Oldest is dropped when
// full, and the loss is counted - a batch that silently covered less time than it
// claims would be worse than one that admits the gap.
static void storeCurrentReading() {
  if (g_storeCount >= READING_STORE_MAX) {
    for (uint16_t i = 1; i < READING_STORE_MAX; i++) g_store[i - 1] = g_store[i];
    g_storeCount = READING_STORE_MAX - 1;
    g_storeDropped++;
  }
  StoredReading &r = g_store[g_storeCount++];
  r.temp = g_temp; r.hum = g_hum; r.pres = g_pres; r.gas = g_gas;
  r.pm1 = g_pm1; r.pm25 = g_pm25; r.pm10 = g_pm10;
  r.co2 = g_co2; r.co = g_co; r.coTemp = g_coTemp;
  r.windAngle = g_windAngle; r.windSpeed = g_windSpeed;
  r.windValid = g_windValid ? 1 : 0;
  r.tickAge = g_wakeCount;  // absolute tick; converted to an age at transmit time
  Serial.printf("[store] reading %u/%u kept (tick %lu)\n", (unsigned)g_storeCount,
                (unsigned)READING_STORE_MAX, (unsigned long)g_wakeCount);
}

// ---- FIRE THRESHOLDS, from the project's alarma/prealarma spreadsheet ----
//
// Transcribed 2026-09-28. Four numbers per magnitude, matching the sheet's four
// columns: an ON threshold to ENTER each level, an OFF threshold to LEAVE it. The
// gap between them is hysteresis - without it a reading sitting on a limit would
// toggle the node in and out of pre-alarm on sensor noise alone, and pre-alarm
// does not sleep.
//
// DIRECTION IS PER MAGNITUDE and is not cosmetic. Temperature, particulates, CO
// and wind trip when they RISE. HUMIDITY TRIPS WHEN IT FALLS - dry air is the fire
// indicator - so its thresholds descend (25 -> 20 -> 15 -> 10) and the comparison
// inverts with them. Getting that backwards would leave a node silent in exactly
// the conditions it exists to detect.
//
// ONE magnitude crossing is enough to change level (OR, not AND).
//
// A table rather than scattered #defines so it can be read straight down against
// the spreadsheet and checked - the only way this stays honest as numbers change.
struct AlarmRule {
  const char *name;
  const float *value;
  uint8_t source;       // which sensor must have read this cycle - see ruleHasData()
  bool higherIsWorse;
  float preOff, preOn;
  float almOff, almOn;
  bool enabled;
};

#define ALARM_SRC_BME 0
#define ALARM_SRC_BMV 1
#define ALARM_SRC_CO 2
#define ALARM_SRC_CO2 3
#define ALARM_SRC_WIND 4

static const AlarmRule kAlarmRules[] = {
    // name        value         source          higher  preOff    preOn   almOff    almOn  on
    {"temp",      &g_temp,      ALARM_SRC_BME,  true,    45.0f,   50.0f,   55.0f,   60.0f, true},
    {"rh",        &g_hum,       ALARM_SRC_BME,  false,   25.0f,   20.0f,   15.0f,   10.0f, true},
    {"pm1",       &g_pm1,       ALARM_SRC_BMV,  true,    90.0f,  100.0f,  225.0f,  250.0f, true},
    {"pm25",      &g_pm25,      ALARM_SRC_BMV,  true,    90.0f,  100.0f,  225.0f,  250.0f, true},
    {"pm10",      &g_pm10,      ALARM_SRC_BMV,  true,    90.0f,  100.0f,  225.0f,  250.0f, true},
    {"co",        &g_co,        ALARM_SRC_CO,   true,     6.0f,    8.0f,   15.0f,   20.0f, true},
    {"windSpeed", &g_windSpeed, ALARM_SRC_WIND, true,     2.0f,    5.0f,    8.0f,   10.0f, true},

    // ---- DISABLED, each for a measured reason rather than an oversight ----
    //
    // gas: the sheet gives 15000/7000 ohm, but BME690 gas resistance climbs from
    // ~6k cold to ~45k as the heater stabilises, and under the sleep cycle EVERY
    // burst starts cold. Nodes C2 and C3 read 5685 ohm in clean air - below the
    // sheet's full-alarm figure - while C1 reads 47033. The same sensor in the same
    // room spans both, so an absolute threshold cannot separate a cold heater from
    // smoke. Needs a heater-settled reading before any number here means anything.
    {"gas",  &g_gas,  ALARM_SRC_BME, false, 18000.0f, 15000.0f, 9000.0f, 7000.0f, false},

    // co2: the sheet gives 800/1000 ppm, but all three nodes read 1104-1517 ppm in
    // clean indoor air, already above full alarm; the sheet's own reference is 555.
    // Separately the CM1106's reading FREEZES within a boot and only changes across
    // power cycles, so a trigger on it would be blind to a real fire and liable to
    // latch on a stale number.
    {"co2",  &g_co2,  ALARM_SRC_CO2, true,   750.0f,  800.0f,  900.0f, 1000.0f, false},

    // pres: all four cells in the sheet are 1031 hPa - no hysteresis, no gap
    // between levels, no stated direction. The sheet's reference is 970 and the
    // nodes read 943-950. As written it would never fire or always fire.
    {"pres", &g_pres, ALARM_SRC_BME, true,  1031.0f, 1031.0f, 1031.0f, 1031.0f, false},
};

#define ALARM_RULE_COUNT (sizeof(kAlarmRules) / sizeof(kAlarmRules[0]))

// ---- LIVE THRESHOLDS ----
//
// kAlarmRules above is now the DEFAULTS. The values actually compared against
// live here, in RTC memory, so Module A can change one over the air without a
// reflash - which is the whole point of the umbral command.
//
// RTC_NOINIT so a change survives deep sleep: a node that forgot its thresholds
// on every wake would silently revert to the compiled defaults ten seconds after
// being configured, and the dashboard would show a value the node was not using.
//
// Order matches kAlarmRules exactly. [0]=preOff [1]=preOn [2]=almOff [3]=almOn.
#define THR_PRE_OFF 0
#define THR_PRE_ON 1
#define THR_ALM_OFF 2
#define THR_ALM_ON 3
RTC_NOINIT_ATTR float g_thr[ALARM_RULE_COUNT][4];

// Copies the compiled defaults over the live values. Called only when the boot
// magic changes - see the BOOTCOUNT_MAGIC block.
static void seedThresholds() {
  for (size_t i = 0; i < ALARM_RULE_COUNT; i++) {
    g_thr[i][THR_PRE_OFF] = kAlarmRules[i].preOff;
    g_thr[i][THR_PRE_ON] = kAlarmRules[i].preOn;
    g_thr[i][THR_ALM_OFF] = kAlarmRules[i].almOff;
    g_thr[i][THR_ALM_ON] = kAlarmRules[i].almOn;
  }
}

// Sets one threshold by its dashboard name, e.g. "temp_pre_on" or
// "pm25_alarma_off". Returns false if the name matches no rule, so an unknown key
// is reported rather than silently accepted - a threshold that looks applied but
// is not is the worst outcome available here.
// Resolves "temp_pre_on" to the exact cell the alarm logic reads, or nullptr.
//
// Split out of setThresholdByName so the ACK can READ BACK what was stored
// instead of echoing the string that arrived - see the call site. One lookup
// for both directions means the value Module A is told cannot disagree with
// the value this node compares against.
static float *thresholdSlot(const char *key) {
  static const struct { const char *suffix; uint8_t idx; } kSuffix[] = {
      {"_pre_off", THR_PRE_OFF},
      {"_pre_on", THR_PRE_ON},
      {"_alarma_off", THR_ALM_OFF},
      {"_alarma_on", THR_ALM_ON},
      // The protocol document uses these instead of the spreadsheet's names.
      // Both are accepted rather than guessing which Module A will send.
      {"_alarm_off", THR_ALM_OFF},
      {"_alarm_on", THR_ALM_ON},
  };

  const size_t klen = strlen(key);
  for (size_t sfx = 0; sfx < sizeof(kSuffix) / sizeof(kSuffix[0]); sfx++) {
    const size_t slen = strlen(kSuffix[sfx].suffix);
    if (klen <= slen) continue;
    if (strcmp(key + klen - slen, kSuffix[sfx].suffix) != 0) continue;

    const size_t namelen = klen - slen;
    for (size_t i = 0; i < ALARM_RULE_COUNT; i++) {
      if (strlen(kAlarmRules[i].name) != namelen) continue;
      if (strncmp(kAlarmRules[i].name, key, namelen) != 0) continue;
      return &g_thr[i][kSuffix[sfx].idx];
    }
  }
  return nullptr;
}

static bool setThresholdByName(const char *key, float value) {
  float *slot = thresholdSlot(key);
  if (!slot) return false;
  *slot = value;
  return true;
}

// Did this rule's sensor actually produce a reading this cycle?
//
// THIS GUARD IS LOAD-BEARING, not defensive tidiness. The reading globals hold 0.0
// when a sensor has not read, and humidity trips on LOW values - so an unread
// BME690 would present rh = 0.0, which is below even the full-alarm threshold of
// 10, and every node with a dead humidity sensor would sit in permanent fire
// alarm. A rule whose sensor is absent, disabled or stale is SKIPPED, never
// evaluated against a zero.
static bool ruleHasData(uint8_t source) {
  switch (source) {
    case ALARM_SRC_BME:  return bmeReady && g_bmeEnabled && g_bmeFresh;
    case ALARM_SRC_BMV:  return bmvReady && g_bmvEnabled && g_bmvFresh;
    case ALARM_SRC_CO:   return sen0466Ready && g_sen0466Enabled && g_coFresh;
    case ALARM_SRC_CO2:  return g_cm1106Enabled && g_co2Fresh;
    case ALARM_SRC_WIND: return g_calypsoEnabled && g_calFresh && g_windValid;
  }
  return false;
}

// Has this rule crossed the ON threshold for the given level? Direction-aware.
static bool ruleTripped(const AlarmRule &r, float v, bool fireLevel, size_t idx) {
  const float on = g_thr[idx][fireLevel ? THR_ALM_ON : THR_PRE_ON];
  return r.higherIsWorse ? (v >= on) : (v <= on);
}

// Has it fallen back past the OFF threshold? The gap between OFF and ON is the
// hysteresis - a value between them holds whatever state it is already in.
static bool ruleCleared(const AlarmRule &r, float v, bool fireLevel, size_t idx) {
  const float off = g_thr[idx][fireLevel ? THR_ALM_OFF : THR_PRE_OFF];
  return r.higherIsWorse ? (v < off) : (v > off);
}

// Highest level any single rule is calling for. OR across magnitudes: one is
// enough. Returns ALARM_NORMAL if nothing is tripped.
static AlarmState evaluateRules(const char **whyOut) {
  AlarmState worst = ALARM_NORMAL;
  for (size_t i = 0; i < ALARM_RULE_COUNT; i++) {
    const AlarmRule &r = kAlarmRules[i];
    if (!r.enabled || !ruleHasData(r.source)) continue;
    const float v = *r.value;
    if (ruleTripped(r, v, true, i)) {
      *whyOut = r.name;
      return ALARM_FIRE;  // nothing outranks this, stop looking
    }
    if (worst == ALARM_NORMAL && ruleTripped(r, v, false, i)) {
      *whyOut = r.name;
      worst = ALARM_PREALARM;
    }
  }
  return worst;
}

// Are ALL enabled rules back below the OFF threshold for the level we are in?
// Every rule must agree before a level clears - one still tripped holds it.
static bool allRulesCleared(bool fireLevel) {
  for (size_t i = 0; i < ALARM_RULE_COUNT; i++) {
    const AlarmRule &r = kAlarmRules[i];
    if (!r.enabled || !ruleHasData(r.source)) continue;
    if (!ruleCleared(r, *r.value, fireLevel, i)) return false;
  }
  return true;
}

static const char *alarmStateName(AlarmState st) {
  switch (st) {
    case ALARM_NORMAL:   return "NORMAL";
    case ALARM_PREALARM: return "PRE-ALARM";
    case ALARM_FIRE:     return "ALARM";
  }
  return "?";
}

// Runs after every sensor read. Raises immediately on a trip; lowers only after
// ALARM_CLEAR_CONSECUTIVE consecutive reads with every rule clear, so one dip
// mid-fire cannot end it. Module A can force-clear via CFG,ALARM=0.
//
// Raising is instant and lowering is slow, deliberately: the cost of being late to
// a fire is not symmetric with the cost of staying alert a few reads too long.
static void updateAlarmState() {
#if FORCE_ALARM_STATE
  // Frozen for a power measurement - no escalation, no clearing, no reading of
  // the sensors at all. See FORCE_ALARM_STATE.
  return;
#endif
  const char *why = "";
  const AlarmState want = evaluateRules(&why);

  // THE CONFIRMATION WINDOW EXPIRING IS ITSELF AN ESCALATION.
  //
  // Checked before the normal comparison, because "still tripped at pre-alarm
  // level after ten minutes" does not raise `want` on its own - the reading has
  // not got any worse, it has simply not gone away, and that is the whole point.
  // Without this the node would sit in pre-alarm indefinitely on a real fire that
  // never quite reached the alarm threshold.
  if (g_alarmState == ALARM_PREALARM && want >= ALARM_PREALARM &&
      g_prealarmMs >= PREALARM_CONFIRM_MS) {
    g_alarmState = ALARM_FIRE;
    g_alarmClearRun = 0;
    Serial.printf("\n*** ALARM: %s still over threshold after %lu min - confirmed ***\n",
                  why, (unsigned long)(PREALARM_CONFIRM_MS / 60000UL));
    Serial.flush();
    return;
  }

  // Escalation is immediate at any level.
  if (want > g_alarmState) {
    const AlarmState from = (AlarmState)g_alarmState;
    g_alarmState = want;
    g_alarmClearRun = 0;
    // The window starts at the initial cause, so it restarts only on a genuine
    // entry into pre-alarm from normal - not on an escalation to full alarm.
    if (from == ALARM_NORMAL) g_prealarmMs = 0;
    Serial.printf("\n*** %s: %s *** (was %s) - staying awake, all sensors continuous\n",
                  alarmStateName(want), why, alarmStateName(from));
    Serial.flush();
    return;
  }

  if (g_alarmState == ALARM_NORMAL) return;

  // Still tripped at the current level - reset the clean-read run.
  if (want >= g_alarmState) {
    g_alarmClearRun = 0;
    return;
  }

  // Step down one level at a time, and only after a run of clean reads.
  const bool leavingFire = (g_alarmState == ALARM_FIRE);
  if (!allRulesCleared(leavingFire)) {
    g_alarmClearRun = 0;
    return;
  }

  if (++g_alarmClearRun < ALARM_CLEAR_CONSECUTIVE) {
    Serial.printf("[alarm] %s, clean read %u/%d\n", alarmStateName((AlarmState)g_alarmState),
                  (unsigned)g_alarmClearRun, ALARM_CLEAR_CONSECUTIVE);
    return;
  }

  g_alarmClearRun = 0;
  g_alarmState = leavingFire ? ALARM_PREALARM : ALARM_NORMAL;
  // Dropping back into pre-alarm from alarm gets a fresh window rather than
  // inheriting an already-expired one, which would re-escalate on the next read.
  g_prealarmMs = 0;
  Serial.printf("\n*** dropped to %s after %d clean reads ***\n",
                alarmStateName((AlarmState)g_alarmState), ALARM_CLEAR_CONSECUTIVE);
  Serial.flush();
}

// Sends everything in the store, then empties it. One stored reading emits the
// EXACT legacy single-reading format, so the default 3/3 configuration needs no
// change on Module B at all; the batch format only appears once the user actually
// raises LORA_TRANS above SENSOR_READ. A batch is split across packets when it
// exceeds the RYLR998's 240-byte limit rather than being truncated.
static void transmitStore() {
  if (!radioReady) {
    Serial.printf("TX skipped (radio not initialized): %u stored reading(s) held\n",
                  (unsigned)g_storeCount);
    return;
  }
  if (g_storeCount == 0) {
    Serial.println("[tx] nothing stored this cycle - not transmitting");
    return;
  }

  if (g_storeDropped > 0) {
    Serial.printf("[tx] WARNING %lu reading(s) were dropped - store too small for the\n"
                  "     configured LORA_TRANS/SENSOR_READ ratio\n",
                  (unsigned long)g_storeDropped);
  }

  // Every sensor off before the radio transmits, then the rails they sit on. The
  // RYLR998 pulls ~120mA on a send and this board cannot supply that on top of the
  // sensor rails' idle draw - it browns out at radio.send() if they are left up.
  quiesceAll();
  delay(SLOT_GAP_MS);
  powerDownSensorRails();

  char packet[241];
  uint16_t sent = 0;
  while (sent < g_storeCount) {
    int len = 0;
    packet[0] = 0;
    uint16_t first = sent;

    if (g_storeCount == 1) {
      const StoredReading &r = g_store[0];
      len = snprintf(packet, sizeof(packet),
                     "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d",
                     r.temp, r.hum, r.pres, r.gas, r.pm1, r.pm25, r.pm10, r.co2, r.co,
                     r.coTemp, r.windAngle, r.windSpeed, (int)r.windValid);
      sent = 1;
    } else {
      // Batch: "B,<count>," then readings separated by ';', each ending in the age
      // in ticks so the receiver can place them in time without a clock here.
      len = snprintf(packet, sizeof(packet), "B,%u,", (unsigned)(g_storeCount - first));
      while (sent < g_storeCount) {
        const StoredReading &r = g_store[sent];
        char one[176];
        uint32_t age = (g_wakeCount >= r.tickAge) ? (g_wakeCount - r.tickAge) : 0;
        int oneLen = snprintf(one, sizeof(one),
                              "%.1f,%.1f,%.1f,%.0f,%.1f,%.1f,%.1f,%.0f,%.1f,%.1f,%.0f,%.1f,%d,%lu;",
                              r.temp, r.hum, r.pres, r.gas, r.pm1, r.pm25, r.pm10, r.co2, r.co,
                              r.coTemp, r.windAngle, r.windSpeed, (int)r.windValid,
                              (unsigned long)age);
        if (oneLen < 0) break;
        if (len + oneLen >= (int)sizeof(packet) - 1) break;  // this packet is full
        memcpy(packet + len, one, oneLen + 1);
        len += oneLen;
        sent++;
      }
      if (sent == first) {  // a single reading could not fit even alone
        Serial.println("[tx] ERROR one reading exceeds the packet size - dropping it");
        sent++;
        continue;
      }
    }

    if (len < 0) len = 0;
    if (len >= (int)sizeof(packet)) len = (int)sizeof(packet) - 1;

    // LED off across the send itself. The transmit burst is the tightest moment
    // in the whole cycle, and the indicator must not be one of the loads competing
    // with it - the same reasoning that quiesces the sensors just above. Yellow is
    // restored immediately after, so it still reads as "working" to the eye.
    ledOff();
    bool ok = radio.send(LORA_RX_ADDR, packet, (uint8_t)len);
    ledWorking();
    Serial.printf("TX [%u..%u/%u] %d bytes: %s  (%s)\n", (unsigned)first, (unsigned)(sent - 1),
                  (unsigned)g_storeCount, len, packet, ok ? "sent" : "send FAILED");
  }

  // Sent means done: clear the store and start accumulating again.
  g_storeCount = 0;
  g_storeDropped = 0;

  // Post-TX listen window - the only moment Module A's CFG can reach this node.
  LoRaMessage msg;
  uint32_t t0 = millis();
  while (millis() - t0 < LORA_POST_TX_LISTEN_MS) {
    if (radio.poll(msg)) {
      handleInboundMessage(msg);
      break;
    }
    delay(1);  // yields to FreeRTOS so the idle task can feed the task WDT
  }
}


// One scheduled read: walk every slot once, in turn, with nothing overlapping.
//
// Note this is deliberately NOT the old "retry until everything is fresh" loop.
// Each slot now bounds its own time internally, and retrying the whole set would
// re-fire the BMV080 laser repeatedly - the exact load pattern that caused the
// resets. A sensor that misses its slot keeps its last known value and is named
// in the log line as STALE.
// Sends a compact health packet alongside the telemetry.
//
// WHY THIS EXISTS: the telemetry payload carries VALUES only. A sensor that dies
// keeps publishing its last cached value, so the data looks perfectly healthy - it
// is exactly how node C1's frozen co2=663 went unexplained for days while the read
// path was dead. With a serial cable that ambiguity is annoying; with the node
// outside and untethered it is fatal to any test, because there is no other way to
// find out.
//
// Deliberately a SEPARATE packet rather than extra fields on the telemetry: adding
// a field would break Module B's 13-field parser, whereas an unrecognised payload
// is recorded verbatim by the porthole logger as kind=other and ignored safely by
// everything else.
//
//   STAT,<wake>,<resetReason>,<nvsBoots>,<bme><bmv><co2><co><wind>,<awakeMs>
//
// The five flags are 1 = read OK this cycle, 0 = not. Read them as the answer to
// "which sensors were actually alive when this packet left".
// Raw millivolts at the supercap sense pin. NOT the supercap terminal voltage -
// the divider ratio is unknown, so this is the number at the ADC. Averaged over a
// few samples because a single ADC reading on this part is noisy.
//
// Reported in the STAT packet so a discharge test can be followed over the radio
// rather than by sitting next to the node with a meter: the node tells you its own
// charge state on every status packet.
// ---- Supercapacitor charge, as a percentage of USABLE ENERGY ----
//
// CALIBRATED 2026-09-23 from the resistor values on the updated PCB.
//
// The sense point is the junction of a two-resistor divider across the pack:
//
//     Vcap --[ 51.1k ]--+-- GPIO14
//                       |
//                    [ 322k ]
//                       |
//                      GND
//
// so the pin sees the LOWER leg and the factor back to the terminals is
//
//     ratio = (Rtop + Rbottom) / Rbottom = (51.1 + 322) / 322 = 1.15870
//
// Sanity check against real readings: a pin reading of 2304 mV - the highest seen
// on a fully charged pack - maps to 2670 mV at the terminals, which is a 2.7V
// supercapacitor cell at full charge. It agrees.
//
// NOTE this is the UPDATED PCB. An older board has no divider fitted at all and
// its pin floats at ~100 mV whatever the pack is doing; that is not a flat battery
// and not a bug. Check the revision before reading anything into the millivolts.
//
// KNOWN LIMITATION AT THE TOP OF THE RANGE. This divider barely divides - it
// scales by 0.863 - so a full 3.8V pack puts 3280 mV on the pin. The ESP32-S3's
// ADC at the default 11dB attenuation is specified to about 3100 mV and is already
// losing linearity well before that, so the last stretch of charge reads
// compressed and eventually clips: above roughly 3.59V at the terminals the pin
// saturates and the gauge stops climbing, likely topping out near 90% rather than
// 100%.
//
// So "not quite reaching 100%" is expected here and is not a charging fault. The
// bottom and middle of the range, which is where the useful information is, are
// unaffected. Fixing it properly means a divider with more attenuation - roughly
// 2:1 would put a full pack near 1.9V, comfortably inside the accurate region.
#define SUPERCAP_DIVIDER_RATIO 1.15870f

// Set to 1 once SUPERCAP_DIVIDER_RATIO above is a MEASURED number.
//
// While this is 0 the percentage is reported as -1 ("unknown") rather than as a
// computed value. That matters: with the placeholder ratio the maths produced a
// confident-looking 0%, which is both wrong and alarming - a node claiming it is
// flat when it is fully charged is worse than a node admitting it does not know.
// The raw millivolts are still sent either way, so a discharge can be followed by
// its trend before calibration.
#define SUPERCAP_CALIBRATED 1

// The pack is rated 3.8V - stated 2026-09-23.
//
// WORTH RECORDING HOW THIS WAS BRIEFLY GOT WRONG, because the mistake is easy to
// repeat: the highest terminal voltage observed during a day's testing was ~2.67V,
// and that was taken as "fully charged", which put the pack at 2.7V and implied
// two cells in parallel. It was not full - the caps were being charged from empty
// all that day and simply never reached the top. A reading is only the top of the
// range if something says the charge had finished. Nothing did.
#define SUPERCAP_V_FULL 3.8f

// STILL THE LEAST CERTAIN NUMBER HERE - it decides where the gauge reads zero.
//
// Two conflicting pieces of evidence:
//   - The original constant claimed the regulator gives up at 2.5V, unsourced.
//   - Observed 2026-09-22: the board brownout-looped with the pin at ~1629 mV,
//     i.e. ~1.89V at the terminals - so it was still running below 2.5V, though
//     that was measured while a bench supply was also misbehaving.
//
// 1.9V is used because it is the one figure actually observed rather than assumed,
// but it is probably too low to be the right ANSWER even if it is the right
// measurement: a 3.8V pack is likely a lithium-ion capacitor, and LICs are damaged
// by discharge below roughly 2.2V. If these are LICs, the floor should protect the
// cell, not just the regulator - raising this to ~2.2V costs a little reported
// capacity and stops the node draining them into damage.
//
// Raising it makes the node report empty sooner, which is the safe direction.
// TIED TO THE HIBERNATION FLOOR, deliberately.
//
// This was 1.9V, the voltage at which the board was once seen to brown out. But
// the node now stops at HIB_ENTER_V to protect the cells, so energy below that is
// not usable energy - it is energy we have decided not to take. Reporting a
// percentage against 1.9V would have the node hibernate at a displayed 11%, which
// reads as a bug.
//
// With the two equal, 0% and "hibernating now" are the same point and the gauge
// means what an operator assumes it means: how much is left before it stops.
#define SUPERCAP_V_EMPTY HIB_ENTER_V





// Percent of USABLE energy remaining, not percent of voltage.
//
// A supercapacitor stores 0.5*C*V^2, so energy falls with the SQUARE of voltage
// while the voltage itself falls linearly. Reporting a voltage percentage would
// say "50%" at 3.1V when only about 35% of the usable energy is left - flattering,
// and misleading exactly when it matters. This uses the energy form:
//
//     (V^2 - Vempty^2) / (Vfull^2 - Vempty^2)
//
// so 50% means roughly half the remaining RUNTIME, which is what anyone reading it
// actually wants to know.
static int supercapPercent(float volts) {
  const float lo = SUPERCAP_V_EMPTY * SUPERCAP_V_EMPTY;
  const float hi = SUPERCAP_V_FULL * SUPERCAP_V_FULL;
  float e = (volts * volts - lo) / (hi - lo);
  if (e < 0.0f) e = 0.0f;
  if (e > 1.0f) e = 1.0f;
  return (int)(e * 100.0f + 0.5f);
}

static int readSupercapMv() {
  const int kSamples = 8;
  uint32_t sum = 0;
  for (int i = 0; i < kSamples; i++) {
    sum += (uint32_t)analogReadMilliVolts(PIN_SUPERCAP_SENSE);
    delay(2);
  }
  return (int)(sum / kSamples);
}

// Pack terminal voltage, through the calibrated divider.
static float supercapVolts() {
  return (readSupercapMv() / 1000.0f) * SUPERCAP_DIVIDER_RATIO;
}

// ---- Low-battery hibernation ----
//
// Quiesce, cut everything, and sleep long. Called when the pack has fallen to
// HIB_ENTER_V. Sensors are told to sleep BEFORE their rail disappears rather than
// having power yanked mid-measurement - the same courtesy the normal sleep path
// pays the BMV080 and CM1106.
//
// Sensor sleep() calls are nearly free here because the rail is about to go anyway
// - an unpowered part draws nothing. They exist so shutdown is orderly, and
// because two of them (BME690, SEN0466) used to be empty stubs that made "tell the
// sensors to sleep" quietly mean nothing at all.
static void enterHibernation(float volts, bool announce) {
  Serial.printf("\n[HIBERNATE] pack at %.2fV, below %.2fV - stopping to protect the cells\n",
                volts, (double)HIB_ENTER_V);
  Serial.flush();

  if (railsArePowered()) {
    if (bmeReady) bme690.sleep();
    if (sen0466Ready) sen0466.sleep();
    if (bmvReady) bmv080.stopMeasurement();
    cm1106.sleep();
  }

#if HIB_ANNOUNCE_ENTRY
  // The ONE transmit hibernation ever makes, and only on the way in. Silence from
  // here is deliberate, but silence with no explanation is indistinguishable at
  // Module A from a dead node - and telling those apart otherwise costs a trip
  // into a forest. Skipped if the radio was never brought up on this wake.
  if (announce) {
    sendHibernationNotice(volts);
  }
#else
  (void)announce;
#endif

  g_hibernating = true;
  ledOff();
  enterDeepSleep("hibernating - low battery", HIB_CHECK_SECONDS);
}

static void transmitStatus() {
  // Flags come from the last READ tick, not from this transmit tick - see
  // g_lastReadHealth. The trailing age says how many ticks ago that was, so a
  // stale snapshot is visible rather than silently assumed current.
  uint8_t h = g_lastReadHealth;
  unsigned long ageTicks =
      (g_wakeCount >= g_lastReadWake) ? (unsigned long)(g_wakeCount - g_lastReadWake) : 0UL;

  int vmv = readSupercapMv();
  float vcap = (vmv / 1000.0f) * SUPERCAP_DIVIDER_RATIO;

  // NO DIVIDER FITTED -> SAY "UNKNOWN", NEVER "0%".
  //
  // Only the updated PCB has the 51.1k/322k divider. On an older board GPIO14
  // floats and reads ~100 mV no matter what the pack is doing - and once the
  // ratio is calibrated, that floats straight through the energy formula and
  // comes out as a confident 0%.
  //
  // A node that is plainly alive and transmitting while reporting a flat battery
  // is worse than one admitting it does not know: it invites someone to replace a
  // healthy pack, and it would fire a low-battery alarm on every old board.
  //
  // The node cannot be running at all from a pack this low - the rail gave up
  // around 1.9V - so a reading under SUPERCAP_V_SANITY_FLOOR means the sense line
  // is absent, not that the energy is gone. One firmware then behaves correctly
  // on both PCB revisions instead of needing a per-board build.
  int pct;
  if (!SUPERCAP_CALIBRATED || vcap < SUPERCAP_V_SANITY_FLOOR) {
    pct = -1;
  } else {
    pct = supercapPercent(vcap);
  }

  char msg[96];
  // Alarm state is APPENDED as a ninth field rather than inserted, so a Module B
  // running older firmware keeps reading the eight it knows by index and simply
  // ignores the extra one. Inserting it would have shifted every field after it
  // and silently corrupted the charge readings - the same class of bug as the
  // batch-parse corruption that shifted every value by two.
  int len = snprintf(msg, sizeof(msg), "STAT,%lu,%d,%lu,%d%d%d%d%d,%lu,%d,%d,%d",
                     (unsigned long)g_wakeCount, (int)esp_reset_reason(),
                     (unsigned long)nvsBoots,
                     (h >> 4) & 1, (h >> 3) & 1, (h >> 2) & 1, (h >> 1) & 1, h & 1,
                     ageTicks, vmv, pct, (int)g_alarmState);
  if (len < 0) len = 0;
  if (len >= (int)sizeof(msg)) len = (int)sizeof(msg) - 1;

  bool ok = radio.send(LORA_RX_ADDR, msg, (uint8_t)len);
  if (!SUPERCAP_CALIBRATED) {
    Serial.printf("[stat] %s  (%d mV at the pin; %% UNCALIBRATED - set the divider ratio) (%s)\n",
                  msg, vmv, ok ? "sent" : "FAILED");
  } else if (pct < 0) {
    Serial.printf("[stat] %s  (%d mV at the pin = %.2fV, below the %.1fV sanity floor - "
                  "no divider fitted? older PCB) (%s)\n",
                  msg, vmv, vcap, (double)SUPERCAP_V_SANITY_FLOOR, ok ? "sent" : "FAILED");
  } else {
    Serial.printf("[stat] %s  (cap %.2fV = %d%% usable, flags from wake %lu, %lu tick(s) ago) (%s)\n",
                  msg, vcap, pct, (unsigned long)g_lastReadWake, ageTicks,
                  ok ? "sent" : "FAILED");
  }
  Serial.flush();
}

// True if a sensor that was expected to work did not read OK this cycle. Drives
// the red LED - "expected" means enabled AND initialised, so an absent Calypso or
// a deliberately skipped SEN0466 does not raise a false alarm.
static bool sensorsDegraded() {
  if (bmeReady && g_bmeEnabled && g_bmeSt != ReadingStatus::Ok) return true;
  if (bmvReady && g_bmvEnabled && g_bmvSt != ReadingStatus::Ok) return true;
  if (g_cm1106Enabled && g_co2St != ReadingStatus::Ok) return true;
  return false;
}

// ONE read tick services EVERY sensor, then the node sleeps. Transmit happens on
// its own tick. This is the requested cycle:
//
//   tick 1,2  idle - wave, straight back to sleep, no rail powered
//   tick 3    wake, read ALL sensors, store, sleep   (sensor_read resets to 0)
//   tick 4    wake, transmit, sleep                  (lora_trans resets to 0)
//
// The slots still run STRICTLY ONE AT A TIME within the tick - that part is not
// negotiable, it is what took this board from 31 resets in 140s to none.
//
// The hard part is the BME690's gas heater at 320C. Reading everything in one
// wake previously failed in a very specific way: the BME690 is read first, its own
// gas value came back low (15764 instead of ~45000) whenever the rail sagged, and
// the next two sensors then could not start:
//
//   gas 52925 -> bmv=OK     co2=OK
//   gas 15764 -> bmv=NoAck  co2=Timeout
//
// Widening the inter-slot gap from 250ms to 1500ms did not help, because the sag
// happens INSIDE the heater cycle rather than after it. What did help was a full
// rail power-cycle between the heater and the next sensor - previously achieved by
// putting them on different ticks. So that recovery is done here instead, inside
// the tick, right after the heater runs: rails down, settle, rails back up.
//
// It costs the BMV080's startup delay once per read tick, which is the price of
// reading everything in one wake rather than spreading it out.
static void recoverRailsAfterHeater() {
  Serial.println("[read] rail recovery after the BME690 heater");
  Serial.flush();

  digitalWrite(PIN_PCB_EN_A, (PIN_PCB_EN_A_ACTIVE == LOW) ? HIGH : LOW);
  digitalWrite(PIN_PCB_EN_B, (PIN_PCB_EN_B_ACTIVE == LOW) ? HIGH : LOW);
  delay(HEATER_RECOVERY_OFF_MS);

  g_railsUp = true;
  digitalWrite(PIN_PCB_EN_A, PIN_PCB_EN_A_ACTIVE);
  digitalWrite(PIN_PCB_EN_B, PIN_PCB_EN_B_ACTIVE);
  delay(PIN_PCB_EN_SETTLE_MS);

  // The BMV080 needs its full startup window after any power cycle or it NoAcks -
  // that is exactly the failure this recovery exists to prevent, so skipping the
  // wait here would defeat the point.
  delay(BMV080_STARTUP_DELAY_MS);
}

// SENTINEL read - every tick, and deliberately cheap.
//
// BME690 with the heater OFF (a couple of hundred milliseconds instead of 10.8
// seconds) plus CO. Nothing here switches a rail or fires a laser, so the whole
// thing costs roughly what the wake itself costs.
//
// The CO sensor is read here only when it is enabled and initialised. It is
// excluded from the sleeping cycle at present because of its 210s settle - once it
// is powered continuously (which needs the 5V rail to stay up, a board change) it
// becomes the fastest signal the node has.
static void readSentinels() {
  uint32_t t0 = millis();

  if (bmeReady && g_bmeEnabled) {
    Reading env = bme690.readFast();
    g_bmeSt = env.status;
    if (env.ok()) {
      g_bmeFresh = true;
      g_temp = env.values[0];
      g_hum = env.values[1];
      g_pres = env.values[2];
      // gas deliberately NOT written - readFast() takes no gas measurement, and
      // overwriting the cached value with 0 would look like a collapse to the
      // alarm logic.
    }
  }

  if (sen0466Ready && g_sen0466Enabled) {
    Reading co = sen0466.read();
    g_coSt = co.status;
    if (co.ok()) {
      g_coFresh = true;
      g_co = co.values[0];
      g_coTemp = co.values[1];
    }
  }

  Serial.printf("[sentinel] bme=%s(T%.1f H%.1f) co=%s(%.2f)  (%lums)\n",
                statusName(g_bmeSt), g_temp, g_hum, statusName(g_coSt), g_co,
                (unsigned long)(millis() - t0));
  Serial.flush();
}

// BURST read - the expensive sensors, on their own slow schedule.
static void readFastSensorsOnce() {
  uint32_t t0 = millis();
  char pending[64];

  for (int slot = 0; slot < SLOT_COUNT; slot++) {
    if (millis() - t0 >= SENSOR_READ_WINDOW_MS) {
      Serial.printf("[read] WINDOW EXPIRED after %lums - slots %d..%d skipped\n",
                    (unsigned long)(millis() - t0), slot, SLOT_COUNT - 1);
      break;
    }

    // PER-SLOT TIMING. The power model needs the time each sensor is actually
    // drawing current, and that is not any of the #defines - those are ceilings
    // and gaps, not what a slot costs in practice. Measured here, in the real
    // sequence, because a slot's cost depends on what ran before it: the BMV080
    // follows a rail cycle, the CM1106 a power-cycle and warm-up.
    //
    // "active" is the slot itself; "recovery" is the enforced quiet time after
    // it, which is also time the node is awake and the ESP is drawing its ~35mA.
    // Both belong in the budget, so both are reported.
    const uint32_t slotT0 = millis();
    runSlot(slot);
    const uint32_t slotMs = millis() - slotT0;

    const uint32_t recT0 = millis();
    quiesceAll();

    // The heater is the one load big enough to stop its neighbours starting, so it
    // gets a rail cycle rather than merely a gap.
    if (slot == SLOT_BME690 && bmeReady && g_bmeEnabled) {
      recoverRailsAfterHeater();
    } else {
      delay(SLOT_GAP_MS);
    }
    Serial.printf("[t] %-8s active=%lums recovery=%lums\n", slotName(slot),
                  (unsigned long)slotMs, (unsigned long)(millis() - recT0));
    Serial.flush();
  }

  Serial.printf("[t] TOTAL burst=%lums\n", (unsigned long)(millis() - t0));
  Serial.flush();

  // Snapshot health NOW, while the statuses refer to reads that just happened.
  g_lastReadHealth = (uint8_t)(((g_bmeSt == ReadingStatus::Ok ? 1 : 0) << 4) |
                               ((g_bmvSt == ReadingStatus::Ok ? 1 : 0) << 3) |
                               ((g_co2St == ReadingStatus::Ok ? 1 : 0) << 2) |
                               ((g_coSt == ReadingStatus::Ok ? 1 : 0) << 1) |
                               (g_windValid ? 1 : 0));
  g_lastReadWake = g_wakeCount;

  uint8_t stillPending = pendingSensors(pending, sizeof(pending));
  Serial.printf("[read] bme=%s(T%.1f H%.1f gas%.0f) bmv=%s(pm2.5=%.1f) co2=%s(%.0f) "
                "co=%s(%.2f) wind=%s(rx=%u)%s%s  (all sensors, %lums)\n",
                statusName(g_bmeSt), g_temp, g_hum, g_gas,
                statusName(g_bmvSt), g_pm25,
                statusName(g_co2St), g_co2,
                statusName(g_coSt), g_co,
                !g_calypsoEnabled ? "off" : (g_windValid ? "OK" : "silent"),
                calypso.lastReadBytes(),
                stillPending ? "  STALE: " : "", stillPending ? pending : "",
                (unsigned long)(millis() - t0));
  if (g_windValid) {
    Serial.printf("[wind] angle=%.1f deg  speed=%.2f\n", g_windAngle, g_windSpeed);
  }
  Serial.flush();
}

void loop() {
  // A run that lasts this long counts as healthy, so the NVS counter is cleared and
  // the next boot starts from zero. Without this every reflash would eventually
  // trip safe mode. 15s comfortably exceeds the time the board has been surviving.
  static bool markedGood = false;
  if (!markedGood && millis() > 15000) {
    markedGood = true;
    bootGuardMarkGood();
  }

  // Safe mode: sensors stay off, but the node does NOT go silent, and it does NOT
  // stay here forever.
  //
  // It used to do both. g_safeMode is decided once in setup(); loop() cleared the
  // NVS counter at 15s and then returned here on every pass, never sleeping and
  // therefore never rebooting to re-evaluate it. A single unlucky boot latched the
  // node into safe mode until a human power-cycled it - and with the radio never
  // initialised, the node could not say so. Node C1 sat like that from 2026-09-11,
  // powered and awake, while it looked from every angle like a dead board.
  if (g_safeMode) {
    static uint32_t lastSaid = 0;
    if (millis() - lastSaid > 5000) {
      lastSaid = millis();
      Serial.printf("[safe mode] sensors off, %lus until auto-recovery reboot\n",
                    (unsigned long)((SAFE_MODE_RECOVER_MS - millis()) / 1000));
      Serial.flush();
    }

    // Announce it over the radio, once. The whole point is that a node in trouble
    // must still be heard - a distress packet is far more useful than silence, and
    // it costs one transmit. Sensors stay off; only the LoRa rail comes up.
    static bool distressSent = false;
    if (!distressSent && millis() > 3000) {
      distressSent = true;
      sendSafeModeDistress();
    }

    // Then reboot back into normal operation. The NVS counter was cleared at 15s
    // above, so the next boot starts clean; if the underlying fault is still there
    // it will simply trip safe mode again and send another distress packet, which
    // is a visible heartbeat rather than a silent latch.
    if (millis() >= SAFE_MODE_RECOVER_MS) {
      Serial.println("[safe mode] recovery window elapsed - restarting into normal mode");
      Serial.flush();
      delay(50);
      esp_restart();
    }

    delay(200);
    return;
  }

  // ---- PRE-ALARM / ALARM: no sleeping, every sensor, continuously ----
  //
  // The node stays awake and keeps reading until the readings settle or Module A
  // clears it, and transmits on ALARM_TX_MIN_GAP_MS rather than the normal
  // schedule - a fire is the one case where latency beats airtime. Each transmit's
  // listen window is also how a clear arrives.
  //
  // STAT goes out alongside the readings, which it previously did not: STAT is the
  // packet that carries the alarm state, so without it Module B could see a node
  // transmitting hard and still have no idea it was in alarm. That is the whole
  // path from "this node detected a fire" to "somebody is told".
  if (g_alarmState >= ALARM_PREALARM) {
#if FORCE_ALARM_STATE
    // Repeat the warning on a running board: someone reading the serial half an
    // hour in should not have to have seen the boot banner to know this is a test
    // build sitting in a fake alarm.
    static uint32_t lastForcedWarn = 0;
    if (millis() - lastForcedWarn > 30000) {
      lastForcedWarn = millis();
      Serial.printf("[TEST BUILD] alarm state FORCED to %s - not from the sensors\n",
                    (FORCE_ALARM_STATE >= 2) ? "ALARM" : "PRE-ALARM");
      Serial.flush();
    }
#endif
    // Accumulate before evaluating, so the window is current when updateAlarmState
    // tests it. Only pre-alarm is timed; full alarm has nothing left to confirm.
    static uint32_t lastAccumMs = 0;
    const uint32_t nowMs = millis();
    if (lastAccumMs != 0 && g_alarmState == ALARM_PREALARM) {
      g_prealarmMs += (nowMs - lastAccumMs);
    }
    lastAccumMs = nowMs;

    readFastSensorsOnce();
    updateAlarmState();
    storeCurrentReading();

    static uint32_t lastAlarmTxMs = 0;
    if (millis() - lastAlarmTxMs >= ALARM_TX_MIN_GAP_MS) {
      lastAlarmTxMs = millis();
      transmitStore();
      transmitStatus();
    }
    delay(SAMPLE_GAP_MS);
    return;  // never falls through to the sleep path while raised
  }

#if !SLEEP_ENABLED
  // ---- CONTINUOUS (bench) ----
  // Sample every pass, transmit on a timer. No sleeping, so the USB port stays up
  // and the LED stays watchable for as long as you need.
  readFastSensorsOnce();
  updateAlarmState();
  storeCurrentReading();

  static uint32_t lastTxMs = 0;
  if (millis() - lastTxMs >= NOSLEEP_TX_GAP_MS) {
    lastTxMs = millis();
    ledWorking();
    transmitStore();
    ledOff();
  }
  delay(SAMPLE_GAP_MS);
  return;
#endif

  // ---- NORMAL: one tick of work, then sleep ----
  // Both counters were advanced in setup(); this decides what this tick owes.
  // Decided in setup(), before anything was powered - see the idle-tick bail-out.
  bool readDue = g_readDue;
  bool txDue = g_txDue;
  bool burstDue = g_burstDue;

  // ALL OF THIS TICK'S WORK RUNS EXACTLY ONCE, however many times loop() is
  // entered.
  //
  // loop() runs repeatedly whenever the node is awake without sleeping: throughout
  // the 30s cold-boot window, and continuously in pre-alarm. The due-flags are
  // decided once in setup() and do not change within a boot, so without this guard
  // every pass repeats the whole tick. Measured before the fix: 17 sentinel reads
  // and SIX IDENTICAL TRANSMITS inside a single cold-boot window - wasted radio
  // energy and duplicate rows arriving at Module B.
  //
  // (The repeated transmits predate the sentinel work; splitting the schedules
  // simply made them visible.)
  //
  // A plain static is the right scope: every tick is a fresh boot out of deep
  // sleep, so this starts false each tick by construction.
  static bool tickWorkDone = false;
  if (!tickWorkDone) {
    tickWorkDone = true;

    // Sentinels first - cheap, and the signals that move first in a fire, so the
    // alarm check sees fresh temperature and CO even on a tick that owes nothing
    // else.
    readSentinels();

    // The expensive group on its own slow schedule. Before the alarm check, so a
    // burst tick decides with particulates and gas included rather than on
    // sentinels alone.
    if (burstDue) {
      g_burstCount = 0;
      readFastSensorsOnce();
    }

    updateAlarmState();

    // Store on a burst tick (a complete set) or a scheduled read tick. A
    // sentinel-only tick does not store: it would fill the batch with rows whose
    // PM and CO2 are just the previous burst's values repeated.
    if (burstDue || readDue) {
      g_sensorReadCount = 0;
      storeCurrentReading();
    }

    if (txDue) {
      g_loraTransCount = 0;
      ledWorking();  // white for the transmit itself
      transmitStore();
      transmitStatus();
    }

    // A burst tick is the only one where the PM and CO2 statuses mean anything, so
    // the full health check waits for it rather than judging on sentinels alone.
    if (burstDue && sensorsDegraded()) {
      Serial.println("[health] a sensor that was expected did not read - LED red");
      Serial.flush();
      ledTrouble();
    }
  }

  if (g_alarmState >= ALARM_PREALARM) return;  // stay awake - handled above

  if (coldBootWindowOpen()) {
    delay(SAMPLE_GAP_MS);
    return;  // still inside the reflash window - do not sleep yet
  }

  enterDeepSleep(readDue || txDue ? "tick work done" : "nothing due");

  // Only reachable when SLEEP_ENABLED is 0.
  delay(SAMPLE_GAP_MS);
}


