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

#define LORA_MY_ADDR 1  // this node's AT+ADDRESS
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

// ---------- BMV080 duty cycling ----------
// The laser draws ~68mA - the largest load in the system by an order of magnitude,
// and notes/power_budget.md budgets it at 20s per 30 min, not continuously. This
// firmware ran it continuously (roughly 90x its energy allowance), and the board
// reset on every boot where it was measuring. It is now started only around an
// actual sample and stopped the moment a frame lands.
#define BMV080_MEASURE_TIMEOUT_MS 30000UL  // give up waiting for a frame after this
#define BMV080_SETTLE_MS 2000UL            // laser preheat before frames are meaningful

// ---------- Fire detection thresholds ----------
// PLACEHOLDER VALUES - these are NOT from the project's alarma/prealarma
// spreadsheet, which was never transcribed into this repo. They are deliberately
// conservative starting points so the state machine can be exercised; replace
// them with the real figures before this means anything in the field.
//
// The CM1106 is deliberately NOT a trigger: its CO2 field freezes within a boot
// and only changes across power cycles (see Cm1106Sensor::frozenValueRun), so a
// fire trigger hung on it would be worthless in both directions - blind to a real
// fire and liable to latch on a stale number.
#define ALARM_PM25_UGM3 50.0f        // smoke: BMV080 PM2.5
#define ALARM_TEMP_C 50.0f           // BME690 temperature
#define ALARM_GAS_DROP_FRAC 0.50f    // BME690 gas resistance falling this far below baseline
#define ALARM_CO_PPM 50.0f           // SEN0466, only while it is in the cycle

// Consecutive all-sensor reads below every threshold before pre-alarm auto-clears.
// More than one, so a single dip does not end an alarm during a real fire.
#define ALARM_CLEAR_CONSECUTIVE 5

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
// So a healthy node at the default 3/3 counters reads as: green blink, dark,
// green blink, dark, green blink then a yellow stretch - repeating. Green with no
// yellow every third tick means ticks are being counted but work is not
// happening; yellow that never goes out means it is stuck awake.
//
// Brightness is moderate and single-to-dual channel. Full white on a WS2812 is
// ~60mA, rivalling the BMV080 laser; this costs a fraction of that, and the green
// flash is brief by design because it fires on every tick including idle ones.
#define LED_GREEN_R 0
#define LED_GREEN_G 120
#define LED_GREEN_B 0
#define LED_YELLOW_R 120
#define LED_YELLOW_G 80
#define LED_YELLOW_B 0
#define LED_FLASH_MS 120

// Each state is driven BOTH ways, because we know the pin (GPIO21) but not the
// type. neopixelWrite() first - which lights a WS2812 and leaves a plain LED dark
// - then a millisecond for a WS2812 to latch its colour, then a plain level. A
// WS2812 holds what it latched and ignores the steady level that follows; a plain
// LED responds to that level. Whichever is fitted, one of the two lights it and
// the other does nothing.
//
// Consequence worth knowing: on a plain single-colour LED the green/yellow
// distinction collapses to on/off, so the wake flash and the working state look
// the same. The timing still distinguishes them - a 120ms blink versus a hold of
// several seconds.
static void ledWrite(uint8_t r, uint8_t g, uint8_t b, bool level) {
  neopixelWrite(PIN_STATUS_LED, r, g, b);
  delay(1);
  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, level ? HIGH : LOW);
}

static void ledOff() {
  ledWrite(0, 0, 0, false);
}

// Brief green blink: "I woke up". Fires on every wake, so it is kept short - it
// is the one LED cost paid by idle ticks, which are otherwise nearly free.
static void ledFlashWake() {
  ledWrite(LED_GREEN_R, LED_GREEN_G, LED_GREEN_B, true);
  delay(LED_FLASH_MS);
  ledOff();
}

// Held for the whole of a working tick, not blinked, so its length is meaningful.
static void ledWorking() {
  ledWrite(LED_YELLOW_R, LED_YELLOW_G, LED_YELLOW_B, true);
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
#define SLEEP_ENABLED 0

// Transmit cadence when SLEEP_ENABLED is 0. The tick counters cannot drive it in
// this mode - they advance once per boot, and without sleep there are no reboots,
// so they would sit at 1/3 forever and nothing would ever be sent.
#define NOSLEEP_TX_GAP_MS 15000UL
#define SLEEP_CYCLE_SECONDS 10UL        // one tick; work is scheduled in ticks, not seconds

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
#define SENSOR_READ_EVERY_DEFAULT 3
#define LORA_TRANS_EVERY_DEFAULT 3
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
#define SENSOR_READ_WINDOW_MS 20000UL
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
#define BOOTCOUNT_MAGIC 0xC0FFEE01UL
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
RTC_NOINIT_ATTR uint32_t g_txPeriodMsPersist;

// Tick counters and their end-user thresholds. Counters advance every tick and
// reset on reaching their threshold; thresholds arrive by downlink from Module A.
RTC_NOINIT_ATTR uint16_t g_sensorReadCount;
RTC_NOINIT_ATTR uint16_t g_loraTransCount;
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
enum AlarmState : uint8_t { ALARM_NORMAL = 0, ALARM_PREALARM = 1 };
RTC_NOINIT_ATTR uint8_t g_alarmState;
RTC_NOINIT_ATTR uint16_t g_alarmClearRun;   // consecutive clean reads while in pre-alarm
RTC_NOINIT_ATTR float g_gasBaseline;        // BME690 gas resistance in clean air

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

// Last-known-good readings, refreshed continuously by sampleSensors() - see
// file header. A field stays at its last good value (0 until the first ever
// success) rather than snapping to 0 on a transient miss.
float g_temp = 0, g_hum = 0, g_pres = 0, g_gas = 0;
float g_pm1 = 0, g_pm25 = 0, g_pm10 = 0;
float g_co2 = 0;
float g_co = 0, g_coTemp = 0;
float g_windAngle = 0, g_windSpeed = 0;
bool g_windValid = false;

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

  char kv[200];
  if (!applyConfigCommand(msg.payload, msg.length, kv, sizeof(kv))) {
    return;  // not a recognized CFG command, or nothing in it applied - no ACK
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

static void bootGuardBegin() {
  bootStore.begin("boot", false);
  nvsBoots = bootStore.getUInt("n", 0) + 1;
  bootStore.putUInt("n", nvsBoots);
  Serial.printf("NVS boot count (survives power loss): %lu\n", (unsigned long)nvsBoots);
  Serial.flush();
}

// Called once a run has lasted long enough to count as healthy.
static void bootGuardMarkGood() {
  bootStore.putUInt("n", 0);
  Serial.println("Boot guard: run marked good, NVS counter cleared");
  Serial.flush();
}

// Rails brought up one at a time, with USB given time to enumerate first.
// Switching both rails at the same instant draws enough inrush to pull VBUS down
// through the FETs - the Calypso on 5V plus the bulk caps on both rails is close
// to a short for the first few milliseconds.
//
// The delays are DEBUG values. Once it boots reliably, walk them down to find
// where it breaks: that number is how much soft-start the deployment firmware
// needs, and it matters because on solar plus supercaps there is no USB port
// propping the rail up.
// Soft-start a rail gate by ramping it with PWM instead of switching it hard.
//
// Switching a FET fully on in one step makes the rail's bulk capacitance look
// like a short for the first few milliseconds - the inrush is limited only by
// ESR and trace resistance. The board survives WRITING GPIO11 high and dies in
// the settle window straight after, which is that inrush, not the instruction.
//
// Ramping the gate keeps the FET partly on during the ramp, so it acts as a
// current limiter while the caps charge. The cost is that the FET spends the ramp
// in its linear region dissipating power, which is why the ramp is kept short -
// this is a soft-start, not a regulator.
//
// SOFTSTART_MS is a DEBUG value. Once it boots reliably, walk it down to find
// where it breaks; that number is the soft-start the deployment firmware needs,
// and it matters because on solar plus supercaps nothing else is propping the
// rail up.
#define SOFTSTART_CHANNEL 0
#define SOFTSTART_FREQ_HZ 20000
#define SOFTSTART_RES_BITS 8
#define SOFTSTART_STEPS 64
#define SOFTSTART_MS 400

static void softStartRail(int pin, int activeLevel, const char *name) {
  Serial.printf("%s: soft-start ramp over %dms\n", name, SOFTSTART_MS);
  Serial.flush();

  ledcSetup(SOFTSTART_CHANNEL, SOFTSTART_FREQ_HZ, SOFTSTART_RES_BITS);
  ledcAttachPin(pin, SOFTSTART_CHANNEL);

  const int maxDuty = (1 << SOFTSTART_RES_BITS) - 1;
  for (int i = 0; i <= SOFTSTART_STEPS; i++) {
    int duty = (maxDuty * i) / SOFTSTART_STEPS;
    // An active-LOW gate turns on as duty falls, so the ramp runs the other way.
    ledcWrite(SOFTSTART_CHANNEL, (activeLevel == LOW) ? (maxDuty - duty) : duty);
    delay(SOFTSTART_MS / SOFTSTART_STEPS);
  }

  // Hand the pin back to plain GPIO, fully on. Leaving it on PWM would keep the
  // FET switching forever and put ripple on the rail.
  ledcDetachPin(pin);
  pinMode(pin, OUTPUT);
  digitalWrite(pin, activeLevel);

  Serial.printf("%s: ramp complete, gate held %s\n", name, activeLevel == LOW ? "LOW" : "HIGH");
  Serial.flush();
}

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
  Serial.flush();
  delay(500);

  Serial.println("rails up");
  Serial.flush();
}

static bool g_safeMode = false;

// Defined below, but setup() must be able to sleep before it ever powers a rail.
static void enterDeepSleep(const char *why);

// A cold boot always initialises fully and stays awake, whatever the counters say:
// that is the window in which the board can be reflashed, and skipping init there
// would leave a freshly powered node looking dead for its first few ticks.
static bool isColdBoot();

// What this tick owes. Decided BEFORE anything is powered, so an idle tick can
// return to sleep without bringing up a single rail.
static bool g_readDue = false;
static bool g_txDue = false;

void setup() {
  Serial.begin(115200);
  uint32_t waitStart = millis();
  while (!Serial && millis() - waitStart < 3000) delay(10);
  delay(500);

  Serial.println();
  Serial.println("=== CHIP FOREST + LoRa TX: sensors -> RYLR998 -> ground station ===");
  Serial.flush();

  // First, before any rail is touched: the NVS counter that survives a power loss.
  bootGuardBegin();
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

  if (g_bootMagic != BOOTCOUNT_MAGIC) {
    g_bootMagic = BOOTCOUNT_MAGIC;
    g_bootCount = 0;
    g_wakeCount = 0;
    g_sleepFlag = 0;
    g_txPeriodMsPersist = LORA_TX_PERIOD_MS;
    g_sensorReadCount = 0;
    g_loraTransCount = 0;
    g_sensorReadEvery = SENSOR_READ_EVERY_DEFAULT;
    g_loraTransEvery = LORA_TRANS_EVERY_DEFAULT;
    g_storeCount = 0;
    g_storeDropped = 0;
    g_alarmState = ALARM_NORMAL;
    g_alarmClearRun = 0;
    g_gasBaseline = 0.0f;
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
  if (g_sensorReadCount < 0xFFFF) g_sensorReadCount++;
  if (g_loraTransCount < 0xFFFF) g_loraTransCount++;
  Serial.printf("Tick: sensor_read %u/%u, lora_trans %u/%u\n",
                (unsigned)g_sensorReadCount, (unsigned)g_sensorReadEvery,
                (unsigned)g_loraTransCount, (unsigned)g_loraTransEvery);

  // Green: woke up. Before the idle bail-out below, so every wake flashes -
  // including the two ticks in three that go straight back to sleep.
  ledFlashWake();

  g_readDue = (g_sensorReadCount >= g_sensorReadEvery);
  g_txDue = (g_loraTransCount >= g_loraTransEvery);

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
  if (!g_readDue && !g_txDue && !isColdBoot()) {
    enterDeepSleep("nothing due - no rails powered");
    return;  // unreachable: enterDeepSleep does not return
  }
#else
  // Continuous mode initialises everything every time - there is no idle tick to
  // skip, and bailing out here would leave setup() half-done.
  g_readDue = true;
  g_txDue = true;
#endif

  // Yellow: this tick has real work - rails, sensors, radio. Held until sleep.
  ledWorking();
  g_bootCount++;
  Serial.printf("Boot #%lu since last power loss\n", (unsigned long)g_bootCount);
  Serial.printf("Last reset reason: %d = %s\n", (int)rr, rrName);
  Serial.flush();

  // Deep sleep left these pads latched (see kHeldGates). Nothing can drive them
  // until the hold is released, so this must happen before the pinMode calls
  // below - otherwise the writes are silently ignored and the rails stay off.
  // BISECT: gpio_hold is the one thing the minimal rail sketch never touches, and
  // that sketch holds these same rails for 80+ seconds. If a previous deep-sleep
  // run latched these pads, releasing the hold here is a candidate for the rail
  // misbehaving on assert. Disabled to test that.
#if 0
  for (size_t i = 0; i < sizeof(kHeldGates) / sizeof(kHeldGates[0]); i++) {
    gpio_hold_dis(kHeldGates[i]);
  }
  gpio_deep_sleep_hold_dis();
#endif

  // Rails one at a time, with USB given time to enumerate first - see enableRails().
  // Safe mode after three boots without a good run leaves them off entirely, so a
  // board that cannot hold its supply still gives back a usable serial port.
  // Threshold raised from 3: while the failure is being reproduced deliberately,
  // a 3-boot trigger hijacks every test run before the interesting output appears.
  g_safeMode = (nvsBoots >= 12);
  enableRails(g_safeMode);

  // ---- BISECT (temporary) ----
  // The minimal rail sketch holds all three rails for 80+ seconds with zero
  // resets, while this firmware dies at the same pin write - so the hardware is
  // fine and something here is at fault. The failure point has also moved every
  // time the code changed, which points at corruption rather than a fixed bug.
  //
  // This stops setup() right after the rails: no sensors, no radio, no I2C, no
  // UARTs. If it survives, the fault is in what comes AFTER; if it still dies,
  // the fault is in what comes BEFORE. Remove once that is known.
#if 1
  Serial.println("[bisect] stopping after rails - no sensor or radio init");
  Serial.flush();
  g_safeMode = true;  // makes loop() idle instead of trying to read
  return;
#endif
  if (g_safeMode) {
    Serial.println("Rails off - skipping sensor and radio init this boot.");
    Serial.flush();
    return;  // nothing below can work without power
  }

  power.enable3V3Sensors();
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  Serial.printf("Waiting %dms for BMV080 startup...\n", BMV080_STARTUP_DELAY_MS);
  delay(BMV080_STARTUP_DELAY_MS);

  step("bme690.begin");
  bmeReady = bme690.begin();
  Serial.println(bmeReady ? "BME690: OK" : "BME690: NOT FOUND");

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
  Serial.println(bmvReady ? "BMV080: OK" : "BMV080: NOT FOUND");
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
static void enterDeepSleep(const char *why) {
#if !SLEEP_ENABLED
  (void)why;
  return;
#else
  // Off before sleeping. A WS2812 latches its last value, so without this it would
  // stay lit through the entire sleep - burning current and making the indicator
  // meaningless.
  ledOff();

  Serial.printf("\n[sleep] %s - sleeping %lus\n", why, (unsigned long)SLEEP_CYCLE_SECONDS);

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
  delay(50);  // let the USB CDC drain before the peripheral dies with the sleep

  g_sleepFlag = SLEEP_FLAG_MAGIC;  // so the next boot knows it came from sleep
  esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_CYCLE_SECONDS * 1000000ULL);
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

static void sampleSensors() {
  if (g_calypsoEnabled) {
    // Calypso streams NMEA - drop the stale buffer, then poll briefly for a
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
  }

  // BMV080: laser on only around an actual sample, never continuously. Idle ->
  // Measuring when a fresh PM value is wanted, back to Idle as soon as a frame
  // lands or the timeout expires. Non-blocking - one read() attempt per loop pass,
  // so every other sensor keeps being sampled while this one warms up.
  if (bmvReady && g_bmvEnabled) {
    if (g_bmvPhase == BmvPhase::Idle) {
      if (!g_bmvFresh) {
        if (bmv080.startMeasurement()) {
          g_bmvPhase = BmvPhase::Measuring;
          g_bmvPhaseStartedMs = millis();
        } else {
          g_bmvSt = ReadingStatus::NoAck;
        }
      }
    } else {
      uint32_t elapsed = millis() - g_bmvPhaseStartedMs;
      if (elapsed >= BMV080_SETTLE_MS) {
        Reading pm = bmv080.read();
        g_bmvSt = pm.status;
        if (pm.ok()) {
          g_bmvFresh = true;
          g_pm1 = pm.values[0];
          g_pm25 = pm.values[1];
          g_pm10 = pm.values[2];
          bmv080.stopMeasurement();
          g_bmvPhase = BmvPhase::Idle;
          Serial.printf("[bmv080] frame after %lums, laser off (pm2.5=%.1f)\n",
                        (unsigned long)elapsed, g_pm25);
        }
      }
      if (g_bmvPhase == BmvPhase::Measuring && elapsed >= BMV080_MEASURE_TIMEOUT_MS) {
        bmv080.stopMeasurement();
        g_bmvPhase = BmvPhase::Idle;
        Serial.printf("[bmv080] no frame in %lums - laser off, will retry\n",
                      (unsigned long)elapsed);
      }
    }
  }

  if (bmeReady && g_bmeEnabled) {
    Reading env = bme690.read();
    g_bmeSt = env.status;
    if (env.ok()) {
      g_bmeFresh = true;
      g_temp = env.values[0];
      g_hum = env.values[1];
      g_pres = env.values[2];
      g_gas = env.values[3];
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

  if (g_cm1106Enabled) {
    Reading co2 = cm1106.read();
    g_co2St = co2.status;
    if (co2.ok()) {
      g_co2Fresh = true;
      g_co2 = co2.values[0];
    }

    // Dump the wire bytes, not the parsed ppm. A ppm that never moves is
    // ambiguous; byte-identical frames are not. identicalRun counts consecutive
    // identical responses - a nonzero run means the sensor is repeating itself.
    Serial.print("[cm1106 raw] ");
    if (cm1106.lastRawLen() == 0) {
      Serial.print("(nothing received)");
    } else {
      for (uint8_t i = 0; i < cm1106.lastRawLen(); i++) Serial.printf("%02X ", cm1106.lastRaw()[i]);
    }
    Serial.printf("| status=%s frozenValueRun=%u\n", statusName(co2.status),
                  (unsigned)cm1106.frozenValueRun());

    // Pegged measurement behind a healthy link - only a power cycle has ever
    // cleared it. Drop the fresh flag too: a frozen number must not be published
    // as though it were a live reading.
    if (cm1106.frozenValueRun() >= CM1106_FREEZE_LIMIT) {
      Serial.printf("[cm1106] value frozen for %u reads - power-cycling via EN\n",
                    (unsigned)cm1106.frozenValueRun());
      cm1106.powerCycle();
      g_co2St = ReadingStatus::NotReady;
      g_co2Fresh = false;
    }
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

// Does the newest reading look like fire? Kept separate from the state machine so
// the criteria stay readable and in one place.
static bool readingsIndicateFire(const char **whyOut) {
  if (bmvReady && g_bmvEnabled && g_pm25 >= ALARM_PM25_UGM3) { *whyOut = "PM2.5"; return true; }
  if (bmeReady && g_bmeEnabled) {
    if (g_temp >= ALARM_TEMP_C) { *whyOut = "temperature"; return true; }
    // Gas RESISTANCE falls as VOCs rise, so a drop below a fraction of the clean-air
    // baseline is the smoke signal. Needs a baseline first, hence the guard.
    if (g_gasBaseline > 0.0f && g_gas > 0.0f &&
        g_gas < g_gasBaseline * (1.0f - ALARM_GAS_DROP_FRAC)) {
      *whyOut = "BME690 gas resistance drop";
      return true;
    }
  }
  if (sen0466Ready && g_sen0466Enabled && g_co >= ALARM_CO_PPM) { *whyOut = "CO"; return true; }
  return false;
}

// Runs after every sensor read. Enters pre-alarm on a trip; leaves it only after
// ALARM_CLEAR_CONSECUTIVE consecutive clean reads, so one dip mid-fire cannot end
// it. Module A can also force-clear via CFG,ALARM=0.
static void updateAlarmState() {
  const char *why = "";
  bool fire = readingsIndicateFire(&why);

  if (g_alarmState == ALARM_NORMAL) {
    // Learn the clean-air gas baseline only while nothing looks wrong, or a fire
    // would teach the node that smoke is normal.
    if (!fire && bmeReady && g_gas > 0.0f) {
      g_gasBaseline = (g_gasBaseline <= 0.0f) ? g_gas : (g_gasBaseline * 0.9f + g_gas * 0.1f);
    }
    if (fire) {
      g_alarmState = ALARM_PREALARM;
      g_alarmClearRun = 0;
      Serial.printf("\n*** PRE-ALARM: %s ***  staying awake, all sensors continuous\n", why);
    }
  } else {
    if (fire) {
      g_alarmClearRun = 0;
    } else if (++g_alarmClearRun >= ALARM_CLEAR_CONSECUTIVE) {
      g_alarmState = ALARM_NORMAL;
      g_alarmClearRun = 0;
      Serial.printf("\n*** PRE-ALARM CLEARED after %d clean reads - back to the tick cycle ***\n",
                    ALARM_CLEAR_CONSECUTIVE);
    } else {
      Serial.printf("[alarm] clean read %u/%d\n", (unsigned)g_alarmClearRun, ALARM_CLEAR_CONSECUTIVE);
    }
  }
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

    bool ok = radio.send(LORA_RX_ADDR, packet, (uint8_t)len);
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
  }
}


// Samples the fast sensors repeatedly until each has gone fresh, bounded in time.
// Used for one scheduled read - not the old continuous sampling.
static void readFastSensorsOnce() {
  uint32_t t0 = millis();
  char pending[64];
  while (millis() - t0 < SENSOR_READ_WINDOW_MS) {
    sampleSensors();
    if (pendingSensors(pending, sizeof(pending)) == 0) break;
    delay(SAMPLE_GAP_MS);
  }
  uint8_t stillPending = pendingSensors(pending, sizeof(pending));
  // Calypso status belongs here explicitly. Without it the only evidence about
  // the wind sensor is three zeros in the payload, which cannot distinguish "no
  // bytes arriving at all" from "bytes arriving but failing the checksum" - and
  // those point at completely different faults. rx counts raw bytes seen.
  Serial.printf("[read] co2=%.1f co=%.2f T%.1f H%.1f gas%.0f pm2.5=%.1f | wind=%s(rx=%u)"
                "%s%s\n",
                g_co2, g_co, g_temp, g_hum, g_gas, g_pm25,
                !g_calypsoEnabled ? "off" : (g_windValid ? "OK" : "silent"),
                calypso.lastReadBytes(), stillPending ? "  STALE: " : "",
                stillPending ? pending : "");
  if (g_windValid) {
    Serial.printf("[wind] angle=%.1f deg  speed=%.2f\n", g_windAngle, g_windSpeed);
  }
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

  // Safe mode: rails are off, so there is nothing to read or transmit. Hold the
  // port open and say so, rather than looping through code that cannot work.
  if (g_safeMode) {
    static uint32_t lastSaid = 0;
    if (millis() - lastSaid > 5000) {
      lastSaid = millis();
      Serial.println("[safe mode] rails off, waiting - reflash to clear");
      Serial.flush();
    }
    delay(200);
    return;
  }

  // ---- PRE-ALARM: no sleeping, every sensor, continuously ----
  // The node stays awake and keeps reading until the readings settle or Module A
  // clears it. Transmits still follow the lora_trans schedule so the link is not
  // flooded, but each transmit's listen window is also how a clear arrives.
  if (g_alarmState == ALARM_PREALARM) {
    readFastSensorsOnce();
    updateAlarmState();
    storeCurrentReading();

    static uint32_t lastAlarmTxMs = 0;
    if (millis() - lastAlarmTxMs >= LORA_TX_MIN_GAP_MS) {
      lastAlarmTxMs = millis();
      transmitStore();
    }
    delay(SAMPLE_GAP_MS);
    return;  // never falls through to the sleep path while in pre-alarm
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

  if (readDue) {
    g_sensorReadCount = 0;
    readFastSensorsOnce();
    updateAlarmState();
    storeCurrentReading();
    if (g_alarmState == ALARM_PREALARM) return;  // just tripped - stay awake
  }

  if (txDue) {
    g_loraTransCount = 0;
    transmitStore();
  }

  if (coldBootWindowOpen()) {
    delay(SAMPLE_GAP_MS);
    return;  // still inside the reflash window - do not sleep yet
  }

  enterDeepSleep(readDue || txDue ? "tick work done" : "nothing due");

  // Only reachable when SLEEP_ENABLED is 0.
  delay(SAMPLE_GAP_MS);
}


