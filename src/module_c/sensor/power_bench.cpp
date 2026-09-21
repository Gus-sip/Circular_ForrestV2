// CHIP FOREST - Module C power bench
//
// Holds ONE sensor in a steady, indefinite state so a meter has time to settle,
// and switches between them over serial so the whole survey needs ONE flash.
// That matters on this board: the production firmware deep-sleeps, so its USB
// port exists for a second or two per wake and reflashing between every
// measurement means fighting for the port six times over.
//
// There is no sleep here, no tick scheduler and no automatic LoRa. The node sits
// in whatever state you selected until you select another one.
//
// ---------------------------------------------------------------------------
// HOW TO USE IT
//
//   1. Flash, open the serial monitor at 115200, press '?' for the menu.
//   2. Put the meter inline - a USB power meter on the cable, or a multimeter in
//      series with the supply.
//   3. Press '0' (everything off). Write that number down. THIS IS THE ONE THAT
//      MATTERS MOST: every other figure is only meaningful as a difference from
//      a baseline.
//   4. Press '1' (rails on, no sensor running). The step from 0 to 1 is what the
//      rails and the sensors' quiescent draw cost before anything is measuring.
//   5. Step through '2'..'9', giving each 20-30s to settle.
//
//   Sensor current = (reading in that state) - (reading in state 1).
//
// Press 'q' before taking a reading. USB CDC traffic is itself a load, and the
// heartbeat print is a periodic bump in the very measurement you are taking.
// Quiet mode stops all printing until you press a key again.
//
// ---------------------------------------------------------------------------
// WHAT THIS CANNOT TELL YOU
//
// A meter on the USB cable reads the WHOLE BOARD: MCU, regulators, USB PHY and
// the sensor together. That is exactly why the differences matter and the
// absolute numbers do not. If you meter the 3V3 or 5V rail directly instead (see
// module-c-rail-gate, which parks the gates for precisely this), you get the
// sensor side alone - better numbers, more probing.
//
// These are also STEADY-STATE figures. Peaks are not visible on a slow meter:
// the BMV080's laser and the CM1106's lamp both pull hard for a short time, and
// a LoRa transmit is a ~100ms spike. A cheap USB meter averages all of that away.
// ---------------------------------------------------------------------------

#include <Arduino.h>
#include <Wire.h>

#include "Config.h"
#include "pins.h"
#include "radio/RYLR998.h"
#include "sensors/Bme690Sensor.h"
#include "sensors/Bmv080Sensor.h"
#include "sensors/CalypsoSensor.h"
#include "sensors/Cm1106Sensor.h"
#include "sensors/Sen0466Sensor.h"

// RYLR998 wiring - matches chip_forest_lora_tx.cpp. UART0 is free; UART1 and
// UART2 belong to the Calypso and the CM1106.
#define LORA_RX_PIN 4
#define LORA_TX_PIN 5

// Address used only while transmitting on the bench. 3 is this board (C2), and
// 2 is Module B - so the LoRa TX mode doubles as a live link test if Module B
// happens to be listening.
#define LORA_MY_ADDR_BENCH 3
#define LORA_DEST_ADDR_BENCH 2

Bme690Sensor bme690(BME690_I2C_ADDR);
Sen0466Sensor sen0466(SEN0466_I2C_ADDR);
Bmv080Sensor bmv080(BMV080_I2C_ADDR);
Cm1106Sensor cm1106(Serial2, PIN_CM1106_RX, PIN_CM1106_TX, PIN_CM1106_EN, CM1106_WARMUP_MS, CM1106_BAUD);
CalypsoSensor calypso(Serial1, PIN_CALYPSO_RX, PIN_CALYPSO_TX, CALYPSO_BAUD);
RYLR998 radio(Serial0, LORA_RX_PIN, LORA_TX_PIN);

enum class Mode : uint8_t {
  AllOff = 0,
  RailsOnly,
  BmeFast,     // heater OFF
  BmeGas,      // heater ON
  Bmv080,      // laser running continuously
  Cm1106,      // power-cycled per read - it only measures on power-up
  Sen0466,
  Calypso,
  LoraIdle,
  LoraTx
};

static Mode g_mode = Mode::AllOff;
static bool g_quiet = false;
static uint32_t g_modeEnteredMs = 0;
static uint32_t g_lastBeatMs = 0;
static uint32_t g_lastWorkMs = 0;
static uint32_t g_opCount = 0;
static char g_lastValue[96] = "-";
static bool g_ready = false;  // the selected sensor's begin() succeeded

// ---------------------------------------------------------------------------
// Rails
//
// The two gates have OPPOSITE active levels - GPIO10 (3V3) is active LOW and
// GPIO11 (5V) is active HIGH. A single shared "off level" switches one off and
// the other ON, which is how the 5V rail once stayed dark for weeks.
//
// GPIO14 is the supercap sense (ANALOG IN) and GPIO12 goes to the CM1106.
// NEITHER is ever driven here.
// ---------------------------------------------------------------------------
static void railsOn() {
  digitalWrite(PIN_PCB_EN_A, PIN_PCB_EN_A_ACTIVE);
  digitalWrite(PIN_PCB_EN_B, PIN_PCB_EN_B_ACTIVE);
}

static void railsOff() {
  digitalWrite(PIN_PCB_EN_A, (PIN_PCB_EN_A_ACTIVE == LOW) ? HIGH : LOW);
  digitalWrite(PIN_PCB_EN_B, (PIN_PCB_EN_B_ACTIVE == LOW) ? HIGH : LOW);
}

static void bmvEnable(bool on) {
  digitalWrite(PIN_BMV080_EN, on ? PIN_BMV080_EN_ACTIVE
                                 : ((PIN_BMV080_EN_ACTIVE == LOW) ? HIGH : LOW));
}

static void loraEnable(bool on) {
  digitalWrite(PIN_LORA_EN, on ? PIN_LORA_EN_ACTIVE
                               : ((PIN_LORA_EN_ACTIVE == LOW) ? HIGH : LOW));
}

static int supercapMv() { return (int)analogReadMilliVolts(PIN_SUPERCAP_SENSE); }

static const char *modeName(Mode m) {
  switch (m) {
    case Mode::AllOff:    return "0  EVERYTHING OFF (baseline)";
    case Mode::RailsOnly: return "1  rails on, no sensor running";
    case Mode::BmeFast:   return "2  BME690 T/H/P, heater OFF";
    case Mode::BmeGas:    return "3  BME690 + GAS HEATER";
    case Mode::Bmv080:    return "4  BMV080 laser, continuous";
    case Mode::Cm1106:    return "5  CM1106 CO2 (power-cycled per read)";
    case Mode::Sen0466:   return "6  SEN0466 CO";
    case Mode::Calypso:   return "7  Calypso wind (5V, listening)";
    case Mode::LoraIdle:  return "8  LoRa powered, idle";
    case Mode::LoraTx:    return "9  LoRa transmitting continuously";
  }
  return "?";
}

static void printMenu() {
  Serial.println();
  Serial.println("=========== CHIP FOREST power bench ===========");
  Serial.println("  0  everything off .......... THE ESP32-S3 ITSELF. Take this first.");
  Serial.println("     rails off, radio off, nothing initialised - so this reading IS");
  Serial.println("     the MCU + regulators + USB. Press 'l'/'h' here to see what the");
  Serial.println("     CPU clock alone is worth (80 vs 240 MHz).");
  Serial.println("  1  rails on, no sensor ..... rails + quiescent draw");
  Serial.println("  2  BME690, heater OFF ...... the sentinel read");
  Serial.println("  3  BME690 + gas heater ..... the expensive one");
  Serial.println("  4  BMV080 laser ............ continuous");
  Serial.println("  5  CM1106 CO2 .............. power-cycled per read");
  Serial.println("  6  SEN0466 CO .............. 210s settle, be patient");
  Serial.println("  7  Calypso wind ............ 5V rail, passive listen");
  Serial.println("  8  LoRa powered, idle");
  Serial.println("  9  LoRa transmitting ....... repeated 200-byte packets");
  Serial.println();
  Serial.println("  q  quiet - stop printing (take the reading here)");
  Serial.println("  l  CPU 80 MHz      h  CPU 240 MHz");
  Serial.println("  ?  this menu       s  current state");
  Serial.println("===============================================");
  Serial.println("Sensor current = this state - state 1.");
  Serial.println();
}

// Tear the current mode down before building the next one. Sensors on this board
// MUST run one at a time - concurrent operation resets it (31 resets against 2),
// so switching state always goes through a full stop.
static void teardown() {
  switch (g_mode) {
    case Mode::Bmv080:
      bmv080.stopMeasurement();
      bmvEnable(false);
      break;
    case Mode::Cm1106:
      cm1106.sleep();
      break;
    case Mode::Calypso:
      Serial1.end();
      break;
    case Mode::LoraIdle:
    case Mode::LoraTx:
      Serial0.end();
      loraEnable(false);
      break;
    default:
      break;
  }
  railsOff();
  delay(300);  // let the rails actually fall before the next thing comes up
}

static void enterMode(Mode m) {
  teardown();
  g_mode = m;
  g_modeEnteredMs = millis();
  g_opCount = 0;
  g_ready = false;
  snprintf(g_lastValue, sizeof(g_lastValue), "-");

  Serial.println();
  Serial.printf("==> %s\n", modeName(m));

  if (m == Mode::AllOff) {
    Serial.println("    rails off, nothing initialised. Let it settle, then read the meter.");
    return;
  }

  railsOn();
  delay(400);  // rail settle before touching anything on it

  switch (m) {
    case Mode::RailsOnly:
      Serial.println("    rails up, no sensor begun. This is the subtraction baseline.");
      g_ready = true;
      break;

    case Mode::BmeFast:
    case Mode::BmeGas:
      Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
      g_ready = bme690.begin();
      Serial.printf("    bme690.begin -> %s\n", g_ready ? "OK" : "FAILED");
      if (m == Mode::BmeGas) {
        Serial.println("    NOTE: each gas read fires the heater for ~10.8s, so the draw");
        Serial.println("          rises and falls rather than sitting flat.");
      }
      break;

    case Mode::Bmv080:
      Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
      bmvEnable(true);
      delay(200);
      g_ready = bmv080.begin();
      Serial.printf("    bmv080.begin -> %s\n", g_ready ? "OK" : "FAILED");
      if (g_ready) {
        bool started = bmv080.startMeasurement();
        Serial.printf("    startMeasurement -> %s (laser stays on from here)\n",
                      started ? "OK" : "FAILED");
      }
      break;

    case Mode::Cm1106:
      g_ready = cm1106.begin();
      Serial.printf("    cm1106.begin -> %s\n", g_ready ? "OK" : "FAILED");
      Serial.println("    NOTE: single-shot sensor - it only measures on power-up, so this");
      Serial.println("          cycles EN per reading. Expect pulses, not a flat line.");
      break;

    case Mode::Sen0466:
      Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
      g_ready = sen0466.begin();
      Serial.printf("    sen0466.begin -> %s\n", g_ready ? "OK" : "FAILED");
      Serial.println("    NOTE: 210s settle. Readings before that are not trustworthy,");
      Serial.println("          though the CURRENT is valid as soon as it is powered.");
      break;

    case Mode::Calypso:
      g_ready = calypso.begin();
      Serial.printf("    calypso.begin -> %s (5V rail; streams unprompted, we only listen)\n",
                    g_ready ? "OK" : "FAILED");
      break;

    case Mode::LoraIdle:
    case Mode::LoraTx: {
      loraEnable(true);
      delay(PIN_LORA_EN_SETTLE_MS);
      RYLR998Params params{7, 7, 1, 12};  // SF7/BW7/CR1/preamble12, as production
      g_ready = radio.begin(LORA_MY_ADDR_BENCH, 5, 868000000UL, params, &Serial);
      Serial.printf("    radio.begin -> %s\n", g_ready ? "OK" : "FAILED");
      if (m == Mode::LoraTx) {
        Serial.println("    NOTE: a transmit is a ~100ms spike. A slow meter shows the");
        Serial.println("          average, not the peak - that needs a scope across a shunt.");
      }
      break;
    }

    default:
      break;
  }

  Serial.println("    press 'q' when it has settled, then read the meter.");
}

// The work each mode does, called on a slow cadence so the state stays steady
// rather than hammering the bus.
static void doWork() {
  switch (g_mode) {
    case Mode::BmeFast: {
      Reading r = bme690.readFast();
      snprintf(g_lastValue, sizeof(g_lastValue), "T%.1f H%.1f P%.1f",
               r.values[0], r.values[1], r.values[2]);
      break;
    }
    case Mode::BmeGas: {
      Reading r = bme690.read();
      snprintf(g_lastValue, sizeof(g_lastValue), "T%.1f H%.1f gas%.0f",
               r.values[0], r.values[1], r.values[3]);
      break;
    }
    case Mode::Bmv080: {
      Reading r = bmv080.read();
      snprintf(g_lastValue, sizeof(g_lastValue), "pm2.5=%.1f", r.values[0]);
      break;
    }
    case Mode::Cm1106: {
      cm1106.powerCycle();
      Reading r = cm1106.read();
      snprintf(g_lastValue, sizeof(g_lastValue), "co2=%.0f", r.values[0]);
      break;
    }
    case Mode::Sen0466: {
      Reading r = sen0466.read();
      snprintf(g_lastValue, sizeof(g_lastValue), "co=%.2f", r.values[0]);
      break;
    }
    case Mode::Calypso: {
      Reading r = calypso.read();
      snprintf(g_lastValue, sizeof(g_lastValue), "dir=%.0f spd=%.1f rx=%u",
               r.values[0], r.values[1], (unsigned)calypso.lastReadBytes());
      break;
    }
    case Mode::LoraTx: {
      static char payload[201];
      memset(payload, 'A', sizeof(payload) - 1);
      payload[sizeof(payload) - 1] = 0;
      bool ok = radio.send(LORA_DEST_ADDR_BENCH, payload, (uint8_t)(sizeof(payload) - 1));
      snprintf(g_lastValue, sizeof(g_lastValue), "200B -> addr %d: %s",
               LORA_DEST_ADDR_BENCH, ok ? "sent" : "FAILED");
      break;
    }
    default:
      return;  // AllOff, RailsOnly, LoraIdle do nothing by design
  }
  g_opCount++;
}

// How often each mode repeats its work. Fast enough to hold a steady load,
// slow enough not to be a benchmark of the I2C bus.
static uint32_t workPeriodMs() {
  switch (g_mode) {
    case Mode::BmeFast:  return 500;
    case Mode::BmeGas:   return 1000;
    case Mode::Bmv080:   return 2000;
    case Mode::Cm1106:   return 4000;
    case Mode::Sen0466:  return 1000;
    case Mode::Calypso:  return 1000;
    case Mode::LoraTx:   return 1500;
    default:             return 0;  // no work
  }
}

static void handleKey(char c) {
  if (g_quiet && c != 'q') {
    g_quiet = false;
    Serial.println("\n[printing back on]");
  }

  switch (c) {
    case '0': enterMode(Mode::AllOff); break;
    case '1': enterMode(Mode::RailsOnly); break;
    case '2': enterMode(Mode::BmeFast); break;
    case '3': enterMode(Mode::BmeGas); break;
    case '4': enterMode(Mode::Bmv080); break;
    case '5': enterMode(Mode::Cm1106); break;
    case '6': enterMode(Mode::Sen0466); break;
    case '7': enterMode(Mode::Calypso); break;
    case '8': enterMode(Mode::LoraIdle); break;
    case '9': enterMode(Mode::LoraTx); break;

    case 'q':
      g_quiet = true;
      Serial.println("\n[quiet - printing stopped so it is not part of your reading]");
      Serial.println("[any key resumes]");
      Serial.flush();
      break;

    case 'l':
      setCpuFrequencyMhz(80);
      Serial.printf("\n[CPU -> %lu MHz]\n", (unsigned long)getCpuFrequencyMhz());
      break;

    case 'h':
      setCpuFrequencyMhz(240);
      Serial.printf("\n[CPU -> %lu MHz]\n", (unsigned long)getCpuFrequencyMhz());
      break;

    case 's':
      Serial.printf("\n[state] %s | %lus in this mode | CPU %lu MHz | supercap %d mV\n",
                    modeName(g_mode),
                    (unsigned long)((millis() - g_modeEnteredMs) / 1000),
                    (unsigned long)getCpuFrequencyMhz(), supercapMv());
      break;

    case '?': printMenu(); break;
    default: break;
  }
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  delay(400);

  // Gates are outputs. GPIO14 (supercap sense) and GPIO12 (CM1106) are NOT
  // touched - 14 is an analog input and driving 12 fights a working sensor.
  pinMode(PIN_PCB_EN_A, OUTPUT);
  pinMode(PIN_PCB_EN_B, OUTPUT);
  pinMode(PIN_BMV080_EN, OUTPUT);
  pinMode(PIN_LORA_EN, OUTPUT);

  railsOff();
  bmvEnable(false);
  loraEnable(false);

  analogReadResolution(12);

  Serial.println();
  Serial.println("CHIP FOREST - Module C power bench");
  Serial.println("No sleep, no scheduler. One sensor at a time, held indefinitely.");
  printMenu();

  enterMode(Mode::AllOff);
}

void loop() {
  while (Serial.available()) handleKey((char)Serial.read());

  uint32_t period = workPeriodMs();
  if (period && g_ready && millis() - g_lastWorkMs >= period) {
    g_lastWorkMs = millis();
    doWork();
  }

  if (!g_quiet && millis() - g_lastBeatMs >= 2000) {
    g_lastBeatMs = millis();
    Serial.printf("[%3lus] %-38s n=%-4lu %-28s cap=%d mV\n",
                  (unsigned long)((millis() - g_modeEnteredMs) / 1000),
                  modeName(g_mode), (unsigned long)g_opCount, g_lastValue, supercapMv());
  }

  delay(10);  // yields to the idle task - the Task WDT watches CPU0 and panics at 5s
}
