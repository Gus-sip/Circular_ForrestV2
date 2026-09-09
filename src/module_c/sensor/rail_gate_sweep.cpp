/*
 * Rail-gate combination sweep - which gate states can this board survive?
 *
 * Walks all eight combinations of GPIO10 / GPIO11 / GPIO13 and records, for each,
 * whether the board stayed alive while holding it. Nothing else runs: no sensors,
 * no I2C, no radio. The only thing under test is whether energising a given set of
 * rails browns the board out.
 *
 * The results live in NVS, not RTC memory, and that choice is the whole trick.
 * Node C1 browns out hard enough to drop the RTC power domain - its RTC_NOINIT
 * boot counter re-initialised on all 124 resets - so anything kept there would be
 * wiped by exactly the event being measured. NVS is in flash and survives a total
 * power loss, so a combination that kills the board is still on record afterwards.
 *
 * Each combination is marked IN-PROGRESS before it is applied and PASS only after
 * surviving the hold. So if the board dies mid-combination, the next boot finds
 * that entry still IN-PROGRESS and marks it KILLED - the failure records itself.
 *
 * Run it, let it reboot as many times as it likes, and read the final table.
 */

#include <Arduino.h>
#include <Preferences.h>

#define GATE_A_PIN 10  // 3V3 sensor rail
#define GATE_B_PIN 11  // 5V sensor rail
#define GATE_C_PIN 13  // RYLR998 LoRa rail

#define HOLD_MS 6000UL     // long enough for a brownout to show itself
#define SETTLE_MS 400UL    // let rails come up before we start counting

#define COMBO_COUNT 8
#define ST_UNTESTED 0
#define ST_INPROGRESS 1
#define ST_PASS 2
#define ST_KILLED 3

// idx bit 0 -> GPIO10, bit 1 -> GPIO11, bit 2 -> GPIO13. 0 = LOW, 1 = HIGH.
static int lvlA(int i) { return (i & 1) ? HIGH : LOW; }
static int lvlB(int i) { return (i & 2) ? HIGH : LOW; }
static int lvlC(int i) { return (i & 4) ? HIGH : LOW; }

static const char *L(int v) { return v == HIGH ? "HIGH" : "LOW "; }

Preferences prefs;
static uint8_t results[COMBO_COUNT];

static void loadResults() {
  prefs.getBytes("res", results, sizeof(results));
}

static void saveResults() {
  prefs.putBytes("res", results, sizeof(results));
}

static void printTable() {
  Serial.println();
  Serial.println("  #  GPIO10 GPIO11 GPIO13   result");
  Serial.println("  -  ------ ------ ------   ------");
  for (int i = 0; i < COMBO_COUNT; i++) {
    const char *r = "untested";
    switch (results[i]) {
      case ST_INPROGRESS: r = "IN PROGRESS"; break;
      case ST_PASS:       r = "survived"; break;
      case ST_KILLED:     r = "*** BROWNED OUT ***"; break;
    }
    Serial.printf("  %d   %s   %s   %s    %s\n", i, L(lvlA(i)), L(lvlB(i)), L(lvlC(i)), r);
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  delay(400);

  Serial.println();
  Serial.println("=== Rail gate combination sweep (GPIO10/11/13) ===");

  prefs.begin("railtest", false);

  // "reset" in NVS lets a fresh run be forced without reflashing; cleared here so
  // the next boot resumes rather than restarting.
  bool fresh = !prefs.isKey("res");
  loadResults();

  // Anything still IN PROGRESS means the board died while holding it - the whole
  // point of writing the marker before applying the combination.
  for (int i = 0; i < COMBO_COUNT; i++) {
    if (results[i] == ST_INPROGRESS) {
      results[i] = ST_KILLED;
      Serial.printf("Combination %d killed the board - recorded.\n", i);
    }
  }
  if (fresh) {
    for (int i = 0; i < COMBO_COUNT; i++) results[i] = ST_UNTESTED;
    Serial.println("Fresh run - all combinations untested.");
  }
  saveResults();
  printTable();

  pinMode(GATE_A_PIN, OUTPUT);
  pinMode(GATE_B_PIN, OUTPUT);
  pinMode(GATE_C_PIN, OUTPUT);

  // Park everything in the known-safe state between tests: 5V off (that is the
  // rail that kills C1), 3V3 off, LoRa off. Its own gate polarity is irrelevant
  // here - these are raw levels.
  digitalWrite(GATE_A_PIN, HIGH);
  digitalWrite(GATE_B_PIN, LOW);
  digitalWrite(GATE_C_PIN, HIGH);
  delay(500);

  for (int i = 0; i < COMBO_COUNT; i++) {
    if (results[i] != ST_UNTESTED) continue;  // already has a verdict

    Serial.printf("Testing %d: GPIO10=%s GPIO11=%s GPIO13=%s ... ", i, L(lvlA(i)), L(lvlB(i)),
                  L(lvlC(i)));
    Serial.flush();

    // Record the attempt BEFORE energising, so a brownout cannot erase the fact
    // that this combination was the one being held.
    results[i] = ST_INPROGRESS;
    saveResults();
    delay(50);

    digitalWrite(GATE_A_PIN, lvlA(i));
    digitalWrite(GATE_B_PIN, lvlB(i));
    digitalWrite(GATE_C_PIN, lvlC(i));
    delay(SETTLE_MS);

    uint32_t start = millis();
    while (millis() - start < HOLD_MS) {
      delay(100);  // if the board dies in here, the next boot marks it KILLED
    }

    results[i] = ST_PASS;
    saveResults();
    Serial.println("survived");

    // Back to safe before the next one, so each test starts from the same place.
    digitalWrite(GATE_A_PIN, HIGH);
    digitalWrite(GATE_B_PIN, LOW);
    digitalWrite(GATE_C_PIN, HIGH);
    delay(500);
  }

  Serial.println();
  Serial.println("=== SWEEP COMPLETE ===");
  printTable();
  Serial.println("Rails parked safe (5V off). Power-cycle to re-run from scratch");
  Serial.println("only after erasing NVS - otherwise verdicts are kept.");
}

void loop() {
  static uint32_t last = 0;
  if (millis() - last < 15000) return;
  last = millis();
  Serial.println("[done] sweep finished - see the table above.");
}
