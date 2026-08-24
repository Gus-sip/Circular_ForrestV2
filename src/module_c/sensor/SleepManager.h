#pragma once

#include <Arduino.h>

// Light sleep between the node's 120s cycles. Only a timer wake source is
// needed - the RYLR998 no longer has to listen asynchronously (Module B can
// only reach this node in the brief post-TX window each cycle - see
// chip_forest_lora_tx.cpp's file header). Light sleep (not deep sleep) is
// deliberate: it preserves RAM/peripheral driver state across cycles, so
// sensor warmup (BMV080's 5s, CM1106's 3s) and the RYLR998's AT link only
// happen once at boot, not every 120s - see that file's header for the
// reasoning.
class SleepManager {
public:
  // Arms a fresh one-shot timer for timerIntervalUs (must be re-armed every
  // call - esp_sleep_enable_timer_wakeup() is a countdown-from-now, not a
  // periodic timer) and light-sleeps until it fires.
  void sleep(uint64_t timerIntervalUs);
};
