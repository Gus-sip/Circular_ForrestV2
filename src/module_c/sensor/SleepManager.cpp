#include "SleepManager.h"
#include "esp_sleep.h"

void SleepManager::sleep(uint64_t timerIntervalUs) {
  esp_sleep_enable_timer_wakeup(timerIntervalUs);
  esp_light_sleep_start();
}
