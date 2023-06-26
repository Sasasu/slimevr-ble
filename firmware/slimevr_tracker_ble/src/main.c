#include "zephyr/kernel.h"
#include <stdbool.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main);

int main(void) {
  LOG_INF("Starting SlimeVR Tracker");

  while (true) {
    k_sleep(K_SECONDS(1000));
  }

  return 0;
}
