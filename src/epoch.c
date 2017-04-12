
#include "epoch.h"

static int32_t epoch_offset;

void set_epoch_time(int32_t epoch, uint16_t ms) {
  epoch_offset = epoch;
  //uint16_t sleep_time = 1000 - ms;
  // Need to wake in sleep time ms
}
