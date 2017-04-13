
#include "epoch.h"
#include "leds.h"
#include "sensor.h"
#include "app_timer.h"

//  Timer settings
#define APP_TIMER_PRESCALER             31   // divisor value - 1
#define TICKS_PER_SECOND                1024 // f / (APP_TIMER_PRESCALER+1)
#define APP_TIMER_MAX_TIMERS            2
#define APP_TIMER_OP_QUEUE_SIZE         3

static int32_t epoch_offset;
static bool started_timer = false;
static app_timer_id_t alignment_timer_ID;
static app_timer_id_t measurement_timer_ID;

static void measurement_timer_cb(void * p_context)
{
  epoch_offset += 1;
  sensor_update();
  if (epoch_offset % 10 == 0) {
    toggle_led(LED_GREEN);
  }
}

static void start_measurement_timer() {
  app_timer_create(&measurement_timer_ID, APP_TIMER_MODE_REPEATED, measurement_timer_cb);
  app_timer_start(measurement_timer_ID, TICKS_PER_SECOND, NULL);
  epoch_offset += 1;  // Timer will kick off 1 second from now; account for this
}

static void timer_aligned_cb(void * p_context) {
  start_measurement_timer();
}

static void start_timer(uint16_t start_delay) {
  APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);
  int32_t start_delay_ticks = (start_delay * (TICKS_PER_SECOND/1000.0));
  if (start_delay_ticks > 5) {
    app_timer_create(&alignment_timer_ID, APP_TIMER_MODE_SINGLE_SHOT, timer_aligned_cb);
    app_timer_start(alignment_timer_ID, start_delay_ticks, NULL);
  } else {
    start_measurement_timer();
  }
}

void set_epoch_time(int32_t epoch, uint16_t ms) {
  epoch_offset = epoch;
  if (!started_timer) {
    started_timer = true;
    uint16_t start_delay = 1000 - ms;
    start_timer(start_delay);
  }
}
