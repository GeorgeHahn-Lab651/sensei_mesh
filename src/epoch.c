
#include "epoch.h"
#include "leds.h"
#include "sensor.h"
#include "app_timer.h"

//  Timer settings
#define APP_TIMER_PRESCALER             63 // Wraparound value -1
#define APP_TIMER_MAX_TIMERS            1
#define APP_TIMER_OP_QUEUE_SIZE         1


static int32_t epoch_offset;
static bool started_timer = false;
static app_timer_id_t         timer_ID;


static void timeout_cb(void * p_context)
{
  sensor_update();
  toggle_led(LED_RED);
}

static void start_timer() {
  started_timer = true;
  APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);
  app_timer_create(&timer_ID, APP_TIMER_MODE_REPEATED, timeout_cb);
  app_timer_start(timer_ID, 512, NULL);
}

void set_epoch_time(int32_t epoch, uint16_t ms) {
  epoch_offset = epoch;
  //uint16_t sleep_time = 1000 - ms;
  // Need to wake in sleep time ms
  // then start 1s timer
  if (!started_timer) {
    start_timer();
  }
}
