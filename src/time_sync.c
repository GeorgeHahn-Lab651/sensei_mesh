
#include "time_sync.h"
#include "leds.h"
#include "sensor.h"
#include "app_timer.h"
#include "config.h"

//  Timer settings
#define APP_TIMER_PRESCALER             31   // divisor value - 1
#define TICKS_PER_SECOND                1024 // f / (APP_TIMER_PRESCALER+1)
#define APP_TIMER_MAX_TIMERS            2
#define APP_TIMER_OP_QUEUE_SIZE         3

static int32_t m_current_time;
static int16_t m_clock_version = 0;
static app_timer_id_t m_alignment_timer_ID;
static app_timer_id_t m_measurement_timer_ID;

static void measurement_timer_cb(void * p_context)
{
  m_current_time += 1;
  main_timer_cb();

  // Todo; this should kick off some amount of time later, after
  // all heartbeats have been observed and collected
  offset_timer_cb();
}

static void start_measurement_timer() {
  app_timer_start(m_measurement_timer_ID, TICKS_PER_SECOND, NULL);
}

static void timer_aligned_cb(void * p_context) {
  m_current_time += 1;
  start_measurement_timer();
}

static void start_timer(uint16_t start_delay) {
  int32_t start_delay_ticks = (start_delay * (TICKS_PER_SECOND/1000.0));
  if (start_delay_ticks > 5) {
    app_timer_start(m_alignment_timer_ID, start_delay_ticks, NULL);
  } else {
    start_measurement_timer();
  }
}

void time_sync_init() {
  APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);
  app_timer_create(&m_measurement_timer_ID, APP_TIMER_MODE_REPEATED, measurement_timer_cb);
  app_timer_create(&m_alignment_timer_ID, APP_TIMER_MODE_SINGLE_SHOT, timer_aligned_cb);
}

void set_clock_time(int32_t epoch, uint16_t ms, clock_source_t clock_source, int16_t clock_version) {

  if (clock_source == CLOCK_SOURCE_RF) {
    // modulo math to handle wraparound
    uint16_t version_delta = clock_version - m_clock_version;
    if (version_delta > 0 && version_delta < 0xff00) {
      m_clock_version = clock_version;
    } else {
      // Older or same clock version
      return;
    }
  } else if (clock_source == CLOCK_SOURCE_SERIAL) {
    m_clock_version++;
  }
  m_current_time = epoch;
  uint16_t start_delay = (1000 - ms) % 1000;
  app_timer_stop_all();
  start_timer(start_delay);
}

int32_t get_clock_time() {
  return m_current_time;
}

int16_t get_clock_version() {
  return m_clock_version;
}
