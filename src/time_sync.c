
#include "time_sync.h"
#include "leds.h"
#include "sensor.h"
#include "app_timer.h"
#include "mesh_packet.h"
#include "transport_control.h"
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
static tc_tx_config_t m_tx_config;

#define HEARTBEAT_ADV_DATA_TYPE (0x48)

static void send_hearbeat_packet() {
  // Send out time sync packet

  mesh_packet_t *p_packet;
  if (mesh_packet_acquire(&p_packet)) {

    const uint8_t length = sizeof(heartbeat_ad_t);

    mesh_packet_set_local_addr(p_packet);

    p_packet->header.length = (BLE_GAP_ADDR_LEN + 1 /* adv_data_length */ + 1 /* adv_data_type */) + length;
    p_packet->header.type = BLE_PACKET_TYPE_ADV_NONCONN_IND;

    ble_ad_t* p_adv_data = (ble_ad_t*) &p_packet->payload[0];

    /* fill  adv data header fields */
    p_adv_data->adv_data_length = 1 + length;
    p_adv_data->adv_data_type = HEARTBEAT_ADV_DATA_TYPE;  // Normal mesh packets are MESH_ADV_DATA_TYPE (0x16)

    heartbeat_ad_t* p_heartbeat_ad = (heartbeat_ad_t*) &p_adv_data->data[0];
    p_heartbeat_ad->sensor_id = get_sensor_id();
    p_heartbeat_ad->epoch_time = m_current_time;
    p_heartbeat_ad->clock_version = m_clock_version;

    if (tc_tx(p_packet, &m_tx_config) != NRF_SUCCESS) {
      toggle_led(LED_RED);
    }

    mesh_packet_ref_count_dec(p_packet);
  }
}

static void measurement_timer_cb(void * p_context)
{
  m_current_time += 1;
  sensor_update();
  if (m_current_time % 10 == 0) {
    toggle_led(LED_GREEN);
  }

  send_hearbeat_packet();
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

static void received_heartbeat_cb(heartbeat_ad_t *p_heartbeat_ad) {

  // modulo math to handle wraparound
  uint16_t version_delta = p_heartbeat_ad->clock_version - m_clock_version;
  if (version_delta > 0 && version_delta < 0xff00) {
    m_clock_version = p_heartbeat_ad->clock_version;
    set_clock_time(p_heartbeat_ad->epoch_time, 0, CLOCK_SOURCE_RF);
  }
}

static void packet_peek_cb(rbc_mesh_packet_peek_params_t *params) {
  if (params->packet_type == BLE_PACKET_TYPE_ADV_NONCONN_IND &&
      params->p_payload[1] == HEARTBEAT_ADV_DATA_TYPE) {
    received_heartbeat_cb((heartbeat_ad_t*)&params->p_payload[2]);
  }
}

void time_sync_init() {
  m_tx_config.alt_access_address = false;
  m_tx_config.first_channel = 38;
  m_tx_config.channel_map = 1;
  m_tx_config.tx_power = RBC_MESH_TXPOWER_0dBm;

  APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);
  app_timer_create(&m_measurement_timer_ID, APP_TIMER_MODE_REPEATED, measurement_timer_cb);
  app_timer_create(&m_alignment_timer_ID, APP_TIMER_MODE_SINGLE_SHOT, timer_aligned_cb);

  rbc_mesh_packet_peek_cb_set(packet_peek_cb);
}

void set_clock_time(int32_t epoch, uint16_t ms, clock_source_t clock_source) {
  toggle_led(LED_RED);
  if (clock_source == CLOCK_SOURCE_SERIAL) {
    m_clock_version++;
  }
  m_current_time = epoch;
  uint16_t start_delay = (1000 - ms) % 1000;
  app_timer_stop_all();
  start_timer(start_delay);
}
