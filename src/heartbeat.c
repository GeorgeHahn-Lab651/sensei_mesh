#include "heartbeat.h"
#include "mesh_packet.h"
#include "transport_control.h"
#include "config.h"
#include "time_sync.h"
#include "leds.h"

static tc_tx_config_t m_tx_config;

void heartbeat_init() {
  m_tx_config.alt_access_address = false;
  m_tx_config.first_channel = 38;
  m_tx_config.channel_map = 1;
  m_tx_config.tx_power = RBC_MESH_TXPOWER_0dBm;
}

void send_heartbeat_packet() {
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
    p_heartbeat_ad->epoch_time = get_clock_time();
    p_heartbeat_ad->clock_version = get_clock_version();

    if (tc_tx(p_packet, &m_tx_config) != NRF_SUCCESS) {
      toggle_led(LED_RED);
    }

    mesh_packet_ref_count_dec(p_packet);
  }
}

void received_heartbeat(heartbeat_ad_t *p_heartbeat_ad) {
  set_clock_time(p_heartbeat_ad->epoch_time, 0, CLOCK_SOURCE_RF, p_heartbeat_ad->clock_version);
}
