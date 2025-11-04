#pragma once

#include <iostream>
#include <sstream>
#include <iomanip>
#include <string>
#include <functional>
#include <map>
#include "esp_random.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/components/uart/uart_component.h"
#include "connector_packets.h"
#include "slip_decoder.h"

namespace esphome {
namespace twc_gen2 {

// Encapsulate charger state information specific to a single wall connector.
class ConnectorState {
 public:
  // The RS485 code references the relevant ConnectorState by twcid.
  // The Controller sets the corresponding connector_nr by looking up the serial_number in the configuration.
  // The Controller code references the relevant ConnectorState by connector_nr.
  uint16_t twcid = 0;        // Each Tesla Wall Connector generates a unique random ID for use on the RS485 bus.
  uint8_t connector_nr = 0;  // Connector number (1,2,3) as defined in the yaml file (by serial number).
  uint8_t serial_number[SERIAL_NUMBER_LEN + 1] = {};  // Serial number + null-terminator

  uint8_t max_current = 16;           // Max current configured in the controller
  uint8_t max_current_last_send = 0;  // Max current last send to TWC
  uint8_t twc_max_current = 0;        // Max current last confirmed by the TWC

  // For each TWC on the RS485 bus, a sequence of commands is send.
  // The command_nr points determines what command that will be send next.
  uint8_t command_nr = 0;

  // Set to 1 allow charing to allow charging, or 0 otherwise.
  uint8_t allow_charging = 1;
  // Value that was last send to the TWC, can be used to determine if the value needs to be resend.
  uint8_t allow_charging_last_send = 1;

  // State as reported by the TWC
  uint8_t state = 0;
  // Actual current used by the car as reported by the TWC
  uint8_t actual_current = 0;

  // VIN + null-terminator
  uint8_t vin_incoming[VIN_LEN + 1] = {};
  uint8_t vin[VIN_LEN + 1] = {};

  // Sensor values
  uint8_t power_state_flag = 0;
  uint32_t total_kwh = 0;
  uint16_t phase1_voltage = 0;
  uint16_t phase2_voltage = 0;
  uint16_t phase3_voltage = 0;
  uint8_t phase1_current = 0;
  uint8_t phase2_current = 0;
  uint8_t phase3_current = 0;

  void log_values();
};

// This class takes care of sending and receiving datagrams on the RS485 bus.
class RS485 {
 public:
  RS485(uart::UARTComponent *serial, GPIOPin *flow_control_pin)
      : serial_(serial), flow_control_pin_(flow_control_pin) {}

  void setup();
  void read_incoming_data();

  void set_callback(std::function<void(ConnectorState *)> c) { callback_ = c; }

  ConnectorState *get_or_take_connector_by_twcid(uint16_t twcid);
  ConnectorState *get_connector_by_nr(uint8_t nr);

  void log_connector_state(const ConnectorState *c);

 private:
  void do_callback(ConnectorState *packet);

  void send_presence(bool presence2 = false);
  void send_heartbeat(ConnectorState *c);
  void send_command(uint16_t command, uint16_t send_to);
  void send_data(uint8_t *packet, size_t length);

  void decode_secondary_presence(RESP_PACKET_T *presence);
  void decode_secondary_heartbeat(S_HEARTBEAT_T *heartbeat);
  void decode_power_state(EXTENDED_RESP_PACKET_T *power_state);
  void decode_serial(EXTENDED_RESP_PACKET_T *serial);
  void decode_vin(EXTENDED_RESP_PACKET_T *vin_data);

  uint8_t calculate_checksum(uint8_t *buffer, size_t length);
  bool verify_checksum(uint8_t *buffer, size_t length);

  void process_packet(uint8_t *packet, size_t length);

  std::string packet_to_hexstring(const uint8_t *packet, size_t length);

  std::function<void(ConnectorState *)> callback_ = nullptr;

  static void sender_task(void *pvParameter);

  uart::UARTComponent *serial_;
  GPIOPin *flow_control_pin_{nullptr};
  ConnectorState connector_list_[4];

  SlipDecoder slip_decorder;
};
}  // namespace twc_gen2
}  // namespace esphome
