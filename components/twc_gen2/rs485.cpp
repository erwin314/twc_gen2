
#include <cstring>
#include <cstdarg>
#include <cassert>
#include <sstream>

#include "esphome/core/log.h"
#include "esphome/core/component.h"
#include "esphome/core/entity_base.h"
#include "esphome/components/api/custom_api_device.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"

#include "rs485.h"

// Placeholders when editing in non ESP32 development environment.
#ifndef USE_ESP32
void xTaskCreate(...) { abort(); }
void vTaskDelay(...) { abort(); }
#define portTICK_PERIOD_MS 1
#endif

namespace esphome {
namespace twc_gen2 {

static const char *const TAG = "twc_gen2";

void RS485::setup() {
  ESP_LOGD(TAG, "Setup RS485 bus.");

  // Set the flow control pin of the UART.
  if (this->flow_control_pin_ != nullptr) {
    this->flow_control_pin_->digital_write(false);
  }

  // Start a background task that sends a sequence of commands to all TWC's in an endless loop
  xTaskCreate(this->sender_task, "Rs485SenderTask", 2048, this, 1, NULL);
}

void RS485::read_incoming_data() {
  uint8_t byte;
  // Keep processing while there is serial data available.
  while (serial_->available()) {
    serial_->read_byte(&byte);
    // ESP_LOGD(TAG, "B=%02x", byte);
    if (slip_decorder.add_byte(byte)) {
      // Received a complete Datagram
      // ESP_LOGD(TAG, "Received packet");
      process_packet(slip_decorder.datagram, slip_decorder.datagram_length);
    }
  }
}

ConnectorState *RS485::get_or_take_connector_by_twcid(uint16_t twcid) {
  // Find the connector that belongs to the specified twcid
  for (int i = 0; i < 3; i++) {
    if (connector_list_[i].twcid == twcid) {
      return &(connector_list_[i]);
    }
  }

  // Has the TWC selected the same twcid as our Controller?
  if (twcid == CONTROLLER_TWCID) {
    // Return a reference to a dummy connector entry.
    // The TWC should detect that it is using an unavailable
    // twcid and automaticaly select a new one.
    return &(connector_list_[3]);
  }

  // Find an unused connector for this new twcid
  for (int i = 0; i < 3; i++) {
    if (connector_list_[i].twcid == 0) {
      connector_list_[i].twcid = twcid;
      return &(connector_list_[i]);
    }
  }

  // ERROR: Unable to find the twcid and no-more space in the array.
  // return a reference to the dummy connectorstate.
  // (This should not happen.)
  ESP_LOGE(TAG, "More than three TWC's detected on the rs485 bus.");
  return &(connector_list_[3]);
}

ConnectorState *RS485::get_connector_by_nr(uint8_t nr) {
  // Find the connector that has the specified number.
  // This number can be 1, 2 or 3. And corresponds to the number in the yaml file.
  for (int i = 0; i < 3; i++) {
    if (connector_list_[i].connector_nr == nr) {
      return &(connector_list_[i]);
    }
  }
  // Return null if the connector nr does not exist (yet).
  return nullptr;
}

void RS485::do_callback(ConnectorState *c) {
  // TODO: THIS IS FOR DEBUG ONLY
  if (c->connector_nr == 2) {
    c->log_values();
  }

  // Is there a callback method registered?
  if (callback_ != nullptr) {
    // If we have seen atleast 2 power-state-datagrams,
    // we know that atleast one complete communication cycle has completed.
    // So all fields in the connector state should have sensible values.
    if (c->power_state_flag > 1)
      // Perform callback
      callback_(c);
  }
}

void RS485::send_presence(bool presence2) {
  RESP_PACKET_T presence = {};
  PRESENCE_PAYLOAD_T *presence_payload = (PRESENCE_PAYLOAD_T *) &presence.payload;
  uint16_t sign = 0x77;
  presence.command = presence2 ? htons(PRIMARY_PRESENCE2) : htons(PRIMARY_PRESENCE);
  presence.twcid = CONTROLLER_TWCID;
  presence_payload->sign = sign;
  presence_payload->max_allowable_current = 0x0C80;  // All TWC's are designed for upto 32A (in europe)
  presence.checksum = calculate_checksum((uint8_t *) &presence, sizeof(presence));
  send_data((uint8_t *) &presence, sizeof(presence));
}

void RS485::send_heartbeat(ConnectorState *c) {
  P_HEARTBEAT_T heartbeat = {};
  heartbeat.command = htons(PRIMARY_HEARTBEAT);
  heartbeat.src_twcid = CONTROLLER_TWCID;
  heartbeat.dst_twcid = c->twcid;

  // Do we need to send a new (or the first) max current setting?
  if (c->max_current_last_send != c->max_current) {
    // Default to state 5. This might be changed to another state below.
    heartbeat.state = 0x05;
  }

  if (c->state == 4) {
    // TWC state 4 = Starting to charge.
    // Response: Controller state 5 = Set max current
    heartbeat.state = 0x05;
    // Always send START/STOP charging....
    c->allow_charging_last_send = -1;
  } else if (c->state == 8) {
    // TWC state 8 = Starting to charge (used sometimes after state 4)
    // Response: Controller state 8 = Ack+Set max current
    heartbeat.state = 0x08;
  } else if (c->state == 1) {
    // TWC state 1 = Charging
    // Response: Controller 9 only if max current setting has changed in the controller.
    if (c->max_current_last_send != c->max_current) {
      // Only set max current if it has changed.
      heartbeat.state = 0x09;
    }
  }

  // Any non-zero state requires the max current to be transmitted.
  if (heartbeat.state != 0) {
    uint16_t encodedMax = c->max_current * 100;
    heartbeat.max_current = htons(encodedMax);
    c->max_current_last_send = c->max_current;
  }

  heartbeat.checksum = calculate_checksum((uint8_t *) &heartbeat, sizeof(heartbeat));
  send_data((uint8_t *) &heartbeat, sizeof(heartbeat));
}

void RS485::send_data(uint8_t *packet, size_t length) {
  uint8_t outputBuffer[MAX_PACKET_LENGTH];
  uint8_t i;
  uint8_t j = 0;

  if (length > MAX_PACKET_LENGTH) {
    ESP_LOGD(TAG, "Error - packet larger than maximum allowable size!");
    return;
  }

  uint16_t command = ((uint16_t) packet[0] << 8) | packet[1];
  switch (command) {
    case WRITE_ID_DATE:
    case WRITE_MODEL_NO:
      ESP_LOGD(TAG, "WARNING! WRITE COMMANDS ATTEMPTED!  THESE CAN PERMANENTLY BREAK YOUR TWC.  COMMANDS BLOCKED!");
      return;
  }

  // Create a buffer containing the SLIP encoded datagram
  outputBuffer[j++] = SLIP_END;
  for (i = 0; i < length; i++) {
    switch (packet[i]) {
      case SLIP_END:
        outputBuffer[j++] = SLIP_ESC;
        outputBuffer[j++] = SLIP_ESC_END;
        break;
      case SLIP_ESC:
        outputBuffer[j++] = SLIP_ESC;
        outputBuffer[j++] = SLIP_ESC_ESC;
        break;
      default:
        outputBuffer[j++] = packet[i];
    }
  }
  outputBuffer[j++] = SLIP_END;
  outputBuffer[j++] = 0xFF;

  if (this->flow_control_pin_ != nullptr) {
    this->flow_control_pin_->digital_write(true);
  }

  serial_->write_array(outputBuffer, j);
  // Make sure the serial data has finished sending before
  // putting the RS485 transceiver back into receive mode
  serial_->flush();

  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(false);
}

void RS485::decode_secondary_presence(RESP_PACKET_T *presence) {
  PRESENCE_PAYLOAD_T *presence_payload = (PRESENCE_PAYLOAD_T *) presence->payload;

  // This is probably the first incoming datagram from this TWC.
  // So take a new connector for this twcid.
  ConnectorState *connector = get_or_take_connector_by_twcid(presence->twcid);
}

void RS485::decode_secondary_heartbeat(S_HEARTBEAT_T *heartbeat) {
  ConnectorState *c = get_or_take_connector_by_twcid(heartbeat->src_twcid);

  bool is_changed = false;

  // Check if the state of the TWC has changed
  if (c->state != heartbeat->state) {
    c->state = heartbeat->state;
    is_changed = true;
  }

  // Check if the max current as reported by the TWC has changed.
  uint8_t new_twc_max_current = ntohs(heartbeat->max_current) / 100;
  if (c->twc_max_current != new_twc_max_current) {
    c->twc_max_current = new_twc_max_current;
    is_changed = true;
  }

  // Check if the actual current used by the car has changed.
  uint8_t newCurrent = ntohs(heartbeat->actual_current) / 100;
  if (newCurrent != c->actual_current) {
    c->actual_current = newCurrent;
    is_changed = true;
  }

  if (is_changed) {
    do_callback(c);
  }
}

void RS485::decode_power_state(EXTENDED_RESP_PACKET_T *power_state) {
  POWERSTATUS_PAYLOAD_T *power_state_payload = (POWERSTATUS_PAYLOAD_T *) power_state->payload;

  ConnectorState *c = get_or_take_connector_by_twcid(power_state->twcid);

  if (c->power_state_flag < 3) {
    c->power_state_flag = c->power_state_flag + 1;
  }

  bool changed = false;

  uint32_t total_kwh = ntohl(power_state_payload->total_kwh);
  if (total_kwh != c->total_kwh) {
    c->total_kwh = total_kwh;
    changed = true;
  };

  uint16_t voltage = ntohs(power_state_payload->phase1_voltage);
  if (voltage != c->phase1_voltage) {
    c->phase1_voltage = voltage;
    changed = true;
  };

  voltage = ntohs(power_state_payload->phase2_voltage);
  if (voltage != c->phase2_voltage) {
    c->phase2_voltage = voltage;
    changed = true;
  };

  voltage = ntohs(power_state_payload->phase3_voltage);
  if (voltage != c->phase3_voltage) {
    c->phase3_voltage = voltage;
    changed = true;
  };

  uint8_t current = power_state_payload->phase1_current / 2;
  if (current != c->phase1_current) {
    c->phase1_current = current;
    changed = true;
  };

  current = power_state_payload->phase2_current / 2;
  if (current != c->phase2_current) {
    c->phase2_current = current;
    changed = true;
  };

  current = power_state_payload->phase3_current / 2;
  if (current != c->phase3_current) {
    c->phase3_current = current;
    changed = true;
  };

  if (changed) {
    do_callback(c);
  }

  // ESP_LOGD(
  //     TAG, "Decoded: ID: %04x, Power State Total kWh %d, Phase Voltages: %d, %d, %d, Phase Currents: %d, %d,
  //     %d\r\n", power_state->twcid, ntohl(power_state_payload->total_kwh),
  //     ntohs(power_state_payload->phase1_voltage), ntohs(power_state_payload->phase2_voltage),
  //     ntohs(power_state_payload->phase3_voltage), power_state_payload->phase1_current,
  //     power_state_payload->phase2_current, power_state_payload->phase3_current);
}

void RS485::decode_vin(EXTENDED_RESP_PACKET_T *outer_packet) {
  VIN_PAYLOAD_T *packet = (VIN_PAYLOAD_T *) outer_packet->payload;

  ConnectorState *c = get_or_take_connector_by_twcid(outer_packet->twcid);

  switch (ntohs(outer_packet->command)) {
    case RESP_VIN_FIRST:
      memcpy(c->vin_incoming, packet->vin, 7);
      break;
    case RESP_VIN_MIDDLE:
      memcpy(c->vin_incoming + 7, packet->vin, 7);
      break;
    case RESP_VIN_LAST:
      memcpy(c->vin_incoming + 14, packet->vin, 3);

      // We received the last part of a new VIN.
      // Check if it is different than the current VIN.
      if (memcmp(c->vin, c->vin_incoming, VIN_LEN) != 0) {
        // Copy incoming VIN to the actual VIN.
        memcpy(c->vin, c->vin_incoming, VIN_LEN);
        // Zero out the incoming VIN (for debugging)
        memset(c->vin_incoming, 0, VIN_LEN);
        do_callback(c);
      }

      break;
  }
}

void RS485::decode_serial(EXTENDED_RESP_PACKET_T *serial) {
  SERIAL_PAYLOAD_T *serial_payload = (SERIAL_PAYLOAD_T *) serial->payload;

  ConnectorState *c = get_or_take_connector_by_twcid(serial->twcid);

  // Incoming serial number is different then what we know (which could be zero's)?
  if (memcmp(c->serial_number, serial_payload->serial, SERIAL_NUMBER_LEN) != 0) {
    memcpy(c->serial_number, serial_payload->serial, SERIAL_NUMBER_LEN);
    do_callback(c);
  }
}

uint8_t RS485::calculate_checksum(uint8_t *buffer, size_t length) {
  uint8_t checksum = 0;
  for (size_t i = 1; i < length; ++i) {
    checksum += buffer[i];
  }
  return checksum;
}

bool RS485::verify_checksum(uint8_t *buffer, size_t length) {
  return buffer[length - 1] == calculate_checksum(buffer, length - 1);
}

void RS485::sender_task(void *pvParameter) {
  // Reference to ourselfs is passed thrue parameter to this static method
  RS485 *rs485 = static_cast<RS485 *>(pvParameter);

  //  Send the presence startup messages: 5 * Presence(F), 5 * Presence(T)
  for (uint8_t i = 0; i < 10; i++) {
    rs485->send_presence(i >= 5);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  };

  //  Wait for any TWC to come online
  uint8_t n = 0;
  for (;;) {
    if (rs485->connector_list_[n].twcid != 0)
      break;
    n = (n + 1) % 3;
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }

  //  Infinite loop
  int i = 0;
  ConnectorState *c;
  // uint8_t sequence = 0;
  while (true) {
    // Get next connector that has a valid twcid.
    do {
      c = &(rs485->connector_list_[i]);
      i = (i + 1) % 3;
    } while (c->twcid == 0);

    // Send the heartbeat to this TWC.
    // The heartbeat also controls the current limit.
    rs485->send_heartbeat(c);

    // Give time for the answer to be received.
    // vTaskDelay(500 + random(50, 100) / portTICK_PERIOD_MS);
    uint32_t r = 50 + (esp_random() % 51); // [50, 100]
    vTaskDelay(200 + r / portTICK_PERIOD_MS);

    // Should we send a START or STOP charging command?
    if (c->allow_charging != c->allow_charging_last_send) {
      c->allow_charging_last_send = c->allow_charging;
      if (c->allow_charging)
        rs485->send_command(START_CHARGING, c->twcid);
      else
        rs485->send_command(STOP_CHARGING, c->twcid);
      uint32_t r = 50 + (esp_random() % 51); // [50, 100]
      vTaskDelay(200 + r / portTICK_PERIOD_MS);
    }

    switch (c->command_nr) {
      case 0:
        rs485->send_command(GET_PWR_STATE, c->twcid);
        break;
      case 1:
        rs485->send_command(GET_VIN_FIRST, c->twcid);
        break;
      case 2:
        rs485->send_command(GET_VIN_MIDDLE, c->twcid);
        break;
      case 3:
        rs485->send_command(GET_VIN_LAST, c->twcid);
        break;
      case 4:
        rs485->send_command(GET_SERIAL_NUMBER, c->twcid);
        break;
      case 5:
        rs485->send_command(GET_FIRMWARE_VER_EXT, c->twcid);
        break;
    }
    // Allow time for answer to be received (or a presence packet send be a new TWC)
    r = 50 + (esp_random() % 51); // [50, 100]
    vTaskDelay(200 + r / portTICK_PERIOD_MS);

    // Next command
    c->command_nr = (c->command_nr + 1) % 5;

    // If the car is charging, only get power state.
    if (c->state == 1) {
      c->command_nr = 0;
    }

    // If we have the serial number, skip getting the serial/firmware.
    if ((c->serial_number[0] != 0) && (c->command_nr >= 4)) {
      c->command_nr = 0;
    }

    // vTaskDelay(1000 + random(100, 200) / portTICK_PERIOD_MS);
  }
}

void RS485::send_command(uint16_t command, uint16_t send_to) {
  PACKET_T packet = {};
  packet.command = htons(command);
  packet.twcid = CONTROLLER_TWCID;
  packet.secondary_twcid = send_to;

  // if ((command == STOP_CHARGING) || (command == START_CHARGING)) {
  //   packet.payload[0] = 3;
  //   packet.payload[1] = 3;
  //   packet.payload[2] = 3;
  //   packet.payload[3] = 3;
  //   packet.payload[4] = 3;
  //   packet.payload[5] = 3;
  // }

  packet.checksum = calculate_checksum((uint8_t *) &packet, sizeof(packet));
  send_data((uint8_t *) &packet, sizeof(packet));
}

void RS485::process_packet(uint8_t *packet, size_t length) {
  if (!verify_checksum(packet, length)) {
    ESP_LOGD(TAG, "Ignoring packet with checksum error.");
    return;
  }
  // std::string res = packet_to_hexstring(packet, length);
  // ESP_LOGD(TAG, "DEBUG %d  Packet: %s", debugcnt, res.c_str());

  uint16_t command = ((uint16_t) packet[0] << 8) | packet[1];

  switch (command) {
    case SECONDARY_PRESENCE:
      decode_secondary_presence((RESP_PACKET_T *) packet);
      break;
    case SECONDARY_HEARTBEAT:
      decode_secondary_heartbeat((S_HEARTBEAT_T *) packet);
      break;
    case RESP_PWR_STATUS:
      decode_power_state((EXTENDED_RESP_PACKET_T *) packet);
      break;
    case RESP_SERIAL_NUMBER:
      decode_serial((EXTENDED_RESP_PACKET_T *) packet);
      break;
    case RESP_VIN_FIRST:
    case RESP_VIN_MIDDLE:
    case RESP_VIN_LAST:
      decode_vin((EXTENDED_RESP_PACKET_T *) packet);
      break;

    default:
      std::string res = packet_to_hexstring(packet, length);
      ESP_LOGE(TAG, "Ignoring cmd %04x: %s", command, res.c_str());
      break;
  }
}

std::string RS485::packet_to_hexstring(const uint8_t *packet, size_t length) {
  std::ostringstream oss;
  for (size_t i = 0; i < length; ++i) {
    // Append each byte as a 2-character hexadecimal value
    oss << std::hex << static_cast<int>(packet[i]);
    if (i < length - 1) {
      // Add a space between hex values
      oss << " ";
    }
  }
  // Return the constructed hex string
  return oss.str();
}

void RS485::log_connector_state(const ConnectorState *state) {
  ESP_LOGD(TAG, "Connector Number: %d", state->connector_nr);
  ESP_LOGD(TAG, "Serial Number: %s", (char *) (state->serial_number));
  ESP_LOGD(TAG, "TWC ID: %04x", state->twcid);
  ESP_LOGD(TAG, " Flag: %d", state->power_state_flag);
  ESP_LOGD(TAG, "VIN: %s", (char *) state->vin);
  ESP_LOGD(TAG, "State: %d", state->state);
  ESP_LOGD(TAG, "Actual Current: %d", state->actual_current);
  ESP_LOGD(TAG, "Max Current: %d", state->max_current);
  ESP_LOGD(TAG, "Max Current Last Send: %d", state->max_current_last_send);

  // Log sensor values
  ESP_LOGD(TAG, "Total kWh: %u", state->total_kwh);
  ESP_LOGD(TAG, "Phase 1 Voltage: %d", state->phase1_voltage);
  ESP_LOGD(TAG, "Phase 2 Voltage: %d", state->phase2_voltage);
  ESP_LOGD(TAG, "Phase 3 Voltage: %d", state->phase3_voltage);
  ESP_LOGD(TAG, "Phase 1 Current: %d", state->phase1_current);
  ESP_LOGD(TAG, "Phase 2 Current: %d", state->phase2_current);
  ESP_LOGD(TAG, "Phase 3 Current: %d", state->phase3_current);
}

void ConnectorState::log_values() {
  // Print the values in one line
  ESP_LOGD(TAG,
           "nr=%u, "
           "twcid=%04x, "
           "max_current=%u, "
           "max_current_last_send=%u, "
           "twc_max_current=%u, "
           "command_nr=%u, "
           "state=%u, "
           "actual_current=%u, "
           "serial_number=%s, "
           "vin_incoming=%s, "
           "vin=%s, "
           "power_state_flag=%u, "
           "kwh=%u, "
           "V/I=%u,%u,%u/%u,%u,%u ",
           connector_nr, twcid, max_current, max_current_last_send, twc_max_current, command_nr, state, actual_current,
           serial_number, vin_incoming, vin, power_state_flag, total_kwh, phase1_voltage, phase2_voltage,
           phase3_voltage, phase1_current, phase2_current, phase3_current);
}

}  // namespace twc_gen2
}  // namespace esphome
