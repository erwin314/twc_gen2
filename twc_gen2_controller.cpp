#include "twc_gen2_controller.h"
#include <sstream>

namespace esphome {
namespace twc_gen2 {

TwcGen2Controller::TwcGen2Controller() {}

void TwcGen2Controller::setup() {
  ESP_LOGI(TAG, "Starting");

  // Setup flow control pin
  if (this->flow_control_pin_ != nullptr) {
    this->flow_control_pin_->setup();
  }

  // Configure modbus connection to the Tesla Wall Connectors.
  this->rs485_ = new RS485(this->parent_, this->flow_control_pin_);

  // We want to be notified for every change in any of the connectors
  rs485_->set_callback([this](ConnectorState *s) { this->on_connector_changed(s); });

  // Start background thread (unless in read-only mode)
  if (!read_only_)
    rs485_->setup();
}

void TwcGen2Controller::dump_config() {
  ESP_LOGCONFIG(TAG, "TwcGen2 Controller");
  ESP_LOGCONFIG(TAG, "  Read Only: %d", this->read_only_);
  LOG_PIN("  Flow Control Pin: ", this->flow_control_pin_);
};

u_int8_t TwcGen2Controller::serial_to_nr(uint8_t serial[11]) {
  // No serial specified?
  if (serial[0] == 0) {
    return 0;
  }
  // Check if it the serial belongs to connector 1, 2 or 3.
  if (memcmp(serial, serial_list[0], 11) == 0)
    return 1;
  if (memcmp(serial, serial_list[1], 11) == 0)
    return 2;
  if (memcmp(serial, serial_list[2], 11) == 0)
    return 3;

  // There is no connector with this serial.
  return -1;
}

#define PUBLISH_SENSOR(name, value) \
  if (name) { \
    if ((!name->has_state()) || (name->state != value)) { \
      name->publish_state(value); \
    } \
  }

void TwcGen2Controller::loadbalance_max_current() {
  // Count number of TWC's that are charging.
  uint8_t active_count = 0;
  for (uint8_t n = 1; n <= 3; n++) {
    auto con = this->rs485_->get_connector_by_nr(n);
    if (con && (con->actual_current > 0)) {
      active_count++;
    }
  }

  // Perform load balancing if any twc's are charging.
  uint8_t global_limit_divided_over_active_twcs = 6;
  if (active_count > 0) {
    global_limit_divided_over_active_twcs = this->global_limit / active_count;
  }

  for (uint8_t n = 1; n <= 3; n++) {
    auto con = this->rs485_->get_connector_by_nr(n);
    if (con && (con->actual_current > 0)) {
      con->max_current = min(this->twc_limit[n - 1], global_limit_divided_over_active_twcs);
    } else if (con) {
      con->max_current = min(this->twc_limit[n - 1], this->global_limit);
    }
  }
}

void TwcGen2Controller::on_connector_changed(ConnectorState *c) {
  ESP_LOGI(TAG, "Connector %d changed.", c->connector_nr);
  // Unknown connector nr?
  if (c->connector_nr == 0) {
    char *s = (char *) &(c->serial_number[0]);

    // User the serial number to find out to which connector number this state belongs
    auto nr = serial_to_nr(c->serial_number);

    // When first starting up, it could be that the serial-number has not been queried yet.
    if (nr == 0) {
      // skip this round, and wait for the serial-number to be seen.
      ESP_LOGI(TAG, "Still waiting for serial number.");
      return;
    }

    // Unknown serial?
    if (nr == -1) {
      ESP_LOGE(TAG, "Unknown serial number: %s", s);
      return;
    }
    c->connector_nr = nr;
    ESP_LOGI(TAG, "Initialized connector%d (serial: %s).", nr, s);

    // Set the limit of this TWC, initialize to the smallest limit.
    c->max_current = min(this->twc_limit[nr - 1], this->global_limit);
  }

  // rs485_->log_connector_state(c);

  loadbalance_max_current();

  u_int8_t idx = c->connector_nr - 1;
  PUBLISH_SENSOR(serial_text_sensor[idx], (char *) c->serial_number);
  PUBLISH_SENSOR(vin_text_sensor[idx], (char *) c->vin);
  PUBLISH_SENSOR(voltage_sensor[idx][0], c->phase1_voltage);
  PUBLISH_SENSOR(voltage_sensor[idx][1], c->phase2_voltage);
  PUBLISH_SENSOR(voltage_sensor[idx][2], c->phase3_voltage);
  PUBLISH_SENSOR(current_sensor[idx][0], c->phase1_current);
  PUBLISH_SENSOR(current_sensor[idx][1], c->phase2_current);
  PUBLISH_SENSOR(current_sensor[idx][2], c->phase3_current);
  PUBLISH_SENSOR(energy_sensor[idx], c->total_kwh);
  PUBLISH_SENSOR(actual_current_sensor[idx], c->actual_current);
  PUBLISH_SENSOR(state_sensor[idx], c->state);
}

void AllowCharging::write_state(bool state) {
  ESP_LOGD(TAG, "IDX=%d", idx_);
  auto c = controller_->get_connector_by_nr(idx_ + 1);
  if (!c) {
    return;
  }
  c->allow_charging = (uint8_t) state;
  this->publish_state(state);
};

void CurrentLimit::setup() { this->publish_state(this->controller_->global_limit); };

void CurrentLimit::control(float value) {
  this->controller_->global_limit = value;
  this->publish_state(value);
};

}  // namespace twc_gen2
}  // namespace esphome
