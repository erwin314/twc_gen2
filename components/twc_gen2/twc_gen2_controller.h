#pragma once

#include "esphome/core/log.h"
#include "esphome/core/component.h"
#include "esphome/core/entity_base.h"
#include "esphome/components/api/custom_api_device.h"
#include "esphome/components/uart/uart.h"
#include "rs485.h"

#include "esphome/components/number/number.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

namespace esphome {
namespace twc_gen2 {

static const char *TAG = "twc_gen2";

class TwcGen2Controller;

/* Allow Charging switch
 */
class AllowCharging : public switch_ ::Switch, public Component {
 public:
  void setup() override { publish_state(initial_value_); };
  void dump_config() override { LOG_SWITCH("", "Allow Charging switch:", this); };

  void loop() override{};

  float get_setup_priority() const override { return setup_priority::DATA; };

  void set_initial_value(float initial_value) { initial_value_ = initial_value; };
  void set_idx(u_int8_t idx) { idx_ = idx; };
  void set_controller(TwcGen2Controller *controller) { controller_ = controller; };

 protected:
  void write_state(bool state) override;

  bool initial_value_;
  u_int8_t idx_;
  TwcGen2Controller *controller_;
};

/* Current Limit number
 */
class CurrentLimit : public number::Number, public Component {
 public:
  void setup() override;
  void dump_config() override { LOG_NUMBER("", "Current Limit number:", this); };
  float get_setup_priority() const override { return setup_priority::DATA; };
  void set_controller(TwcGen2Controller *controller) { controller_ = controller; };

 protected:
  void control(float value) override;

  TwcGen2Controller *controller_;
};

/* TWC generation 2 Controller.
 */
class TwcGen2Controller : public uart::UARTDevice, public Component {
 public:
  TwcGen2Controller();

  void setup() override;
  void dump_config() override;

  float get_setup_priority() const override { return setup_priority::DATA; };
  void loop() override { this->rs485_->read_incoming_data(); };
  void set_flow_control_pin(GPIOPin *flow_control_pin) { this->flow_control_pin_ = flow_control_pin; };
  void set_read_only(bool v) { read_only_ = v; };
  u_int8_t serial_to_nr(uint8_t serial[11]);
  void loadbalance_max_current();

  void add_allow_charging_switch(u_int8_t idx, switch_::Switch *s) { allow_charging_switch[idx] = s; };
  void set_global_limit_number(number::Number *n) { this->global_limit_number = n; }

  void set_connector_serial(u_int8_t nr, const char *serial) {
    memcpy(&(serial_list[nr]), serial, SERIAL_NUMBER_LEN);
    serial_list[nr][SERIAL_NUMBER_LEN] = 0;
  }

  ConnectorState *get_connector_by_nr(uint8_t nr) { return rs485_->get_connector_by_nr(nr); };
  text_sensor::TextSensor *serial_text_sensor[3] = {nullptr};
  text_sensor::TextSensor *vin_text_sensor[3] = {nullptr};
  sensor::Sensor *voltage_sensor[3][3] = {nullptr};
  sensor::Sensor *current_sensor[3][3] = {nullptr};
  sensor::Sensor *energy_sensor[3] = {nullptr};
  sensor::Sensor *state_sensor[3] = {nullptr};
  sensor::Sensor *actual_current_sensor[3] = {nullptr};

  u_int8_t global_limit = 6;
  u_int8_t twc_limit[3] = {6, 6, 6};

 protected:
  GPIOPin *flow_control_pin_{nullptr};
  RS485 *rs485_{nullptr};

  void on_connector_changed(ConnectorState *c);

  bool read_only_ = false;
  uint16_t twcid_list[3] = {0};
  u_int8_t serial_list[3][SERIAL_NUMBER_LEN + 1] = {0};

  switch_::Switch *allow_charging_switch[3];
  number::Number *global_limit_number;
};

}  // namespace twc_gen2
}  // namespace esphome
