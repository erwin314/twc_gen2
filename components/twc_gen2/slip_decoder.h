#pragma once

#include <functional>
#include <map>
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/components/uart/uart_component.h"

namespace esphome {
namespace twc_gen2 {

// SLIP magic id's
constexpr uint8_t SLIP_END = 0xC0;
constexpr uint8_t SLIP_ESC = 0xDB;
constexpr uint8_t SLIP_ESC_END = 0xDC;
constexpr uint8_t SLIP_ESC_ESC = 0xDD;

constexpr uint8_t MIN_PACKET_LENGTH = 13;
constexpr uint8_t MAX_PACKET_LENGTH = 32;

class SlipDecoder {
 public:
  bool add_byte(uint8_t b);

  uint8_t datagram[MAX_PACKET_LENGTH];
  uint8_t datagram_length = 0;
  bool datagram_complete = false;
  bool datagram_overflow = false;

 private:
  void reset();
  uint8_t decode_slip_esc(uint8_t b);
  bool inside_slip_esc = false;
};

}  // namespace twc_gen2
}  // namespace esphome
