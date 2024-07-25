#include "slip_decoder.h"
#include "esphome/core/log.h"

namespace esphome {
namespace twc_gen2 {

// static const char *const TAG = "twc_gen2";

bool SlipDecoder::add_byte(uint8_t b) {
  // Was the previous decoded byte the end of a completed datagram?
  if (datagram_complete == true) {
    // The incoming byte is part of the next datagram, so reset.
    reset();
  }

  // Is the datagram to big for buffer?
  if (datagram_overflow) {
    if (b == SLIP_END) {
      // Next datagram is starting.
      reset();
    }
    return false;
  }

  // Datagram complete but to small?
  if ((b == SLIP_END) && (datagram_length < MIN_PACKET_LENGTH)) {
    reset();
    return false;
  }

  // Datagram complete?
  if (b == SLIP_END) {
    datagram_complete = true;
    return true;
  }

  // Escape?
  if (b == SLIP_ESC) {
    inside_slip_esc = true;
    return false;
  }

  // To many bytes?
  if (datagram_length >= MAX_PACKET_LENGTH) {
    datagram_overflow = true;
    return false;
  }

  // Need to translate byte because of SLIP escape?
  if (inside_slip_esc) {
    inside_slip_esc = false;
    b = decode_slip_esc(b);
  }

  // ESP_LOGD(TAG, "D[%d]=%02x", datagram_length, b);

  // Add byte to the datagram
  datagram[datagram_length++] = b;

  // We are expecting more byte(s)
  return false;
}

void SlipDecoder::reset() {
  datagram_complete = false;
  datagram_overflow = false;
  inside_slip_esc = false;
  datagram_length = 0;
}

uint8_t SlipDecoder::decode_slip_esc(uint8_t b) {
  switch (b) {
    case SLIP_ESC_END:
      return SLIP_END;
    case SLIP_ESC_ESC:
      return SLIP_ESC;
    default:
      return b;
  }
}

}  // namespace twc_gen2
}  // namespace esphome
