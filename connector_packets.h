#pragma once

#include <cstdint>

namespace esphome {
namespace twc_gen2 {

constexpr uint16_t CONTROLLER_TWCID = 0x1337;
constexpr uint16_t SERIAL_NUMBER_LEN = 11;
constexpr uint16_t VIN_LEN = 7 + 7 + 3;

constexpr uint16_t GET_SERIAL_NUMBER_OLD = 0xFB19;
constexpr uint16_t GET_MODEL_NUMBER = 0xFB1A;
constexpr uint16_t GET_FIRMWARE_VER = 0xFB1B;
constexpr uint16_t GET_PLUG_STATE = 0xFBB4;

constexpr uint16_t PRIMARY_HEARTBEAT = 0xFBE0;
constexpr uint16_t PRIMARY_PRESENCE2 = 0xFBE2;

constexpr uint16_t GET_PWR_STATE = 0xFBEB;
constexpr uint16_t GET_FIRMWARE_VER_EXT = 0xFBEC;
constexpr uint16_t GET_SERIAL_NUMBER = 0xFBED;
constexpr uint16_t GET_VIN_FIRST = 0xFBEE;
constexpr uint16_t GET_VIN_MIDDLE = 0xFBEF;
constexpr uint16_t GET_VIN_LAST = 0xFBF1;

// The next two commands are ** DANGEROUS ** !
// DO NOT USE THESE! They are defined so that
// they can be blocked.
constexpr uint16_t WRITE_ID_DATE = 0xFC19;
constexpr uint16_t WRITE_MODEL_NO = 0xFC1A;

// Commands without responses (0xFC)
constexpr uint16_t IDLE_MESSAGE = 0xFC1D;
constexpr uint16_t PRIMARY_PRESENCE = 0xFCE1;

// Start / Stop
constexpr uint16_t START_CHARGING = 0xFCB1;
constexpr uint16_t STOP_CHARGING = 0xFCB2;
constexpr uint16_t RESP_START_CHARGING = 0xFDB1;
constexpr uint16_t RESP_STOP_CHARGING = 0xFDB2;

// Responses (0xFD)
constexpr uint16_t RESP_SERIAL_NUMBER_OLD = 0xFD19;
constexpr uint16_t RESP_MODEL_NUMBER = 0xFD1A;
constexpr uint16_t RESP_FIRMWARE_VER = 0xFD1B;
constexpr uint16_t RESP_PLUG_STATE = 0xFDB4;

constexpr uint16_t SECONDARY_HEARTBEAT = 0xFDE0;

constexpr uint16_t SECONDARY_PRESENCE = 0xFDE2;  // Sent by secondary on reset
constexpr uint16_t RESP_PWR_STATUS = 0xFDEB;     // Sent by primary on reset
constexpr uint16_t RESP_FIRMWARE_VER_EXT = 0xFDEC;
constexpr uint16_t RESP_SERIAL_NUMBER = 0xFDED;
constexpr uint16_t RESP_VIN_FIRST = 0xFDEE;
constexpr uint16_t RESP_VIN_MIDDLE = 0xFDEF;
constexpr uint16_t RESP_VIN_LAST = 0xFDF1;

struct PACKET_T {
  uint16_t command;
  uint16_t twcid;
  uint16_t secondary_twcid;
  uint8_t payload[6];
  uint8_t checksum;
} PACKED;

struct RESP_PACKET_T {
  uint16_t command;
  uint16_t twcid;
  uint8_t payload[11];
  uint8_t checksum;
} PACKED;

struct EXTENDED_RESP_PACKET_T {
  uint16_t command;
  uint16_t twcid;
  uint8_t payload[15];
  uint8_t checksum;
} PACKED;

struct S_HEARTBEAT_T {
  uint16_t command;
  uint16_t src_twcid;
  uint16_t dst_twcid;
  uint8_t state;
  uint16_t max_current;
  uint16_t actual_current;
  uint8_t padding[4];
  uint8_t checksum;
} PACKED;

struct P_HEARTBEAT_T {
  uint16_t command;
  uint16_t src_twcid;
  uint16_t dst_twcid;
  uint8_t state;
  uint16_t max_current;
  uint8_t plug_inserted;
  uint8_t padding[5];
  uint8_t checksum;
} PACKED;

// Standard response packet payload
struct PRESENCE_PAYLOAD_T {
  uint8_t sign;
  uint16_t max_allowable_current;
  uint8_t padding[8];
} PACKED;

// Extended response packet payload
struct POWERSTATUS_PAYLOAD_T {
  uint32_t total_kwh;
  uint16_t phase1_voltage;
  uint16_t phase2_voltage;
  uint16_t phase3_voltage;
  uint8_t phase1_current;
  uint8_t phase2_current;
  uint8_t phase3_current;
  uint8_t padding[2];
} PACKED;

// Extended response packet payload
struct VIN_PAYLOAD_T {
  uint8_t vin[7];
  uint8_t padding[4];
} PACKED;

// Extended response packet payload
struct SERIAL_PAYLOAD_T {
  uint8_t serial[SERIAL_NUMBER_LEN];
  uint8_t padding[4];
} PACKED;

// Standard response packet Payload
struct EXT_FIRMWARE_PAYLOAD_T {
  uint8_t major;
  uint8_t minor;
  uint8_t revision;
  uint8_t extended;
  uint8_t padding[7];
} PACKED;

}  // namespace twc_gen2
}  // namespace esphome
