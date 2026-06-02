#include "rm_serial_driver/protocol/sentry_protocol.hpp"
#include <cstring>

namespace pka {

std::string SentryProtocol::pack(const rm_interfaces::msg::SentrySerialSendData & data)
{
  std::string buf(SEND_PACKET_SIZE, '\0');

  buf[0] = static_cast<char>(PROTOCOL_FRAME_HEADER);
  buf[1] = static_cast<char>(data.fire_advice);
  float_to_bytes(data.pitch,     &buf[2]);
  float_to_bytes(data.yaw,       &buf[6]);
  float_to_bytes(data.distance,  &buf[10]);
  float_to_bytes(data.linear_x,  &buf[14]);
  float_to_bytes(data.linear_y,  &buf[18]);
  float_to_bytes(data.angular_z, &buf[22]);
  buf[26] = static_cast<char>(data.pose_status);
  // [27-29] reserved: already '\0'
  buf[30] = '\x00';   // 校验位（预留）
  buf[31] = static_cast<char>(PROTOCOL_FRAME_TAIL);

  return buf;
}

bool SentryProtocol::unpack(
  const std::string & raw,
  rm_interfaces::msg::SentrySerialReceiveData & msg)
{
  if (raw.size() != RECV_PACKET_SIZE)                        { return false; }
  if (static_cast<uint8_t>(raw[0])  != PROTOCOL_FRAME_HEADER) { return false; }
  if (static_cast<uint8_t>(raw[31]) != PROTOCOL_FRAME_TAIL)   { return false; }

  msg.mode                   = static_cast<uint8_t>(raw[1]);
  msg.roll                   = bytes_to_float(&raw[2]);
  msg.pitch                  = bytes_to_float(&raw[6]);
  msg.yaw                    = bytes_to_float(&raw[10]);
  msg.chassis_imu_yaw_offset = bytes_to_float(&raw[14]);

  // game_progress [18-19]：uint16 LE
  msg.game_progress = static_cast<uint16_t>(
    (static_cast<uint8_t>(raw[18])) |
    (static_cast<uint8_t>(raw[19]) << 8));

  // current_hp [20-21]：uint16 LE
  msg.current_hp = static_cast<uint16_t>(
    (static_cast<uint8_t>(raw[20])) |
    (static_cast<uint8_t>(raw[21]) << 8));

  // bullet_rest [22-23]：uint16 LE
  msg.bullet_rest = static_cast<uint16_t>(
    (static_cast<uint8_t>(raw[22])) |
    (static_cast<uint8_t>(raw[23]) << 8));

  msg.is_battling = static_cast<uint8_t>(raw[24]);
  msg.is_fortress = static_cast<uint8_t>(raw[25]);
  msg.is_supply   = static_cast<uint8_t>(raw[26]);

  // outpost_hp [27-28]：uint16 LE；[29] 保留 0x00
  msg.outpost_hp = static_cast<uint16_t>(
    (static_cast<uint8_t>(raw[27])) |
    (static_cast<uint8_t>(raw[28]) << 8));

  // [29] reserved, [30] 校验位（暂不校验）
  return true;
}

}  // namespace pka