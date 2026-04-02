#include "rm_serial_driver/protocol/infantry_protocol.hpp"
#include <cstring>

namespace pka {

// ── 发送帧（16字节）打包 ──────────────────────────────────────────────────────
//
//   [0]      帧头   0xFF
//   [1]      fire_advice  (uint8)
//   [2-5]    pitch        (float32 LE)
//   [6-9]    yaw          (float32 LE)
//   [10-13]  distance     (float32 LE)
//   [14]     校验位       0x00（预留）
//   [15]     帧尾   0x0D
//
std::string InfantryProtocol::pack(const rm_interfaces::msg::InfantrySerialSendData & data)
{
  std::string buf(SEND_PACKET_SIZE, '\0');

  buf[0] = static_cast<char>(PROTOCOL_FRAME_HEADER);
  buf[1] = data.fire_advice ? '\x01' : '\x00';
  float_to_bytes(data.pitch,    &buf[2]);
  float_to_bytes(data.yaw,      &buf[6]);
  float_to_bytes(data.distance, &buf[10]);
  buf[14] = '\x00';   // 校验位（预留）
  buf[15] = static_cast<char>(PROTOCOL_FRAME_TAIL);

  return buf;
}

// ── 接收帧（16字节）解包 ──────────────────────────────────────────────────────
//
//   [0]      帧头   0xFF
//   [1]      mode         (uint8)
//   [2-5]    roll         (float32 LE)
//   [6-9]    pitch        (float32 LE)
//   [10-13]  yaw          (float32 LE)
//   [14]     校验位       0x00（预留）
//   [15]     帧尾   0x0D
//
bool InfantryProtocol::unpack(
  const std::string & raw,
  rm_interfaces::msg::InfantrySerialReceiveData & msg)
{
  if (raw.size() != RECV_PACKET_SIZE)                          { return false; }
  if (static_cast<uint8_t>(raw[0])  != PROTOCOL_FRAME_HEADER) { return false; }
  if (static_cast<uint8_t>(raw[15]) != PROTOCOL_FRAME_TAIL)   { return false; }

  msg.mode  = static_cast<uint8_t>(raw[1]);
  msg.roll  = bytes_to_float(&raw[2]);
  msg.pitch = bytes_to_float(&raw[6]);
  msg.yaw   = bytes_to_float(&raw[10]);
  // [14] 校验位（暂不校验）

  return true;
}

}  // namespace pka