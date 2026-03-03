#include "rm_serial_driver/uart_protocol.hpp"

#include <sstream>
#include <iomanip>
#include <cstring>

namespace pka {

// ── 私有：float ↔ 4字节 LE ───────────────────────────────────────────────────

void UARTProtocol::float_to_bytes(float v, char * dst)
{
  // 与 example 中 serial_.write(string) 的内存语义一致：直接 memcpy
  std::memcpy(dst, &v, 4);
}

float UARTProtocol::bytes_to_float(const char * src)
{
  float v;
  std::memcpy(&v, src, 4);
  return v;
}

// ── 打包发送帧 ────────────────────────────────────────────────────────────────
//
// 返回 std::string（32字节），调用方直接：
//   size_t bytes_wrote = serial_->write(packed);   ← 与 example 完全一致
//
std::string UARTProtocol::pack_serial_send_data(
  const rm_interfaces::msg::SerialSendData & data)
{
  std::string buf(PACKET_SIZE, '\0');

  buf[0] = static_cast<char>(FRAME_HEADER);
  buf[1] = data.fire_advice ? '\x01' : '\x00';
  float_to_bytes(data.pitch,     &buf[2]);
  float_to_bytes(data.yaw,       &buf[6]);
  float_to_bytes(data.distance,  &buf[10]);
  float_to_bytes(data.linear_x,  &buf[14]);
  float_to_bytes(data.linear_y,  &buf[18]);
  float_to_bytes(data.angular_z, &buf[22]);
  // [26-29] reserved: already '\0'
  buf[30] = '\x00';   // 校验位（预留）
  buf[31] = static_cast<char>(FRAME_TAIL);

  return buf;
}

// ── 解包接收帧 ────────────────────────────────────────────────────────────────
//
// 输入 raw = serial_->read(RECV_PACKET_SIZE)   ← 与 example 完全一致
//
bool UARTProtocol::unpack_serial_data(
  const std::string & raw,
  rm_interfaces::msg::SerialReceiveData & msg)
{
  if (raw.size() != RECV_PACKET_SIZE) { return false; }

  if (static_cast<uint8_t>(raw[0])  != FRAME_HEADER) { return false; }
  if (static_cast<uint8_t>(raw[31]) != FRAME_TAIL)   { return false; }

  msg.mode                   = static_cast<uint8_t>(raw[1]);
  msg.roll                   = bytes_to_float(&raw[2]);
  msg.pitch                  = bytes_to_float(&raw[6]);
  msg.yaw                    = bytes_to_float(&raw[10]);
  msg.chassis_imu_yaw_offset = bytes_to_float(&raw[14]);

  return true;
}

// ── 调试：hex dump ────────────────────────────────────────────────────────────

std::string UARTProtocol::format_hex(const std::string & raw)
{
  std::ostringstream ss;
  for (size_t i = 0; i < raw.size(); ++i) {
    if (i) { ss << ' '; }
    ss << std::uppercase << std::setw(2) << std::setfill('0')
       << std::hex << (static_cast<unsigned int>(
                         static_cast<unsigned char>(raw[i])) & 0xFF);
  }
  return ss.str();
}

}  // namespace pka