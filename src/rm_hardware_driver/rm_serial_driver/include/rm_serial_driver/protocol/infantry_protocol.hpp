#ifndef RM_SERIAL_DRIVER_PROTOCOL_INFANTRY_PROTOCOL_HPP_
#define RM_SERIAL_DRIVER_PROTOCOL_INFANTRY_PROTOCOL_HPP_

#include "rm_serial_driver/protocol/uart_protocol_base.hpp"

// infantry 专用消息
#include "rm_interfaces/msg/infantry_serial_send_data.hpp"
#include "rm_interfaces/msg/infantry_serial_receive_data.hpp"

namespace pka {

// ── 步兵（Infantry）串口协议 ──────────────────────────────────────────────────
//
// 发送帧（16字节）：
//   [0]      帧头   0xFF
//   [1]      fire_advice  (uint8)
//   [2-5]    pitch        (float32 LE)
//   [6-9]    yaw          (float32 LE)
//   [10-13]  distance     (float32 LE)
//   [14]     校验位       0x00（预留）
//   [15]     帧尾   0x0D
//
// 接收帧（16字节）：
//   [0]      帧头   0xFF
//   [1]      mode         (uint8)
//   [2-5]    roll         (float32 LE)
//   [6-9]    pitch        (float32 LE)
//   [10-13]  yaw          (float32 LE)
//   [14]     校验位       0x00（预留）
//   [15]     帧尾   0x0D
//
class InfantryProtocol : public IUARTProtocol
{
public:
  static constexpr size_t SEND_PACKET_SIZE = 16;
  static constexpr size_t RECV_PACKET_SIZE = 16;

  size_t send_packet_size() const noexcept override { return SEND_PACKET_SIZE; }
  size_t recv_packet_size() const noexcept override { return RECV_PACKET_SIZE; }

  // 打包：返回 std::string，直接传入 serial_->write()
  static std::string pack(const rm_interfaces::msg::InfantrySerialSendData & data);

  // 解包：输入 std::string（serial_->read() 返回值），成功返回 true
  static bool unpack(
    const std::string & raw,
    rm_interfaces::msg::InfantrySerialReceiveData & msg);
};

}  // namespace pka

#endif  // RM_SERIAL_DRIVER_PROTOCOL_INFANTRY_PROTOCOL_HPP_