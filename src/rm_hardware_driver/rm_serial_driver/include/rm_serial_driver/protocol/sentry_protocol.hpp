#ifndef RM_SERIAL_DRIVER_PROTOCOL_SENTRY_PROTOCOL_HPP_
#define RM_SERIAL_DRIVER_PROTOCOL_SENTRY_PROTOCOL_HPP_

#include "rm_serial_driver/protocol/uart_protocol_base.hpp"

// sentry 专用消息（保留原有 msg，命名加 Sentry 前缀）
#include "rm_interfaces/msg/sentry_serial_send_data.hpp"
#include "rm_interfaces/msg/sentry_serial_receive_data.hpp"

namespace pka {

// ── 哨兵（Sentry）串口协议 ────────────────────────────────────────────────────
//
// 发送帧（32字节）：
//   [0]      帧头   0xFF
//   [1]      fire_advice  (uint8)
//   [2-5]    pitch        (float32 LE)
//   [6-9]    yaw          (float32 LE)
//   [10-13]  distance     (float32 LE)
//   [14-17]  linear_x     (float32 LE)
//   [18-21]  linear_y     (float32 LE)
//   [22-25]  angular_z    (float32 LE)
//   [26-29]  reserved     0x00×4
//   [30]     校验位       0x00（预留）
//   [31]     帧尾   0x0D
//
// 接收帧（32字节）：
//   [0]      帧头   0xFF
//   [1]      mode                       (uint8)
//   [2-5]    roll                       (float32 LE)
//   [6-9]    pitch                      (float32 LE)
//   [10-13]  yaw                        (float32 LE)
//   [14-17]  chassis_imu_yaw_offset     (float32 LE)
//   [18-19]  game_progress              (uint16 LE)
//   [20]     friendly_supply_zone_non_exchange  (uint8)
//   [21-22]  current_hp                 (uint16 LE)
//   [23-29]  reserved                   0x00×7
//   [30]     校验位                     0x00
//   [31]     帧尾   0x0D
//
class SentryProtocol : public IUARTProtocol
{
public:
  static constexpr size_t SEND_PACKET_SIZE = 32;
  static constexpr size_t RECV_PACKET_SIZE = 32;

  size_t send_packet_size() const noexcept override { return SEND_PACKET_SIZE; }
  size_t recv_packet_size() const noexcept override { return RECV_PACKET_SIZE; }

  // 打包：返回 std::string，直接传入 serial_->write()
  static std::string pack(const rm_interfaces::msg::SentrySerialSendData & data);

  // 解包：输入 std::string（serial_->read() 返回值），成功返回 true
  static bool unpack(
    const std::string & raw,
    rm_interfaces::msg::SentrySerialReceiveData & msg);
};

}  // namespace pka

#endif  // RM_SERIAL_DRIVER_PROTOCOL_SENTRY_PROTOCOL_HPP_