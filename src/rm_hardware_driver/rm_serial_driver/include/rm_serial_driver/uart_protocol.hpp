#ifndef RM_SERIAL_DRIVER_UART_PROTOCOL_HPP_
#define RM_SERIAL_DRIVER_UART_PROTOCOL_HPP_

#include "rm_interfaces/msg/serial_send_data.hpp"
#include "rm_interfaces/msg/serial_receive_data.hpp"
#include <string>
#include <cstdint>

namespace pka {

class UARTProtocol
{
public:
  static constexpr uint8_t FRAME_HEADER    = 0xFF;
  static constexpr uint8_t FRAME_TAIL      = 0x0D;
  static constexpr size_t  PACKET_SIZE     = 32;  // 发送帧大小（字节）
  static constexpr size_t  RECV_PACKET_SIZE = 32; // 接收帧大小（字节）

  // ── 发送帧（上位机 → 下位机，32字节） ─────────────────────────────────────
  //   [0]      帧头   0xFF
  //   [1]      fire_advice  (uint8)
  //   [2-5]    pitch        (float32 LE)
  //   [6-9]    yaw          (float32 LE)
  //   [10-13]  distance     (float32 LE)
  //   [14-17]  linear_x     (float32 LE)
  //   [18-21]  linear_y     (float32 LE)
  //   [22-25]  angular_z    (float32 LE)
  //   [26-29]  reserved     0x00×4
  //   [30]     校验位       0x00
  //   [31]     帧尾   0x0D

  // ── 接收帧（下位机 → 上位机，32字节） ─────────────────────────────────────
  //   [0]      帧头   0xFF
  //   [1]      mode         (uint8)
  //   [2-5]    roll         (float32 LE)
  //   [6-9]    pitch        (float32 LE)
  //   [10-13]  yaw          (float32 LE)
  //   [14-17]  chassis_imu_yaw_offset  (float32 LE)
  //   [18-29]  reserved     0x00×12
  //   [30]     校验位       0x00
  //   [31]     帧尾   0x0D

  // 打包：返回 std::string（与 serial::Serial::write(const std::string&) 直接匹配）
  static std::string pack_serial_send_data(
    const rm_interfaces::msg::SerialSendData & data);

  // 解包：输入 std::string（与 serial::Serial::read(size_t) 返回值直接匹配）
  static bool unpack_serial_data(
    const std::string & raw,
    rm_interfaces::msg::SerialReceiveData & msg);

  // 调试：将 std::string 中的字节格式化为 "FF 0D 01 ..." 形式
  static std::string format_hex(const std::string & raw);

private:
  static void  float_to_bytes(float v, char * dst);
  static float bytes_to_float(const char * src);
};

}  // namespace pka

#endif  // RM_SERIAL_DRIVER_UART_PROTOCOL_HPP_