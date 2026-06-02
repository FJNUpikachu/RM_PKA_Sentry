#ifndef RM_SERIAL_DRIVER_PROTOCOL_UART_PROTOCOL_BASE_HPP_
#define RM_SERIAL_DRIVER_PROTOCOL_UART_PROTOCOL_BASE_HPP_

#include <string>
#include <cstdint>
#include <cstring>
#include <sstream>
#include <iomanip>

namespace pka {

// ── 公用帧标识 ────────────────────────────────────────────────────────────────
static constexpr uint8_t PROTOCOL_FRAME_HEADER = 0xFF;
static constexpr uint8_t PROTOCOL_FRAME_TAIL   = 0x0D;

// ── 抽象基类：所有机器人串口协议均派生自此类 ─────────────────────────────────
//
// 设计原则：
//   • 仅约束"发包/收包"接口，具体字段由子类决定
//   • 使用 std::string 作为字节容器，与 serial::Serial::read/write 无缝对接
//   • 不持有状态，所有方法均为 const（虽然 C++ 纯虚接口本身无实例）
//
class IUARTProtocol
{
public:
  virtual ~IUARTProtocol() = default;

  // 返回发送帧的字节长度（固定包长协议）
  virtual size_t send_packet_size()  const noexcept = 0;

  // 返回接收帧的字节长度（固定包长协议）
  virtual size_t recv_packet_size()  const noexcept = 0;

  // 帧头字节（子类可重写以使用不同帧头）
  virtual uint8_t frame_header() const noexcept { return PROTOCOL_FRAME_HEADER; }

  // 帧尾字节
  virtual uint8_t frame_tail()   const noexcept { return PROTOCOL_FRAME_TAIL;   }

  // 调试：将原始字节序列格式化为 "FF 0D 01 ..." 形式
  static std::string format_hex(const std::string & raw)
  {
    std::ostringstream ss;
    for (size_t i = 0; i < raw.size(); ++i) {
      if (i) { ss << ' '; }
      ss << std::uppercase << std::setw(2) << std::setfill('0')
         << std::hex
         << (static_cast<unsigned int>(static_cast<unsigned char>(raw[i])) & 0xFF);
    }
    return ss.str();
  }

protected:
  // ── 内联工具函数（供子类使用，无需重复定义）──────────────────────────────
  static void float_to_bytes(float v, char * dst)
  {
    std::memcpy(dst, &v, 4);
  }

  static float bytes_to_float(const char * src)
  {
    float v;
    std::memcpy(&v, src, 4);
    return v;
  }
};

}  // namespace pka

#endif  // RM_SERIAL_DRIVER_PROTOCOL_UART_PROTOCOL_BASE_HPP_