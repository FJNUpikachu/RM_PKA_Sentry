#ifndef RM_SERIAL_DRIVER_UART_NODE_TOOL_HPP_
#define RM_SERIAL_DRIVER_UART_NODE_TOOL_HPP_

#include "rm_serial_driver/uart_node.hpp"
#include "rm_serial_driver/error_codes.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include <geometry_msgs/msg/twist.hpp>

#include <string>

namespace pka {
namespace serial_driver {

class UARTNodeTool
{
public:
  // ── 话题回调（仅缓存） ────────────────────────────────────────────────────
  static void gimbal_cmd_callback(
    UARTNode * node,
    const rm_interfaces::msg::GimbalCmd::SharedPtr msg);

  static void cmd_vel_callback(
    UARTNode * node,
    const geometry_msgs::msg::Twist::SharedPtr msg);

  // ── 定时器回调 ────────────────────────────────────────────────────────────
  static void mock_recv_timer_cb(UARTNode * node);         // mode=0
  static void send_timer_cb(UARTNode * node);              // mode=1
  static void recv_timer_cb(UARTNode * node);              // mode=1,2
  static void virtual_send_timer_cb(UARTNode * node);      // mode=2
  static void health_timer_cb(UARTNode * node);            // mode=1,2

  // ── 串口发送辅助 ──────────────────────────────────────────────────────────
  // 将 rm_interfaces::msg::SerialSendData 打包写入串口
  // 按 example 方式：size_t bytes_wrote = serial_->write(packed_string)
  static void do_send(UARTNode * node,
                      const rm_interfaces::msg::SerialSendData & data);

  // ── 串口重启 ──────────────────────────────────────────────────────────────
  static bool restart_serial(UARTNode * node);

  // ── 错误处理 ──────────────────────────────────────────────────────────────
  static void on_error(UARTNode * node,
                       SerialErrorCode code,
                       const std::string & msg);
};

}  // namespace serial_driver
}  // namespace pka

#endif  // RM_SERIAL_DRIVER_UART_NODE_TOOL_HPP_