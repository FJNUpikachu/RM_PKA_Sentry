#ifndef RM_SERIAL_DRIVER_UART_NODE_TOOL_HPP_
#define RM_SERIAL_DRIVER_UART_NODE_TOOL_HPP_

#include "rm_serial_driver/uart_node.hpp"
#include "rm_serial_driver/error_codes.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include <geometry_msgs/msg/twist.hpp>

#include <atomic>
#include <thread>
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

  // ── 串口发送辅助（内部按 robot_type_ 分派） ──────────────────────────────
  static void do_send_sentry(
    UARTNode * node,
    const rm_interfaces::msg::SentrySerialSendData & data);

  static void do_send_infantry(
    UARTNode * node,
    const rm_interfaces::msg::InfantrySerialSendData & data);

  // ── 裁判系统消息发布辅助（仅哨兵） ───────────────────────────────────────
  static void publish_judge_msgs(
    UARTNode * node,
    const rm_interfaces::msg::SentrySerialReceiveData & recv_msg);

  // ── SetMode 服务通知辅助 ──────────────────────────────────────────────────
  // 当串口包中的 mode 与上次不同时调用，异步发起服务请求，不阻塞接收线程
  // mode 含义与 SetMode.srv / VisionMode 枚举一致：
  //   0 = AUTO_AIM_RED   1 = AUTO_AIM_BLUE
  static void notify_set_mode(UARTNode * node, uint8_t mode);

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