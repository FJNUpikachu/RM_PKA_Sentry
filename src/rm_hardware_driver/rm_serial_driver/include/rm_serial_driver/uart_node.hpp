#ifndef RM_SERIAL_DRIVER_UART_NODE_HPP_
#define RM_SERIAL_DRIVER_UART_NODE_HPP_

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/twist.hpp>

// project interfaces
#include "rm_utils/heartbeat.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/msg/serial_receive_data.hpp"
#include "rm_interfaces/msg/serial_send_data.hpp"

// 内嵌 wjwwood/serial 库
#include "serial/serial.h"

// project
#include "rm_serial_driver/uart_protocol.hpp"
#include "rm_serial_driver/error_codes.hpp"

namespace pka {
namespace serial_driver {

class UARTNodeTool;

class UARTNode : public rclcpp::Node
{
public:
  explicit UARTNode(const rclcpp::NodeOptions & options);
  ~UARTNode();

private:
  friend class UARTNodeTool;

  void init_parameters();
  void init_serial();       // 打开串口（mode=0 时跳过）；重启时也调用
  void init_subscriber();
  void init_publisher();
  void init_timer();

  // TF 广播，roll/pitch/yaw 单位：度
  void broadcastGimbalTF(const rm_interfaces::msg::SerialReceiveData & data);

  // ── 串口对象（按 example 用法，栈语义通过 unique_ptr 管理） ───────────────
  // 构造：serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(ms))
  std::unique_ptr<serial::Serial> serial_;

  // ── ROS 2 ─────────────────────────────────────────────────────────────────
  rclcpp::Publisher<rm_interfaces::msg::SerialReceiveData>::SharedPtr recv_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  HeartBeatPublisher::SharedPtr heartbeat_pub_;

  // 订阅（话题名为空时不创建，缓存字段保持 0）
  rclcpp::Subscription<rm_interfaces::msg::GimbalCmd>::SharedPtr   gimbal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr       cmd_vel_sub_;

  // ── 缓存（话题未到达保持 0，发送自动填 0） ────────────────────────────────
  bool  cached_fire_advice_ {false};
  float cached_pitch_       {0.0f};
  float cached_yaw_         {0.0f};
  float cached_distance_    {0.0f};
  float cached_linear_x_    {0.0f};
  float cached_linear_y_    {0.0f};
  float cached_angular_z_   {0.0f};

  // ── 定时器 ────────────────────────────────────────────────────────────────
  rclcpp::TimerBase::SharedPtr mock_recv_timer_;      // mode=0
  rclcpp::TimerBase::SharedPtr send_timer_;           // mode=1
  rclcpp::TimerBase::SharedPtr recv_timer_;           // mode=1,2
  rclcpp::TimerBase::SharedPtr health_timer_;         // mode=1,2
  rclcpp::TimerBase::SharedPtr virtual_send_timer_;   // mode=2

  // ── 参数 ──────────────────────────────────────────────────────────────────
  std::string port_name_;
  int         baudrate_;
  double      timestamp_offset_;
  bool        debug_;
  int         serial_mode_;

  double virtual_send_frequency_;
  double send_frequency_;
  double read_frequency_;
  int    max_failure_count_;
  double health_check_interval_;
  int    max_restart_attempts_;
  double restart_cooldown_;
  bool   enable_auto_restart_;
  int    restart_delay_;

  double cmd_vel_linear_scale_;
  std::string gimbal_cmd_topic_;
  std::string cmd_vel_topic_;
  std::string target_frame_;

  // mode=0
  uint8_t mock_mode_;
  float   mock_roll_;
  float   mock_pitch_;
  float   mock_yaw_;
  float   mock_chassis_imu_yaw_offset_;
  double  mock_recv_frequency_;

  // mode=2
  bool  virtual_fire_advice_;
  float virtual_pitch_;
  float virtual_yaw_;
  float virtual_distance_;
  float virtual_linear_x_;
  float virtual_linear_y_;
  float virtual_angular_z_;

  // ── 健康状态 ──────────────────────────────────────────────────────────────
  int  consecutive_failure_count_  {0};
  int  total_restart_attempts_     {0};
  bool is_healthy_                 {false};

  // 显式 RCL_ROS_TIME 避免 "different time sources" 异常
  rclcpp::Time last_success_time_ {0, 0, RCL_ROS_TIME};
  rclcpp::Time last_restart_time_ {0, 0, RCL_ROS_TIME};

  SerialErrorCode last_error_code_ {SerialErrorCode::OK};
  std::string     last_error_msg_  {"OK"};
};

}  // namespace serial_driver
}  // namespace pka

#endif  // RM_SERIAL_DRIVER_UART_NODE_HPP_