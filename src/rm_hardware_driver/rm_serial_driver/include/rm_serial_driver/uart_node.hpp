#ifndef RM_SERIAL_DRIVER_UART_NODE_HPP_
#define RM_SERIAL_DRIVER_UART_NODE_HPP_

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/twist.hpp>

// std
#include <atomic>
#include <thread>
#include <memory>

// project interfaces
#include "rm_utils/heartbeat.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"

// ── 哨兵协议消息 ──────────────────────────────────────────────────────────────
#include "rm_interfaces/msg/sentry_serial_send_data.hpp"
#include "rm_interfaces/msg/sentry_serial_receive_data.hpp"
// ── 步兵协议消息 ──────────────────────────────────────────────────────────────
#include "rm_interfaces/msg/infantry_serial_send_data.hpp"
#include "rm_interfaces/msg/infantry_serial_receive_data.hpp"

// ── 裁判系统（仅哨兵协议携带） ───────────────────────────────────────────────
#include "rm_interfaces/msg/game_status.hpp"
#include "rm_interfaces/msg/rfid_status.hpp"
#include "rm_interfaces/msg/robot_status.hpp"
#include "rm_interfaces/msg/pose_status.hpp"
#include "rm_interfaces/msg/tower_status.hpp"

// ── SetMode 服务（用于通知 armor_detector 切换颜色）──────────────────────────
#include "rm_interfaces/srv/set_mode.hpp"

// 内嵌 wjwwood/serial 库
#include "serial/serial.h"

// ── 协议层 ────────────────────────────────────────────────────────────────────
#include "rm_serial_driver/protocol/uart_protocol_base.hpp"
#include "rm_serial_driver/protocol/protocol_factory.hpp"
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
  void init_serial();
  void init_subscriber();
  void init_publisher();
  void init_timer();

  // TF 广播（roll/pitch/yaw 单位：度）—— 哨兵/步兵均可调用
  void broadcastGimbalTF(float roll, float pitch, float yaw,
                         float chassis_imu_yaw_offset = 0.0f);

  // ── 协议对象（由工厂根据 robot_type_ 创建） ───────────────────────────────
  std::unique_ptr<IUARTProtocol> protocol_;
  RobotType                      robot_type_ {RobotType::SENTRY};

  // ── 串口对象 ──────────────────────────────────────────────────────────────
  std::unique_ptr<serial::Serial> serial_;

  // ── ROS 2 公共 ────────────────────────────────────────────────────────────
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  HeartBeatPublisher::SharedPtr                  heartbeat_pub_;

  // ── 哨兵专用发布 ──────────────────────────────────────────────────────────
  rclcpp::Publisher<rm_interfaces::msg::SentrySerialReceiveData>::SharedPtr
    sentry_recv_pub_;
  rclcpp::Publisher<rm_interfaces::msg::GameStatus>::SharedPtr  game_status_pub_;
  rclcpp::Publisher<rm_interfaces::msg::RfidStatus>::SharedPtr  rfid_status_pub_;
  rclcpp::Publisher<rm_interfaces::msg::RobotStatus>::SharedPtr robot_status_pub_;
  rclcpp::Publisher<rm_interfaces::msg::TowerStatus>::SharedPtr tower_status_pub_;


  // ── 步兵专用发布 ──────────────────────────────────────────────────────────
  rclcpp::Publisher<rm_interfaces::msg::InfantrySerialReceiveData>::SharedPtr
    infantry_recv_pub_;

  // ── 订阅（哨兵 mode=1 有效；步兵 cmd_vel_topic 可为空） ───────────────────
  rclcpp::Subscription<rm_interfaces::msg::GimbalCmd>::SharedPtr   gimbal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr       cmd_vel_sub_;

  rclcpp::Subscription<rm_interfaces::msg::PoseStatus>::SharedPtr pose_status_sub_;

  // ── SetMode 服务客户端（调用 armor_detector/set_mode）────────────────────
  rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr set_mode_client_;

  // ── 缓存（未到达保持 0） ─────────────────────────────────────────────────
  bool  cached_fire_advice_ {false};
  float cached_pitch_       {0.0f};
  float cached_yaw_         {0.0f};
  float cached_distance_    {0.0f};
  float cached_linear_x_    {0.0f};   // 仅哨兵
  float cached_linear_y_    {0.0f};   // 仅哨兵
  float cached_angular_z_   {0.0f};   // 仅哨兵
  uint8_t cached_pose_status_ {0};

  // ── 上一次发送给 armor_detector 的 mode（用于变化检测）───────────────────
  // 初始值设为 255（无效值），保证第一帧必定触发服务调用
  uint8_t last_sent_mode_   {255};

  // ── 定时器 ────────────────────────────────────────────────────────────────
  rclcpp::TimerBase::SharedPtr mock_recv_timer_;
  rclcpp::TimerBase::SharedPtr send_timer_;
  rclcpp::TimerBase::SharedPtr recv_timer_;
  rclcpp::TimerBase::SharedPtr health_timer_;
  rclcpp::TimerBase::SharedPtr virtual_send_timer_;

  // ── 参数 ─────────────────────────────────────────────────────────────────
  std::string robot_type_str_;          // "sentry" | "infantry"
  std::string port_name_;
  int         baudrate_;
  double      timestamp_offset_;
  bool        debug_;
  int         serial_mode_;             // 0=mock  1=real  2=virtual_send

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

  std::string pose_status_topic_;

  // ── set_mode 服务相关参数 ─────────────────────────────────────────────────
  // armor_detector 服务名，可通过参数覆盖
  std::string set_mode_service_name_;

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

  rclcpp::Time last_success_time_ {0, 0, RCL_ROS_TIME};
  rclcpp::Time last_restart_time_ {0, 0, RCL_ROS_TIME};

  SerialErrorCode last_error_code_ {SerialErrorCode::OK};
  std::string     last_error_msg_  {"OK"};

  std::atomic<bool> restart_in_progress_ {false};
};

}  // namespace serial_driver
}  // namespace pka

#endif  // RM_SERIAL_DRIVER_UART_NODE_HPP_