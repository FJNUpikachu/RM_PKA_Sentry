#include "rm_serial_driver/uart_node.hpp"
#include "rm_serial_driver/uart_node_tool.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>
#include <memory>

// pkaLoggerCenter ─ 异步落盘 + 彩色终端输出
#include "rm_utils/pkaLoggerCenter.hpp"

namespace pka {
namespace serial_driver {

// ── 构造 / 析构 ───────────────────────────────────────────────────────────────

UARTNode::UARTNode(const rclcpp::NodeOptions & options)
: Node("serial_driver", options)
{
  PKA_INFO("serial_node", "Starting UARTNode...");
  init_parameters();
  init_serial();
  init_subscriber();
  init_publisher();
  init_timer();
  PKA_INFO("serial_node", "UARTNode started (mode={})", serial_mode_);
}

UARTNode::~UARTNode()
{
  if (mock_recv_timer_)    { mock_recv_timer_->cancel(); }
  if (send_timer_)         { send_timer_->cancel(); }
  if (recv_timer_)         { recv_timer_->cancel(); }
  if (health_timer_)       { health_timer_->cancel(); }
  if (virtual_send_timer_) { virtual_send_timer_->cancel(); }

  if (serial_ && serial_->isOpen()) {
    serial_->close();
    PKA_INFO("serial_node", "Serial port closed");
  }
  PKA_INFO("serial_node", "UARTNode stopped");
}

// ── 参数 ──────────────────────────────────────────────────────────────────────

void UARTNode::init_parameters()
{
  port_name_             = declare_parameter("port_name",              "/dev/ttyACM0");
  baudrate_              = declare_parameter("baudrate",               115200);
  timestamp_offset_      = declare_parameter("timestamp_offset",       0.006);
  debug_                 = declare_parameter("debug",                  false);
  serial_mode_           = declare_parameter("serial_mode",            1);

  virtual_send_frequency_= declare_parameter("virtual_send_frequency", 50.0);
  send_frequency_        = declare_parameter("send_frequency",         100.0);
  read_frequency_        = declare_parameter("read_frequency",         100.0);
  max_failure_count_     = declare_parameter("max_failure_count",      10);
  health_check_interval_ = declare_parameter("health_check_interval",  1.0);
  max_restart_attempts_  = declare_parameter("max_restart_attempts",   5);
  restart_cooldown_      = declare_parameter("restart_cooldown",       2.0);
  enable_auto_restart_   = declare_parameter("enable_auto_restart",    true);
  restart_delay_         = declare_parameter("restart_delay",          1000);

  cmd_vel_linear_scale_  = declare_parameter("cmd_vel_linear_scale",   0.2);
  gimbal_cmd_topic_      = declare_parameter("gimbal_cmd_topic",       "armor_solver/cmd_gimbal");
  cmd_vel_topic_         = declare_parameter("cmd_vel_topic",          "cmd_vel");
  target_frame_          = declare_parameter("target_frame",           "odom");

  mock_mode_                   = static_cast<uint8_t>(declare_parameter("mock_mode",   0));
  mock_roll_                   = declare_parameter("mock_roll",         0.0f);
  mock_pitch_                  = declare_parameter("mock_pitch",        0.0f);
  mock_yaw_                    = declare_parameter("mock_yaw",          0.0f);
  mock_chassis_imu_yaw_offset_ = declare_parameter("mock_chassis_imu_yaw_offset", 0.0f);
  mock_recv_frequency_         = declare_parameter("mock_recv_frequency", 50.0);

  virtual_fire_advice_ = declare_parameter("virtual_fire_advice", false);
  virtual_pitch_       = declare_parameter("virtual_pitch",       0.0f);
  virtual_yaw_         = declare_parameter("virtual_yaw",         0.0f);
  virtual_distance_    = declare_parameter("virtual_distance",    0.0f);
  virtual_linear_x_    = declare_parameter("virtual_linear_x",   0.0f);
  virtual_linear_y_    = declare_parameter("virtual_linear_y",   0.0f);
  virtual_angular_z_   = declare_parameter("virtual_angular_z",  0.0f);

  if (debug_) {
    PKA_DEBUG("serial_node",
      "Parameters loaded: port={} baud={} mode={} target_frame={}",
      port_name_, baudrate_, serial_mode_, target_frame_);
    PKA_DEBUG("serial_node",
      "Topics: gimbal_cmd='{}' cmd_vel='{}'",
      gimbal_cmd_topic_, cmd_vel_topic_);
  }
}

// ── 串口初始化 ────────────────────────────────────────────────────────────────
//
// 参考 serial_example.cc：
//   serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));
//   if (my_serial.isOpen()) { ... }
//
void UARTNode::init_serial()
{
  if (serial_mode_ == 0) {
    is_healthy_ = true;
    consecutive_failure_count_ = 0;
    last_success_time_ = this->now();
    PKA_INFO("serial_node", "serial_mode=0: serial port disabled (mock mode)");
    return;
  }

  // 先关旧串口（重启场景）
  if (serial_ && serial_->isOpen()) {
    serial_->close();
  }
  serial_.reset();

  try {
    // ── 与 example 完全一致的构造方式 ────────────────────────────────────────
    // serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(ms))
    serial_ = std::make_unique<serial::Serial>(
      port_name_,
      static_cast<uint32_t>(baudrate_),
      serial::Timeout::simpleTimeout(100)  // 100ms timeout，不阻塞定时器
    );

    // ── 与 example 完全一致的 isOpen() 检查 ──────────────────────────────────
    if (serial_->isOpen()) {
      serial_->flush();
      is_healthy_ = true;
      consecutive_failure_count_ = 0;
      last_error_code_ = SerialErrorCode::OK;
      last_error_msg_  = "OK";
      last_success_time_ = this->now();
      PKA_INFO("serial_node", "Serial port opened: {} @ {} baud", port_name_, baudrate_);
    } else {
      PKA_ERROR("serial_node","isOpen() returned false after construction");
      return;
    }

  } catch (const serial::IOException & e) {
    serial_.reset();
    is_healthy_ = false;
    last_error_code_ = SerialErrorCode::DEVICE_NOT_FOUND;
    last_error_msg_  = e.what();
    PKA_ERROR("serial_node", "IOException opening [{}]: {}", port_name_, e.what());
    UARTNodeTool::on_error(this, SerialErrorCode::DEVICE_NOT_FOUND, e.what());

  } catch (const serial::SerialException & e) {
    serial_.reset();
    is_healthy_ = false;
    last_error_code_ = SerialErrorCode::UNKNOWN_ERROR;
    last_error_msg_  = e.what();
    PKA_ERROR("serial_node", "SerialException opening [{}]: {}", port_name_, e.what());
    UARTNodeTool::on_error(this, SerialErrorCode::UNKNOWN_ERROR, e.what());

  } catch (const std::invalid_argument & e) {
    serial_.reset();
    is_healthy_ = false;
    last_error_code_ = SerialErrorCode::INVALID_PARAMETER;
    last_error_msg_  = e.what();
    PKA_ERROR("serial_node", "InvalidArgument opening serial: {}", e.what());
    UARTNodeTool::on_error(this, SerialErrorCode::INVALID_PARAMETER, e.what());
  }
}

// ── 订阅 ──────────────────────────────────────────────────────────────────────

void UARTNode::init_subscriber()
{
  if (serial_mode_ != 1) {
    if (debug_) {
      PKA_DEBUG("serial_node", "Subscribers skipped (mode={})", serial_mode_);
    }
    return;
  }

  if (!gimbal_cmd_topic_.empty()) {
    gimbal_sub_ = create_subscription<rm_interfaces::msg::GimbalCmd>(
      gimbal_cmd_topic_, rclcpp::SensorDataQoS(),
      [this](const rm_interfaces::msg::GimbalCmd::SharedPtr msg) {
        UARTNodeTool::gimbal_cmd_callback(this, msg);
      });
    PKA_INFO("serial_node", "Subscribed to gimbal_cmd: '{}'", gimbal_cmd_topic_);
  } else {
    PKA_WARN("serial_node",
      "gimbal_cmd_topic is empty, fire_advice/pitch/yaw/distance will be 0");
  }

  if (!cmd_vel_topic_.empty()) {
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::SensorDataQoS(),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        UARTNodeTool::cmd_vel_callback(this, msg);
      });
    PKA_INFO("serial_node", "Subscribed to cmd_vel: '{}'", cmd_vel_topic_);
  } else {
    PKA_WARN("serial_node",
      "cmd_vel_topic is empty, linear_x/linear_y/angular_z will be 0");
  }
}

// ── 发布 ──────────────────────────────────────────────────────────────────────

void UARTNode::init_publisher()
{
  recv_pub_ = create_publisher<rm_interfaces::msg::SerialReceiveData>(
    "serial/receive", rclcpp::SensorDataQoS());

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  heartbeat_pub_  = HeartBeatPublisher::create(this);

  if (debug_) {
    PKA_DEBUG("serial_node", "Publishers and TF broadcaster initialized");
  }
}

// ── 定时器 ────────────────────────────────────────────────────────────────────

void UARTNode::init_timer()
{
  auto hz_ms  = [](double hz)  { return std::chrono::milliseconds(static_cast<int>(1000.0 / hz)); };
  auto sec_ms = [](double sec) { return std::chrono::milliseconds(static_cast<int>(sec * 1000.0)); };

  switch (serial_mode_) {
    case 0:
      mock_recv_timer_ = create_wall_timer(
        hz_ms(mock_recv_frequency_),
        [this]() { UARTNodeTool::mock_recv_timer_cb(this); });
      if (debug_) {
        PKA_DEBUG("serial_node", "Mode=0: mock_recv @ {:.1f} Hz", mock_recv_frequency_);
      }
      break;

    case 1:
      send_timer_ = create_wall_timer(
        hz_ms(send_frequency_),
        [this]() { UARTNodeTool::send_timer_cb(this); });
      recv_timer_ = create_wall_timer(
        hz_ms(read_frequency_),
        [this]() { UARTNodeTool::recv_timer_cb(this); });
      health_timer_ = create_wall_timer(
        sec_ms(health_check_interval_),
        [this]() { UARTNodeTool::health_timer_cb(this); });
      if (debug_) {
        PKA_DEBUG("serial_node",
          "Mode=1: send@{:.1f}Hz recv@{:.1f}Hz health@{:.1f}s",
          send_frequency_, read_frequency_, health_check_interval_);
      }
      break;

    case 2:
      virtual_send_timer_ = create_wall_timer(
        hz_ms(virtual_send_frequency_),
        [this]() { UARTNodeTool::virtual_send_timer_cb(this); });
      recv_timer_ = create_wall_timer(
        hz_ms(read_frequency_),
        [this]() { UARTNodeTool::recv_timer_cb(this); });
      health_timer_ = create_wall_timer(
        sec_ms(health_check_interval_),
        [this]() { UARTNodeTool::health_timer_cb(this); });
      if (debug_) {
        PKA_DEBUG("serial_node",
          "Mode=2: virtual_send@{:.1f}Hz recv@{:.1f}Hz health@{:.1f}s",
          virtual_send_frequency_, read_frequency_, health_check_interval_);
      }
      break;

    default:
      PKA_ERROR("serial_node", "Unknown serial_mode={}", serial_mode_);
      break;
  }
}

// ── TF 广播 ───────────────────────────────────────────────────────────────────
//
// 时间戳 = now() - abs(timestamp_offset_)
// 保证 armors 消息时间戳（now()）落在 TF buffer 覆盖范围内，
// 彻底避免 tf2_filter "timestamp earlier than all data" 错误。
//
void UARTNode::broadcastGimbalTF(const rm_interfaces::msg::SerialReceiveData & data)
{
  timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();

  tf2::Quaternion q;
  q.setRPY(
    data.roll  * M_PI / 180.0,
    data.pitch * M_PI / 180.0,
    data.yaw   * M_PI / 180.0);

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp    = this->now() - rclcpp::Duration::from_seconds(std::abs(timestamp_offset_));
  t.header.frame_id = target_frame_;
  t.child_frame_id  = "gimbal_link";
  t.transform.rotation      = tf2::toMsg(q);
  t.transform.translation.x = 0.0;
  t.transform.translation.y = 0.0;
  t.transform.translation.z = 0.0;
  tf_broadcaster_->sendTransform(t);

  tf2::Quaternion q1;
  q1.setRPY(
    0.0,
    0.0,
    data.chassis_imu_yaw_offset   * M_PI / 180.0);

  geometry_msgs::msg::TransformStamped t1;
  t1.header.stamp    = this->now() - rclcpp::Duration::from_seconds(std::abs(timestamp_offset_));
  t1.header.frame_id = "base_link";
  t1.child_frame_id  = target_frame_;
  t1.transform.rotation      = tf2::toMsg(q1);
  t1.transform.translation.x = 0.002988;
  t1.transform.translation.y = 0.0;
  t1.transform.translation.z = 0.272;
  tf_broadcaster_->sendTransform(t1);

  if (debug_) {
    PKA_DEBUG("serial_node",
      "TF: {}->gimbal_link  RPY=({:.3f}°, {:.3f}°, {:.3f}°)  stamp=now-{:.4f}s",
      target_frame_, data.roll, data.pitch, data.yaw, std::abs(timestamp_offset_));
  }
}

}  // namespace serial_driver
}  // namespace pka

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pka::serial_driver::UARTNode)