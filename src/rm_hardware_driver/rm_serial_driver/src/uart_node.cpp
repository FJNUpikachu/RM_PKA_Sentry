#include "rm_serial_driver/uart_node.hpp"
#include "rm_serial_driver/uart_node_tool.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>
#include <memory>

#include "rm_utils/pkaLoggerCenter.hpp"

namespace pka {
namespace serial_driver {

// ── 构造 / 析构 ───────────────────────────────────────────────────────────────

UARTNode::UARTNode(const rclcpp::NodeOptions & options)
: Node("serial_driver", options)
{
  PKA_INFO("serial_node", "Starting UARTNode...");
  init_parameters();
  // 创建协议对象（在 init_parameters 解析完 robot_type_str_ 之后）
  protocol_ = ProtocolFactory::create(robot_type_);
  init_serial();
  init_subscriber();
  init_publisher();
  init_timer();
  PKA_INFO("serial_node",
    "UARTNode started (robot_type={} mode={})",
    robot_type_to_string(robot_type_), serial_mode_);
}

UARTNode::~UARTNode()
{
  if (mock_recv_timer_)    { mock_recv_timer_->cancel(); }
  if (send_timer_)         { send_timer_->cancel(); }
  if (recv_timer_)         { recv_timer_->cancel(); }
  if (health_timer_)       { health_timer_->cancel(); }
  if (virtual_send_timer_) { virtual_send_timer_->cancel(); }

  int waited_ms = 0;
  const int wait_step_ms = 50;
  const int max_wait_ms  = restart_delay_ * 2 + 500;
  while (restart_in_progress_.load() && waited_ms < max_wait_ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(wait_step_ms));
    waited_ms += wait_step_ms;
  }
  if (restart_in_progress_.load()) {
    PKA_WARN("serial_node", "Restart thread still running at destruction, force continuing");
  }

  if (serial_ && serial_->isOpen()) {
    serial_->close();
    PKA_INFO("serial_node", "Serial port closed");
  }
  PKA_INFO("serial_node", "UARTNode stopped");
}

// ── 参数 ──────────────────────────────────────────────────────────────────────

void UARTNode::init_parameters()
{
  // ── 机器人类型 ────────────────────────────────────────────────────────────
  robot_type_str_ = declare_parameter("robot_type", std::string("sentry"));
  try {
    robot_type_ = robot_type_from_string(robot_type_str_);
  } catch (const std::invalid_argument & e) {
    PKA_ERROR("serial_node",
      "Invalid robot_type='{}', defaulting to sentry. ({})",
      robot_type_str_, e.what());
    robot_type_ = RobotType::SENTRY;
  }

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

  cmd_vel_linear_scale_  = declare_parameter("cmd_vel_linear_scale",   0.5);
  gimbal_cmd_topic_      = declare_parameter("gimbal_cmd_topic",       "armor_solver/cmd_gimbal");
  cmd_vel_topic_         = declare_parameter("cmd_vel_topic",          "cmd_vel");
  pose_status_topic_     = declare_parameter("pose_status_topic", "PoseStatus");
  target_frame_          = declare_parameter("target_frame",           "odom");

  // ── SetMode 服务名（默认与 armor_detector 的服务名一致）──────────────────
  // 如果有多相机或 namespace 需要修改，可通过此参数覆盖
  set_mode_service_name_ = declare_parameter(
    "set_mode_service_name", std::string("armor_detector/set_mode"));

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
      "Parameters loaded: robot_type={} port={} baud={} mode={} target_frame={} "
      "set_mode_service={}",
      robot_type_str_, port_name_, baudrate_, serial_mode_, target_frame_,
      set_mode_service_name_);
  }
}

// ── 串口初始化 ────────────────────────────────────────────────────────────────

void UARTNode::init_serial()
{
  if (serial_mode_ == 0) {
    is_healthy_ = true;
    consecutive_failure_count_ = 0;
    last_success_time_ = this->now();
    PKA_INFO("serial_node", "serial_mode=0: serial port disabled (mock mode)");
    return;
  }

  if (serial_ && serial_->isOpen()) { serial_->close(); }
  serial_.reset();

  try {
    serial_ = std::make_unique<serial::Serial>(
      port_name_,
      static_cast<uint32_t>(baudrate_),
      serial::Timeout::simpleTimeout(100)
    );

    if (serial_->isOpen()) {
      serial_->flush();
      is_healthy_                = true;
      consecutive_failure_count_ = 0;
      last_error_code_           = SerialErrorCode::OK;
      last_error_msg_            = "OK";
      last_success_time_         = this->now();
      PKA_INFO("serial_node", "Serial port opened: {} @ {} baud", port_name_, baudrate_);
    } else {
      PKA_ERROR("serial_node", "isOpen() returned false after construction");
    }

  } catch (const serial::IOException & e) {
    serial_.reset();
    is_healthy_      = false;
    last_error_code_ = SerialErrorCode::DEVICE_NOT_FOUND;
    last_error_msg_  = e.what();
    PKA_ERROR("serial_node", "IOException opening [{}]: {}", port_name_, e.what());
    UARTNodeTool::on_error(this, SerialErrorCode::DEVICE_NOT_FOUND, e.what());

  } catch (const serial::SerialException & e) {
    serial_.reset();
    is_healthy_      = false;
    last_error_code_ = SerialErrorCode::UNKNOWN_ERROR;
    last_error_msg_  = e.what();
    PKA_ERROR("serial_node", "SerialException opening [{}]: {}", port_name_, e.what());
    UARTNodeTool::on_error(this, SerialErrorCode::UNKNOWN_ERROR, e.what());

  } catch (const std::invalid_argument & e) {
    serial_.reset();
    is_healthy_      = false;
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
  }

  // cmd_vel 仅哨兵协议需要（步兵发送帧无底盘速度字段）
  if (robot_type_ == RobotType::SENTRY && !cmd_vel_topic_.empty()) {
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::SensorDataQoS(),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        UARTNodeTool::cmd_vel_callback(this, msg);
      });
    PKA_INFO("serial_node", "Subscribed to cmd_vel: '{}' (sentry only)", cmd_vel_topic_);

  if (!pose_status_topic_.empty()) {
    pose_status_sub_ = create_subscription<rm_interfaces::msg::PoseStatus>(
      pose_status_topic_, 10,
      [this](const rm_interfaces::msg::PoseStatus::SharedPtr msg) {
        UARTNodeTool::pose_status_callback(this, msg);
      });
    PKA_INFO("serial_node", "Subscribed to pose_status: '{}'", pose_status_topic_);
  }
}

// ── 发布 / 服务客户端 ─────────────────────────────────────────────────────────

void UARTNode::init_publisher()
{
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  heartbeat_pub_  = HeartBeatPublisher::create(this);

  // ── SetMode 服务客户端（通知 armor_detector 切换识别颜色）────────────────
  set_mode_client_ = create_client<rm_interfaces::srv::SetMode>(set_mode_service_name_);
  PKA_INFO("serial_node",
    "SetMode client created for service: '{}'", set_mode_service_name_);

  switch (robot_type_) {
    case RobotType::SENTRY:
      sentry_recv_pub_ = create_publisher<rm_interfaces::msg::SentrySerialReceiveData>(
        "serial/receive", rclcpp::SensorDataQoS());
      game_status_pub_  = create_publisher<rm_interfaces::msg::GameStatus>(
        "/game_status", 10);
      rfid_status_pub_  = create_publisher<rm_interfaces::msg::RfidStatus>(
        "/rfid_status",  10);
      robot_status_pub_ = create_publisher<rm_interfaces::msg::RobotStatus>(
        "/robot_status", 10);
      tower_status_pub_ = create_publisher<rm_interfaces::msg::TowerStatus>(
        "/tower_status", 10);
      PKA_INFO("serial_node",
        "Publishers: serial/receive(sentry), game_status, rfid_status, robot_status,tower_status");
      break;

    case RobotType::INFANTRY:
      infantry_recv_pub_ = create_publisher<rm_interfaces::msg::InfantrySerialReceiveData>(
        "serial/receive", rclcpp::SensorDataQoS());
      PKA_INFO("serial_node", "Publishers: serial/receive(infantry)");
      break;
  }
}

// ── 定时器 ────────────────────────────────────────────────────────────────────

void UARTNode::init_timer()
{
  auto hz_ms  = [](double hz)  {
    return std::chrono::milliseconds(static_cast<int>(1000.0 / hz));
  };
  auto sec_ms = [](double sec) {
    return std::chrono::milliseconds(static_cast<int>(sec * 1000.0));
  };

  switch (serial_mode_) {
    case 0:
      mock_recv_timer_ = create_wall_timer(
        hz_ms(mock_recv_frequency_),
        [this]() { UARTNodeTool::mock_recv_timer_cb(this); });
      PKA_DEBUG("serial_node", "Mode=0: mock_recv @ {:.1f} Hz", mock_recv_frequency_);
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
      PKA_DEBUG("serial_node",
        "Mode=1: send@{:.1f}Hz recv@{:.1f}Hz health@{:.1f}s",
        send_frequency_, read_frequency_, health_check_interval_);
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
      PKA_DEBUG("serial_node",
        "Mode=2: virtual_send@{:.1f}Hz recv@{:.1f}Hz health@{:.1f}s",
        virtual_send_frequency_, read_frequency_, health_check_interval_);
      break;

    default:
      PKA_ERROR("serial_node", "Unknown serial_mode={}", serial_mode_);
      break;
  }
}

// ── TF 广播 ───────────────────────────────────────────────────────────────────

void UARTNode::broadcastGimbalTF(
  float roll, float pitch, float yaw, float chassis_imu_yaw_offset)
{
  timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();

  tf2::Quaternion q;
  q.setRPY(
    static_cast<double>(roll)  * M_PI / 180.0,
    static_cast<double>(pitch) * M_PI / 180.0,
    static_cast<double>(yaw)   * M_PI / 180.0);

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp    = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
  t.header.frame_id = target_frame_;
  t.child_frame_id  = "gimbal_link";
  t.transform.rotation      = tf2::toMsg(q);
  t.transform.translation.x = 0.0;
  t.transform.translation.y = 0.0;
  t.transform.translation.z = 0.0;
  tf_broadcaster_->sendTransform(t);

  tf2::Quaternion q1;
  q1.setRPY(0.0, 0.0,
    static_cast<double>(chassis_imu_yaw_offset) * M_PI / 180.0);

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
      "TF: {}->gimbal_link  RPY=({:.3f}°, {:.3f}°, {:.3f}°)",
      target_frame_, roll, pitch, yaw);
  }
}

}  // namespace serial_driver
}  // namespace pka

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pka::serial_driver::UARTNode)