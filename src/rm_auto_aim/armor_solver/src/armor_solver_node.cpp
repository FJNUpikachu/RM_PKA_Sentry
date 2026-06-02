#include "armor_solver/armor_solver_node.hpp"

// std
#include <algorithm>
#include <memory>
#include <vector>
// project
#include "armor_solver/motion_model.hpp"
#include "rm_utils/common.hpp"
#include "rm_utils/heartbeat.hpp"

namespace pka::auto_aim {
ArmorSolverNode::ArmorSolverNode(const rclcpp::NodeOptions &options)
: Node("armor_solver", options), solver_(nullptr) {
  PKA_INFO("armor_solver", "Starting ArmorSolverNode!");

  // 调试开关
  debug_mode_    = this->declare_parameter("debug", true);
  debug_tracker_ = this->declare_parameter("debug_tracker", false);
  debug_ekf_     = this->declare_parameter("debug_ekf",     false);
  debug_solver_  = this->declare_parameter("debug_solver",  false);
  debug_filter_  = this->declare_parameter("debug_filter",  false);
  debug_outpost_ = this->declare_parameter("debug_outpost", false);
  debug_outpost  = debug_outpost_;

  PKA_INFO("armor_solver",
           "Debug flags: debug={} tracker={} ekf={} solver={} filter={} outpost={}",
           debug_mode_, debug_tracker_, debug_ekf_, debug_solver_, debug_filter_, debug_outpost_);

  // 最大跟踪距离（3D），按目标类型分流
  max_armor_distance_          = this->declare_parameter("max_armor_distance", 5.8);
  max_outpost_armor_distance_  = this->declare_parameter("outpost.max_outpost_armor_distance", max_armor_distance_);
  max_base_armor_distance_     = this->declare_parameter("base.max_base_armor_distance", max_armor_distance_);

  this->declare_parameter("base.yaw_offset", 0.0);
  this->declare_parameter("base.pitch_offset", 0.0);

  // 前哨站参数（声明与读取集中在 outpost_solver）
  declareOutpostParameters(*this);
  outpost_params_ = loadOutpostParams(*this);

  // ── 地面兵种 Tracker ─────────────────────────────────────────────────────
  double max_match_distance = this->declare_parameter("tracker.max_match_distance", 0.2);
  double max_match_yaw_diff = this->declare_parameter("tracker.max_match_yaw_diff", 1.0);
  tracker_ = std::make_unique<Tracker>(max_match_distance, max_match_yaw_diff);
  tracker_->tracking_thres = this->declare_parameter("tracker.tracking_thres", 5);
  lost_time_thres_ = this->declare_parameter("tracker.lost_time_thres", 0.3);

  // ── 前哨站 Tracker（与地面兵种分开，仅 yaw 阈值和 tracking/lost 阈值）───────
  double outpost_max_match_yaw_diff =
    this->declare_parameter("outpost.tracker.max_match_yaw_diff", max_match_yaw_diff);
  tracker_->setOutpostTrackerGates(outpost_max_match_yaw_diff);

  outpost_tracking_thres_ =
    this->declare_parameter("outpost.tracker.tracking_thres", tracker_->tracking_thres);
  outpost_lost_time_thres_ =
    this->declare_parameter("outpost.tracker.lost_time_thres", lost_time_thres_);

  // ── 速度限幅参数 ──────────────────────────────────────────────────────────
  tracker_->vel_clamp_frames      = this->declare_parameter("vel_clamp.frames",      5);
  tracker_->vel_clamp_linear_max  = this->declare_parameter("vel_clamp.linear_max",  3.0);
  //std::cout<<tracker_->vel_clamp_linear_max<<std::endl;
  tracker_->vel_clamp_yaw_max     = this->declare_parameter("vel_clamp.yaw_max",     4.0);
  //std::cout<<tracker_->vel_clamp_yaw_max<<std::endl;

  // ── 地面兵种 EKF 状态钳位参数 ────────────────────────────────────────────
  tracker_->ground_state_clamp_enable =
    this->declare_parameter("ekf.state_clamp.enable", true);
  tracker_->ground_radius_min =
    this->declare_parameter("ekf.state_clamp.radius_min", 0.18);
  tracker_->ground_radius_max =
    this->declare_parameter("ekf.state_clamp.radius_max", 0.35);
  tracker_->ground_d_zc_min =
    this->declare_parameter("ekf.state_clamp.d_zc_min", -0.35);
  tracker_->ground_d_zc_max =
    this->declare_parameter("ekf.state_clamp.d_zc_max",  0.35);
  tracker_->ground_v_yaw_min =
    this->declare_parameter("ekf.state_clamp.v_yaw_min", -20.0);
  tracker_->ground_v_yaw_max =
    this->declare_parameter("ekf.state_clamp.v_yaw_max",  20.0);

  // ── 前哨站 EKF 状态钳位参数 ──────────────────────────────────────────────
  tracker_->outpost_state_clamp_enable =
    this->declare_parameter("outpost.ekf.state_clamp.enable", true);
  tracker_->outpost_radius_min =
    this->declare_parameter("outpost.ekf.state_clamp.radius_min", 0.18);
  tracker_->outpost_radius_max =
    this->declare_parameter("outpost.ekf.state_clamp.radius_max", 0.40);
  tracker_->outpost_v_yaw_min =
    this->declare_parameter("outpost.ekf.state_clamp.v_yaw_min", -4.0);
  tracker_->outpost_v_yaw_max =
    this->declare_parameter("outpost.ekf.state_clamp.v_yaw_max",  4.0);

  // 注入分组调试开关和前哨站配置
  tracker_->debug_tracker = debug_tracker_;
  tracker_->debug_ekf     = debug_ekf_;
  tracker_->outpost_cfg_  = outpost_params_;

  // ── EKF 过程噪声参数（地面兵种）────────────────────────────────────────────
  auto f = Predict(0.005);
  auto h = Measure();
  s2qx_    = this->declare_parameter("ekf.sigma2_q_x",    20.0);
  s2qy_    = this->declare_parameter("ekf.sigma2_q_y",    20.0);
  s2qz_    = this->declare_parameter("ekf.sigma2_q_z",    20.0);
  s2qyaw_  = this->declare_parameter("ekf.sigma2_q_yaw",  100.0);
  s2qr_    = this->declare_parameter("ekf.sigma2_q_r",    800.0);
  s2qd_zc_ = this->declare_parameter("ekf.sigma2_q_d_zc", 800.0);

  // ── EKF 过程噪声参数（前哨站，单独调节）─────────────────────────────────────
  outpost_s2qx_    = this->declare_parameter("outpost.ekf.sigma2_q_x",    s2qx_);
  outpost_s2qy_    = this->declare_parameter("outpost.ekf.sigma2_q_y",    s2qy_);
  outpost_s2qz_    = this->declare_parameter("outpost.ekf.sigma2_q_z",    s2qz_);
  outpost_s2qyaw_  = this->declare_parameter("outpost.ekf.sigma2_q_yaw",  s2qyaw_);
  outpost_s2qr_    = this->declare_parameter("outpost.ekf.sigma2_q_r",    s2qr_);
  outpost_s2qd_zc_ = this->declare_parameter("outpost.ekf.sigma2_q_d_zc", s2qd_zc_);

  // ── EKF 测量噪声参数（地面兵种）─────────────────────────────────────────────
  r_x_   = this->declare_parameter("ekf.r_x",   0.05);
  r_y_   = this->declare_parameter("ekf.r_y",   0.05);
  r_z_   = this->declare_parameter("ekf.r_z",   0.05);
  r_yaw_ = this->declare_parameter("ekf.r_yaw", 0.02);

  // ── EKF 测量噪声参数（前哨站，单独调节）──────────────────────────────────────
  outpost_r_x_   = this->declare_parameter("outpost.ekf.r_x",   r_x_);
  outpost_r_y_   = this->declare_parameter("outpost.ekf.r_y",   r_y_);
  outpost_r_z_   = this->declare_parameter("outpost.ekf.r_z",   r_z_);
  outpost_r_yaw_ = this->declare_parameter("outpost.ekf.r_yaw", r_yaw_);

  // u_q lambda：根据当前跟踪类型自动选择地面或前哨站噪声参数
  auto u_q = [this]()
  {
    const bool is_outpost = tracker_ &&
                            isOutpostId(tracker_->tracked_id, outpost_params_.id);
    Eigen::Matrix<double, X_N, X_N> q;
    const double t   = dt_;
    const double x   = is_outpost ? outpost_s2qx_    : s2qx_;
    const double y   = is_outpost ? outpost_s2qy_    : s2qy_;
    const double z   = is_outpost ? outpost_s2qz_    : s2qz_;
    const double yaw = is_outpost ? outpost_s2qyaw_  : s2qyaw_;
    const double r   = is_outpost ? outpost_s2qr_    : s2qr_;
    const double dzc = is_outpost ? outpost_s2qd_zc_ : s2qd_zc_;

    const double q_x_x  = pow(t, 4) / 4 * x,   q_x_vx  = pow(t, 3) / 2 * x,  q_vx_vx = pow(t, 2) * x;
    const double q_y_y  = pow(t, 4) / 4 * y,   q_y_vy  = pow(t, 3) / 2 * y,  q_vy_vy = pow(t, 2) * y;
    const double q_z_z  = pow(t, 4) / 4 * z,   q_z_vz  = pow(t, 3) / 2 * z,  q_vz_vz = pow(t, 2) * z;
    const double q_yaw_yaw   = pow(t, 4) / 4 * yaw;
    const double q_yaw_vyaw  = pow(t, 3) / 2 * yaw;
    const double q_vyaw_vyaw = pow(t, 2)       * yaw;
    const double q_r   = pow(t, 4) / 4 * r;
    const double q_dzc = pow(t, 4) / 4 * dzc;
    // clang-format off
    //    xc      v_xc    yc      v_yc    zc      v_zc    yaw          v_yaw        r      d_zc
    q <<  q_x_x,  q_x_vx, 0,      0,      0,      0,      0,           0,           0,     0,
          q_x_vx, q_vx_vx,0,      0,      0,      0,      0,           0,           0,     0,
          0,      0,      q_y_y,  q_y_vy, 0,      0,      0,           0,           0,     0,
          0,      0,      q_y_vy, q_vy_vy,0,      0,      0,           0,           0,     0,
          0,      0,      0,      0,      q_z_z,  q_z_vz, 0,           0,           0,     0,
          0,      0,      0,      0,      q_z_vz, q_vz_vz,0,           0,           0,     0,
          0,      0,      0,      0,      0,      0,      q_yaw_yaw,   q_yaw_vyaw,  0,     0,
          0,      0,      0,      0,      0,      0,      q_yaw_vyaw,  q_vyaw_vyaw, 0,     0,
          0,      0,      0,      0,      0,      0,      0,           0,           q_r,   0,
          0,      0,      0,      0,      0,      0,      0,           0,           0,     q_dzc;
    // clang-format on
    return q;
  };

  // u_r lambda：根据当前跟踪类型自动选择地面或前哨站测量噪声参数
  auto u_r = [this](const Eigen::Matrix<double, Z_N, 1> &z)
  {
    const bool is_outpost = tracker_ &&
                            isOutpostId(tracker_->tracked_id, outpost_params_.id);
    const double rx   = is_outpost ? outpost_r_x_   : r_x_;
    const double ry   = is_outpost ? outpost_r_y_   : r_y_;
    const double rz   = is_outpost ? outpost_r_z_   : r_z_;
    const double ryaw = is_outpost ? outpost_r_yaw_ : r_yaw_;
    Eigen::Matrix<double, Z_N, Z_N> r;
    // clang-format off
    r << rx * std::abs(z[0]), 0,                  0,                  0,
         0,                   ry * std::abs(z[1]), 0,                  0,
         0,                   0,                   rz * std::abs(z[2]),0,
         0,                   0,                   0,                   ryaw;
    // clang-format on
    return r;
  };

  Eigen::DiagonalMatrix<double, X_N> p0;
  p0.setIdentity();
  tracker_->ekf = std::make_unique<RobotStateEKF>(f, h, u_q, u_r, p0);

  // Subscriber with tf2 message_filter
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  armors_sub_.subscribe(this, "armor_detector/armors", rmw_qos_profile_sensor_data);
  target_frame_ = this->declare_parameter("target_frame", "odom");
  tf2_filter_ = std::make_shared<tf2_filter>(armors_sub_,
                                             *tf2_buffer_,
                                             target_frame_,
                                             10,
                                             this->get_node_logging_interface(),
                                             this->get_node_clock_interface(),
                                             std::chrono::duration<int>(1));
  tf2_filter_->registerCallback(&ArmorSolverNode::armorsCallback, this);

  measure_pub_ = this->create_publisher<rm_interfaces::msg::Measurement>("armor_solver/measurement",
                                                                         rclcpp::SensorDataQoS());
  target_pub_ = this->create_publisher<rm_interfaces::msg::Target>("armor_solver/target",
                                                                   rclcpp::SensorDataQoS());
  // Outpost-specific debug publishers
  outpost_measure_pub_ = this->create_publisher<rm_interfaces::msg::OutpostMeasurement>("armor_solver/outpost_measurement",
                                                                                       rclcpp::SensorDataQoS());
  outpost_target_pub_ = this->create_publisher<rm_interfaces::msg::OutpostTarget>("armor_solver/outpost_target",
                                                                                   rclcpp::SensorDataQoS());
  // Register debug publishers with outpost solver so it can publish predicted
  // formation targets and measurements for debugging.
  pka::auto_aim::setOutpostDebugPublishers(outpost_target_pub_, outpost_measure_pub_);
  gimbal_pub_ = this->create_publisher<rm_interfaces::msg::GimbalCmd>("armor_solver/cmd_gimbal",
                                                                      rclcpp::SensorDataQoS());
  pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(4),
                                       std::bind(&ArmorSolverNode::timerCallback, this));
  armor_target_.header.frame_id = "";

  enable_ = true;
  set_mode_srv_ = this->create_service<rm_interfaces::srv::SetMode>(
    "armor_solver/set_mode",
    std::bind(
      &ArmorSolverNode::setModeCallback, this, std::placeholders::_1, std::placeholders::_2));

  if (debug_mode_) {
    initMarkers();
  }

  heartbeat_ = HeartBeatPublisher::create(this);
}

void ArmorSolverNode::timerCallback()
{
  if (solver_ == nullptr) {
    return;
  }
  if (!enable_) {
    return;
  }

  // ── 前哨站单板模式：timer 仅发布缓存 cmd（EKF 在 armorsCallback 中更新）────────
  if (outpost_single_cmd_valid_) {
    // SINGLE mode timeout: if no measurement for configured seconds, clear visualization and publish zero cmd
    const auto now = this->now();
    const double elapsed_since_last = (now - last_outpost_single_time_).seconds();
    if (solver_ && solver_->outpost_solver_params_.mode == OutpostMode::SINGLE &&
        elapsed_since_last > solver_->outpost_solver_params_.single_lost_time_thres) {
      outpost_single_cmd_valid_ = false;
      // publish zero gimbal cmd and clear markers
      rm_interfaces::msg::GimbalCmd zero_cmd;
      zero_cmd.yaw_diff = 0;
      zero_cmd.pitch_diff = 0;
      zero_cmd.distance = -1;
      zero_cmd.pitch = 0;
      zero_cmd.yaw = 0;
      zero_cmd.fire_advice = false;
      gimbal_pub_->publish(zero_cmd);
      // clear visualization by emptying armor_target_
      armor_target_.header.frame_id = "";
      if (debug_mode_) publishMarkers(armor_target_, zero_cmd);
      return;
    }

    outpost_single_cmd_.header.stamp = now;
    gimbal_pub_->publish(outpost_single_cmd_);
    if (debug_mode_) {
      publishMarkers(armor_target_, outpost_single_cmd_);
    }
    return;
  }

  rm_interfaces::msg::GimbalCmd control_msg;

  if (armor_target_.header.frame_id.empty()) {
    control_msg.yaw_diff   = 0;
    control_msg.pitch_diff = 0;
    control_msg.distance   = -1;
    control_msg.pitch      = 0;
    control_msg.yaw        = 0;
    control_msg.fire_advice= false;
    gimbal_pub_->publish(control_msg);
    return;
  }

  if (armor_target_.tracking) {
    try {
      control_msg = solver_->solve(armor_target_, last_measure_msg_, this->now(), tf2_buffer_);
    } catch (...) {
      PKA_ERROR("armor_solver", "Something went wrong in solver!");
      control_msg.yaw_diff   = 0;
      control_msg.pitch_diff = 0;
      control_msg.distance   = -1;
      control_msg.fire_advice= false;
    }
  } else {
    control_msg.yaw_diff   = 0;
    control_msg.pitch_diff = 0;
    control_msg.distance   = -1;
    control_msg.fire_advice= false;
  }
  gimbal_pub_->publish(control_msg);

  if (debug_mode_) {
    publishMarkers(armor_target_, control_msg);
  }
}

void ArmorSolverNode::initMarkers() noexcept
{
  position_marker_.ns   = "position";
  position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
  position_marker_.color.a = 1.0;
  position_marker_.color.g = 1.0;
  linear_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  linear_v_marker_.ns   = "linear_v";
  linear_v_marker_.scale.x = 0.03;
  linear_v_marker_.scale.y = 0.05;
  linear_v_marker_.color.a = 1.0;
  linear_v_marker_.color.r = 1.0;
  linear_v_marker_.color.g = 1.0;
  angular_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  angular_v_marker_.ns   = "angular_v";
  angular_v_marker_.scale.x = 0.03;
  angular_v_marker_.scale.y = 0.05;
  angular_v_marker_.color.a = 1.0;
  angular_v_marker_.color.b = 1.0;
  angular_v_marker_.color.g = 1.0;
  armors_marker_.ns   = "filtered_armors";
  armors_marker_.type = visualization_msgs::msg::Marker::CUBE;
  armors_marker_.scale.x = 0.03;
  armors_marker_.scale.z = 0.125;
  armors_marker_.color.a = 1.0;
  armors_marker_.color.b = 1.0;
  selection_marker_.ns   = "selection";
  selection_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  selection_marker_.scale.x = selection_marker_.scale.y = selection_marker_.scale.z = 0.1;
  selection_marker_.color.a = 1.0;
  selection_marker_.color.g = 1.0;
  selection_marker_.color.r = 1.0;
  trajectory_marker_.ns   = "trajectory";
  trajectory_marker_.type = visualization_msgs::msg::Marker::POINTS;
  trajectory_marker_.scale.x = 0.01;
  trajectory_marker_.scale.y = 0.01;
  trajectory_marker_.color.a = 1.0;
  trajectory_marker_.color.r = 1.0;
  trajectory_marker_.color.g = 0.75;
  trajectory_marker_.color.b = 0.79;
  trajectory_marker_.points.clear();

  marker_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("armor_solver/marker", 10);
}

void ArmorSolverNode::armorsCallback(const rm_interfaces::msg::Armors::SharedPtr armors_msg)
{
  // 延迟初始化 Solver（构造函数内无法调用 weak_from_this）
  if (solver_ == nullptr) {
    solver_ = std::make_unique<Solver>(weak_from_this());
    solver_->debug_solver    = debug_solver_;
    solver_->outpost_params_ = outpost_params_;
    debug_outpost = debug_outpost_;
  }

  // 将装甲板坐标从图像坐标系转换到世界坐标系
  for (auto &armor : armors_msg->armors) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = armors_msg->header;
    ps.pose   = armor.pose;
    try {
      armor.pose = tf2_buffer_->transform(ps, target_frame_).pose;
    } catch (const tf2::TransformException &ex) {
      PKA_ERROR("armor_solver", "Transform error: {}", ex.what());
      return;
    }
  }

  // [debug_filter] 过滤前装甲板列表
  if (debug_filter_) {
    PKA_DEBUG("armor_solver",
              "[Filter::input] total armors before filter: {}", armors_msg->armors.size());
  }

  // 过滤 |z| > 2m 的异常装甲板
  armors_msg->armors.erase(
    std::remove_if(armors_msg->armors.begin(), armors_msg->armors.end(),
                   [this](const rm_interfaces::msg::Armor &armor) {
                     if (std::abs(armor.pose.position.z) > 2) {
                       if (debug_filter_) {
                         PKA_DEBUG("armor_solver",
                                   "[Filter::z_outlier] id={} z={:.3f} > 2.0, dropped",
                                   armor.number, armor.pose.position.z);
                       }
                       return true;
                     }
                     return false;
                   }),
    armors_msg->armors.end());

  // 工程不参与瞄准
  armors_msg->armors.erase(
    std::remove_if(armors_msg->armors.begin(), armors_msg->armors.end(),
                   [this](const rm_interfaces::msg::Armor &armor) {
                     if (robotTypeFromId(armor.number) == RobotType::ENGINEER_2) {
                       if (debug_filter_) {
                         PKA_DEBUG("armor_solver", "[Filter::engineer] id={} dropped", armor.number);
                       }
                       return true;
                     }
                     return false;
                   }),
    armors_msg->armors.end());

  // 过滤前哨站顶部装甲板
  armors_msg->armors.erase(
    std::remove_if(armors_msg->armors.begin(), armors_msg->armors.end(),
                   [this](const rm_interfaces::msg::Armor &armor) {
                    const auto &q = armor.pose.orientation;
                    tf2::Quaternion tf2_q;
                    // 从 geometry_msgs::msg::Quaternion 转到 tf2::Quaternion
                    tf2::fromMsg(q, tf2_q);  // 或者直接用构造函数: tf2_q(q.x, q.y, q.z, q.w)

                    // 2. 转换为 RPY
                    double roll, pitch, yaw;
                    tf2::Matrix3x3(tf2_q).getRPY(roll, pitch, yaw);
                    const double max_pitch = 73.0 * M_PI / 180.0;
                     if (std::abs(pitch) > max_pitch) {
                       if (debug_filter_) {
                         PKA_DEBUG("armor_solver", "[Filter::OutpostTop] id={} dropped", armor.number);
                       }
                       return true;
                     }
                     return false;
                   }),
    armors_msg->armors.end());

  // 过滤基地顶部装甲板
  armors_msg->armors.erase(
    std::remove_if(armors_msg->armors.begin(), armors_msg->armors.end(),
                   [this](const rm_interfaces::msg::Armor &armor) {
                     if (robotTypeFromId(armor.number) == RobotType::BASE &&
                        armor.type == "small") {
                       if (debug_filter_) {
                         PKA_DEBUG("armor_solver", "[Filter::BaseTop] id={} dropped", armor.number);
                       }
                       return true;
                     }
                     return false;
                   }),
    armors_msg->armors.end());

  const auto max_dist_for_armor = [this](const rm_interfaces::msg::Armor &armor) -> double {
    switch (robotTypeFromId(armor.number)) {
      case RobotType::OUTPOST:
        return max_outpost_armor_distance_;
      case RobotType::BASE:
        return max_base_armor_distance_;
      default:
        return max_armor_distance_;
    }
  };

  // 按目标类型过滤超过最大跟踪距离的装甲板
  armors_msg->armors.erase(
    std::remove_if(armors_msg->armors.begin(), armors_msg->armors.end(),
                   [this, &max_dist_for_armor](const rm_interfaces::msg::Armor &armor) {
                     const auto &p = armor.pose.position;
                     const double dist = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
                     const double lim = max_dist_for_armor(armor);
                     if (dist > lim) {
                       if (debug_filter_) {
                         PKA_DEBUG("armor_solver",
                                   "[Filter::max_dist] id={} dist={:.3f}m > max={:.3f}m, dropped",
                                   armor.number, dist, lim);
                       }
                       return true;
                     }
                     return false;
                   }),
    armors_msg->armors.end());

  // Init message
  rm_interfaces::msg::Measurement measure_msg;
  rm_interfaces::msg::Target      target_msg;
  rclcpp::Time time       = armors_msg->header.stamp;
  target_msg.header.stamp    = time;
  target_msg.header.frame_id = target_frame_;

  // ── 前哨站单板模式（SINGLE）：4D EKF；有测量时 update，无测量时 predict，保持 target/cmd 连续 ──
  if (solver_ != nullptr) {
    solver_->outpost_solver_params_ = loadOutpostSolverParams(*this);
    if (auto * tc = solver_->outpostTrajectoryCompensator()) {
      auto & p = solver_->outpost_solver_params_;
      tc->velocity   = p.bullet_speed;
      tc->gravity    = p.gravity;
      tc->resistance = p.resistance;
    }

    if (solver_->outpost_solver_params_.mode != OutpostMode::SINGLE) {
      resetOutpostSinglePlateRuntime(outpost_single_rt_);
      outpost_single_cmd_valid_ = false;
    } else {
      double dt_single = 0.005;
      if (last_time_.nanoseconds() > 0) {
        dt_single = (time - last_time_).seconds();
      }
      if (dt_single <= 0.0 || dt_single > 0.5) {
        dt_single = 0.005;
      }

      outpost_single_cmd_.header.stamp    = time;
      outpost_single_cmd_.header.frame_id = target_frame_;
      outpost_single_cmd_valid_ = outpostSinglePlateSolve(
        armors_msg->armors, dt_single, target_frame_, tf2_buffer_,
        solver_->outpost_solver_params_, outpost_params_,
        solver_->outpostTrajectoryCompensator(),
        solver_->outpostManualCompensator(),
        debug_outpost_, &outpost_single_rt_,
        &outpost_single_cmd_, &target_msg);
      if (outpost_single_cmd_valid_) {
        armor_target_ = target_msg;
        target_pub_->publish(target_msg);
        // update last single measurement time for timeout handling
        last_outpost_single_time_ = time;
        last_time_ = time;
        return;
      }
    }
  }

  // Update tracker
  if (tracker_->tracker_state == Tracker::LOST) {
    if (armors_msg->armors.empty()) {
      target_msg.tracking = false;
      armor_target_ = target_msg;
      target_pub_->publish(target_msg);
      last_time_ = time;
      return;
    }

    // 根据目标类型选择阈值
    const bool msg_all_outpost =
      !armors_msg->armors.empty() &&
      std::all_of(armors_msg->armors.begin(), armors_msg->armors.end(),
                  [this](const rm_interfaces::msg::Armor &x) {
                    return isOutpostId(x.number, outpost_params_.id);
                  });
    tracker_->tracking_thres = msg_all_outpost ? outpost_tracking_thres_
                                               : this->get_parameter("tracker.tracking_thres").as_int();
    lost_time_thres_ = msg_all_outpost ? outpost_lost_time_thres_
                                       : this->get_parameter("tracker.lost_time_thres").as_double();

    tracker_->init(armors_msg);
    target_msg.tracking = false;
  } else {
    dt_ = (time - last_time_).seconds();

    // 前哨站使用独立的 tracking_thres 和 lost_time_thres
    const bool is_outpost_tracking =
      isOutpostId(tracker_->tracked_id, outpost_params_.id);
    if (is_outpost_tracking) {
      tracker_->tracking_thres = outpost_tracking_thres_;
      lost_time_thres_         = outpost_lost_time_thres_;
    }
    tracker_->lost_thres = std::abs(static_cast<int>(lost_time_thres_ / dt_));

    // 前哨站切换为 CONSTANT_ROTATION，地面兵种使用 CONSTANT_VEL_ROT
    const MotionModel motion_model = is_outpost_tracking
      ? MotionModel::CONSTANT_ROTATION
      : MotionModel::CONSTANT_VEL_ROT;
    tracker_->setEKFDt(dt_, motion_model);

    if (debug_tracker_) {
      PKA_DEBUG("armor_solver",
                "[Node::callback] dt={:.4f}s lost_thres={} tracked_id={} is_outpost={}",
                dt_, tracker_->lost_thres, tracker_->tracked_id, is_outpost_tracking);
    }

    tracker_->update(armors_msg);

    // 发布 measurement
    measure_msg.x   = tracker_->measurement(0);
    measure_msg.y   = tracker_->measurement(1);
    measure_msg.z   = tracker_->measurement(2);
    measure_msg.yaw = tracker_->measurement(3);
    measure_pub_->publish(measure_msg);
    // save last measurement for passing into outpostSolve
    last_measure_msg_ = measure_msg;
    // publish outpost-specific measurement debug as well
    rm_interfaces::msg::OutpostMeasurement om;
    om.header.stamp = time;
    om.header.frame_id = target_frame_;
    om.x = measure_msg.x; om.y = measure_msg.y; om.z = measure_msg.z; om.yaw = measure_msg.yaw;
    outpost_measure_pub_->publish(om);

    if (tracker_->tracker_state == Tracker::DETECTING) {
      target_msg.tracking = false;
    } else if (tracker_->tracker_state == Tracker::TRACKING ||
               tracker_->tracker_state == Tracker::TEMP_LOST) {
      target_msg.tracking = true;
      const auto &state   = tracker_->target_state;
      target_msg.id       = tracker_->tracked_id;
      target_msg.armors_num = static_cast<int>(tracker_->tracked_armors_num);

      // 统一使用 10-D EKF 状态填充 target_msg（地面兵种和前哨站共用）
      target_msg.position.x  = state(0);
      target_msg.velocity.x  = state(1);
      target_msg.position.y  = state(2);
      target_msg.velocity.y  = state(3);
      target_msg.position.z  = state(4);
      target_msg.velocity.z  = state(5);
      target_msg.yaw         = state(6);
      target_msg.v_yaw       = state(7);
      target_msg.radius_1    = state(8);
      target_msg.radius_2    = tracker_->another_r;
      target_msg.d_zc        = state(9);
      target_msg.d_za        = tracker_->d_za;

      if (!is_outpost_tracking) {
        // 地面兵种：tracked_armor_x/y/z 从 EKF 状态计算（用于 solver 弹道延迟预测）
        target_msg.tracked_armor_x = state(0) - state(8) * cos(state(6));
        target_msg.tracked_armor_y = state(2) - state(8) * sin(state(6));
        target_msg.tracked_armor_z = state(4) + state(9);
      } else {
        // 前哨站：tracked_armor_x/y/z 直接使用原始观测坐标（无预测延时）
        // solver 用此值计算 pitch，不叠加 prediction_delay 和 controller_delay
        outpostFillRawArmorPosition(tracker_->tracked_armor, &target_msg);
      }

      if (debug_tracker_) {
        PKA_DEBUG("armor_solver",
                  "[Node::target] id={} {} "
                  "center=({:.3f},{:.3f},{:.3f}) vel=({:.3f},{:.3f},{:.3f}) "
                  "yaw={:.4f} v_yaw={:.4f} r1={:.3f} d_zc={:.3f}",
                  target_msg.id,
                  (tracker_->tracker_state == Tracker::TRACKING ? "TRACKING" : "TEMP_LOST"),
                  state(0), state(2), state(4),
                  state(1), state(3), state(5),
                  state(6), state(7), state(8), state(9));
      }
    }
  }

  armor_target_ = target_msg;
  target_pub_->publish(target_msg);
  last_time_ = time;
}

void ArmorSolverNode::publishMarkers(const rm_interfaces::msg::Target &target_msg,
                                     const rm_interfaces::msg::GimbalCmd &gimbal_cmd) noexcept {
  position_marker_.header  = target_msg.header;
  linear_v_marker_.header  = target_msg.header;
  angular_v_marker_.header = target_msg.header;
  armors_marker_.header    = target_msg.header;
  selection_marker_.header = target_msg.header;
  trajectory_marker_.header= target_msg.header;

  visualization_msgs::msg::MarkerArray marker_array;
  static std::size_t last_armors_marker_count = 0;

  if (target_msg.tracking) {
    double yaw = target_msg.yaw, r1 = target_msg.radius_1, r2 = target_msg.radius_2;
    double xc = target_msg.position.x, yc = target_msg.position.y, zc = target_msg.position.z;
    double vx = target_msg.velocity.x, vy = target_msg.velocity.y, vz = target_msg.velocity.z;
    double d_za = target_msg.d_za, d_zc = target_msg.d_zc;

    position_marker_.action = visualization_msgs::msg::Marker::ADD;
    position_marker_.pose.position.x = xc;
    position_marker_.pose.position.y = yc;
    position_marker_.pose.position.z = zc;

    linear_v_marker_.action = visualization_msgs::msg::Marker::ADD;
    linear_v_marker_.points.clear();
    linear_v_marker_.points.emplace_back(position_marker_.pose.position);
    geometry_msgs::msg::Point arrow_end = position_marker_.pose.position;
    arrow_end.x += vx;
    arrow_end.y += vy;
    arrow_end.z += vz;
    linear_v_marker_.points.emplace_back(arrow_end);

    angular_v_marker_.action = visualization_msgs::msg::Marker::ADD;
    angular_v_marker_.points.clear();
    angular_v_marker_.points.emplace_back(position_marker_.pose.position);
    arrow_end = position_marker_.pose.position;
    arrow_end.z += target_msg.v_yaw / M_PI;
    angular_v_marker_.points.emplace_back(arrow_end);

    const size_t a_n = target_msg.armors_num;
    armors_marker_.action = (a_n > 0) ?
      visualization_msgs::msg::Marker::ADD :
      visualization_msgs::msg::Marker::DELETE;
    armors_marker_.scale.y =
      isOutpostId(target_msg.id, outpost_params_.id) ? 0.135 :
      (tracker_->tracked_armor.type == "small" ? 0.135 : 0.23);

    bool is_current_pair = true;
    geometry_msgs::msg::Point p_a;
    double r = 0;
    if (a_n == 3) {
      appendOutpostFilteredArmorsStripMarkers(target_msg,
                                              outpost_params_,
                                              target_msg.header.frame_id,
                                              target_msg.header.stamp,
                                              &marker_array,
                                              &armors_marker_);
    } else {
      for (size_t i = 0; i < a_n; i++) {
        const double tmp_yaw = yaw + static_cast<double>(i) * (2 * M_PI / static_cast<double>(a_n));
        if (a_n == 4) {
          r = is_current_pair ? r1 : r2;
          p_a.z = zc + d_zc + (is_current_pair ? 0 : d_za);
          is_current_pair = !is_current_pair;
        } else {
          r = r1;
          p_a.z = zc;
        }
        p_a.x = xc - r * cos(tmp_yaw);
        p_a.y = yc - r * sin(tmp_yaw);

        armors_marker_.id = static_cast<int>(i);
        armors_marker_.pose.position = p_a;
        tf2::Quaternion q;
        const double pitch = isOutpostId(target_msg.id, outpost_params_.id)
          ? outpost_params_.plate_pitch_rad : 0.2618;
        q.setRPY(0.0, pitch, tmp_yaw);
        armors_marker_.pose.orientation = tf2::toMsg(q);
        marker_array.markers.emplace_back(armors_marker_);
      }
    }

    if (last_armors_marker_count > a_n) {
      for (std::size_t i = a_n; i < last_armors_marker_count; i++) {
        visualization_msgs::msg::Marker del = armors_marker_;
        del.action = visualization_msgs::msg::Marker::DELETE;
        del.id = static_cast<int>(i);
        marker_array.markers.emplace_back(del);
      }
    }
    last_armors_marker_count = a_n;

    selection_marker_.action = visualization_msgs::msg::Marker::ADD;
    selection_marker_.points.clear();
    selection_marker_.pose.position.y = gimbal_cmd.distance * sin(gimbal_cmd.yaw   * M_PI / 180.0);
    selection_marker_.pose.position.x = gimbal_cmd.distance * cos(gimbal_cmd.yaw   * M_PI / 180.0);
    selection_marker_.pose.position.z = gimbal_cmd.distance * sin(gimbal_cmd.pitch * M_PI / 180.0);

    trajectory_marker_.action = visualization_msgs::msg::Marker::ADD;
    trajectory_marker_.points.clear();
    trajectory_marker_.header.frame_id = "gimbal_link";
    for (const auto &point : solver_->getTrajectory()) {
      geometry_msgs::msg::Point p;
      p.x = point.first;
      p.z = point.second;
      trajectory_marker_.points.emplace_back(p);
    }
    if (gimbal_cmd.fire_advice) {
      trajectory_marker_.color.r = 0;
      trajectory_marker_.color.g = 1;
      trajectory_marker_.color.b = 0;
    } else {
      trajectory_marker_.color.r = 1;
      trajectory_marker_.color.g = 1;
      trajectory_marker_.color.b = 1;
    }
  } else {
    position_marker_.action  = visualization_msgs::msg::Marker::DELETE;
    linear_v_marker_.action  = visualization_msgs::msg::Marker::DELETE;
    angular_v_marker_.action = visualization_msgs::msg::Marker::DELETE;
    armors_marker_.action    = visualization_msgs::msg::Marker::DELETE;
    trajectory_marker_.action= visualization_msgs::msg::Marker::DELETE;
    selection_marker_.action = visualization_msgs::msg::Marker::DELETE;
    for (std::size_t i = 0; i < last_armors_marker_count; i++) {
      visualization_msgs::msg::Marker del = armors_marker_;
      del.action = visualization_msgs::msg::Marker::DELETE;
      del.id = static_cast<int>(i);
      marker_array.markers.emplace_back(del);
    }
    last_armors_marker_count = 0;
  }

  marker_array.markers.emplace_back(position_marker_);
  marker_array.markers.emplace_back(trajectory_marker_);
  marker_array.markers.emplace_back(linear_v_marker_);
  marker_array.markers.emplace_back(angular_v_marker_);
  marker_array.markers.emplace_back(armors_marker_);
  marker_array.markers.emplace_back(selection_marker_);
  marker_pub_->publish(marker_array);
}

void ArmorSolverNode::setModeCallback(
  const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
  std::shared_ptr<rm_interfaces::srv::SetMode::Response> response) {
  response->success = true;

  VisionMode mode = static_cast<VisionMode>(request->mode);
  std::string mode_name = visionModeToString(mode);
  if (mode_name == "UNKNOWN") {
    PKA_ERROR("armor_solver", "Invalid mode: {}", request->mode);
    return;
  }

  switch (mode) {
    case VisionMode::AUTO_AIM_RED:
    case VisionMode::AUTO_AIM_BLUE: {
      enable_ = true;
      break;
    }
    default: {
      enable_ = false;
      break;
    }
  }

  PKA_WARN("armor_solver", "Set Mode to {}", visionModeToString(mode));
}

}  // namespace pka::auto_aim

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pka::auto_aim::ArmorSolverNode)
