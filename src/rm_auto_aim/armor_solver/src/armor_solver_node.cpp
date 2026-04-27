// Copyright Chen Jun 2023. Licensed under the MIT License.
//
// Additional modifications and features by Chengfu Zou, Labor. Licensed under Apache License 2.0.
//
// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
  // Register logger（注册节点）
  PKA_INFO("armor_solver", "Starting ArmorSolverNode!");

  // 是否为调试模式（rviz 可视化 marker）
  debug_mode_ = this->declare_parameter("debug", true);

  // -------------------------------------------------------
  // 分组调试开关（各组独立控制，均默认关闭）
  // debug_tracker: tracker 匹配、状态机转移、装甲板跳跃日志
  // debug_ekf:     EKF 测量/预测/更新状态向量、速度限幅对比
  // debug_solver:  弹道预测、装甲板选择、开火判断、yaw/pitch 解算
  // debug_filter:  距离/高度过滤日志
  // debug_outpost: 前哨站 outpost_solver 内 PKA_DEBUG 总开关
  // -------------------------------------------------------
  debug_tracker_ = this->declare_parameter("debug_tracker", false);
  debug_ekf_     = this->declare_parameter("debug_ekf",     false);
  debug_solver_  = this->declare_parameter("debug_solver",  false);
  debug_filter_  = this->declare_parameter("debug_filter",  false);
  debug_outpost_ = this->declare_parameter("debug_outpost", false);
  debug_outpost  = debug_outpost_;

  PKA_INFO("armor_solver",
           "Debug flags: debug={} tracker={} ekf={} solver={} filter={} outpost={}",
           debug_mode_, debug_tracker_, debug_ekf_, debug_solver_, debug_filter_, debug_outpost_);

  // 最大跟踪距离：超过此距离的装甲板直接丢弃，不进行 tracker 和 solver 处理
  max_armor_distance_ = this->declare_parameter("max_armor_distance", 5.8);

  // ---------- 前哨站参数（声明与读取集中在 outpost_solver） ----------
  declareOutpostParameters(*this);
  outpost_params_ = loadOutpostParams(*this);
  outpost_mode_   = loadOutpostMode(*this);

  // Tracker
  double max_match_distance = this->declare_parameter("tracker.max_match_distance", 0.2);
  double max_match_yaw_diff = this->declare_parameter("tracker.max_match_yaw_diff", 1.0);
  // Outpost Tracker gates (separated from ground robots)
  double outpost_max_match_distance =
    this->declare_parameter("outpost.tracker.max_match_distance", max_match_distance);
  double outpost_max_match_yaw_diff =
    this->declare_parameter("outpost.tracker.max_match_yaw_diff", max_match_yaw_diff);
  // 创建跟踪器
  tracker_ = std::make_unique<Tracker>(max_match_distance, max_match_yaw_diff);
  tracker_->setOutpostTrackerGates(outpost_max_match_distance, outpost_max_match_yaw_diff);
  // 判断为跟踪的阈值
  tracker_->tracking_thres = this->declare_parameter("tracker.tracking_thres", 5);
  // 判断为丢失目标的阈值（度量为丢帧时间）
  lost_time_thres_ = this->declare_parameter("tracker.lost_time_thres", 0.3);

  // Outpost tracker thres (optional separated)
  outpost_tracking_thres_ =
    this->declare_parameter("outpost.tracker.tracking_thres", tracker_->tracking_thres);
  outpost_lost_time_thres_ =
    this->declare_parameter("outpost.tracker.lost_time_thres", lost_time_thres_);

  // -------------------------------------------------------
  // 速度限幅（Velocity Clamp）参数
  // tracker 从 LOST 重新识别后，前 vel_clamp_frames 帧内对 EKF update()
  // 输出的速度分量做硬限幅，防止初始化瞬间异常速度冲量导致云台抽风。
  // -------------------------------------------------------
  tracker_->vel_clamp_frames      = this->declare_parameter("vel_clamp.frames",      5);
  tracker_->vel_clamp_linear_max  = this->declare_parameter("vel_clamp.linear_max",  3.0);
  tracker_->vel_clamp_yaw_max     = this->declare_parameter("vel_clamp.yaw_max",     4.0);

  // EKF 状态钳位（Ground）
  tracker_->ground_state_clamp_enable =
    this->declare_parameter("ekf.state_clamp.enable", true);
  tracker_->ground_radius_min =
    this->declare_parameter("ekf.state_clamp.radius_min", 0.18);
  tracker_->ground_radius_max =
    this->declare_parameter("ekf.state_clamp.radius_max", 0.35);
  tracker_->ground_d_zc_min =
    this->declare_parameter("ekf.state_clamp.d_zc_min", -0.35);
  tracker_->ground_d_zc_max =
    this->declare_parameter("ekf.state_clamp.d_zc_max", 0.35);
  tracker_->ground_v_yaw_min =
    this->declare_parameter("ekf.state_clamp.v_yaw_min", -20.0);
  tracker_->ground_v_yaw_max =
    this->declare_parameter("ekf.state_clamp.v_yaw_max", 20.0);

  // EKF 状态钳位（Outpost）
  tracker_->outpost_state_clamp_enable =
    this->declare_parameter("outpost.ekf.state_clamp.enable", true);
  tracker_->outpost_radius_min =
    this->declare_parameter("outpost.ekf.state_clamp.radius_min", 0.18);
  tracker_->outpost_radius_max =
    this->declare_parameter("outpost.ekf.state_clamp.radius_max", 0.40);
  tracker_->outpost_d_zc_min =
    this->declare_parameter("outpost.ekf.state_clamp.d_zc_min", -0.35);
  tracker_->outpost_d_zc_max =
    this->declare_parameter("outpost.ekf.state_clamp.d_zc_max", 0.35);
  tracker_->outpost_d_za_min =
    this->declare_parameter("outpost.ekf.state_clamp.d_za_min", 0.03);
  tracker_->outpost_d_za_max =
    this->declare_parameter("outpost.ekf.state_clamp.d_za_max", 0.30);
  tracker_->outpost_v_yaw_min =
    this->declare_parameter("outpost.ekf.state_clamp.v_yaw_min", -20.0);
  tracker_->outpost_v_yaw_max =
    this->declare_parameter("outpost.ekf.state_clamp.v_yaw_max", 20.0);
  tracker_->outpost_v_xyz_max =
    this->declare_parameter("outpost.ekf.state_clamp.v_xyz_max", 0.20);

  // 前哨站运动模型（默认 CONSTANT_ROTATION：抑制静止旋转目标的中心线速度漂移）
  outpost_motion_model_ = this->declare_parameter("outpost.ekf.motion_model", 1);
  outpost_motion_model_ = std::clamp(outpost_motion_model_, 0, 2);

  // 将分组调试开关注入 tracker
  tracker_->debug_tracker = debug_tracker_;
  tracker_->debug_ekf     = debug_ekf_;
  tracker_->outpost_cfg_  = outpost_params_;

  // EKF
  // xa = x_armor, xc = x_robot_center
  // 状态：xc, v_xc, yc, v_yc, zc, v_zc, yaw, v_yaw, r, d_zc
  // 测量：p, y, d, yaw
  auto f = Predict(0.005);
  auto h = Measure();
  // 过程噪声协方差矩阵
  s2qx_   = declare_parameter("ekf.sigma2_q_x",   20.0);
  s2qy_   = declare_parameter("ekf.sigma2_q_y",   20.0);
  s2qz_   = declare_parameter("ekf.sigma2_q_z",   20.0);
  s2qyaw_ = declare_parameter("ekf.sigma2_q_yaw", 100.0);
  s2qr_   = declare_parameter("ekf.sigma2_q_r",   800.0);
  s2qd_zc_= declare_parameter("ekf.sigma2_q_d_zc",800.0);

  // 前哨站 EKF 过程噪声（与地面兵种分开）
  outpost_s2qx_   = declare_parameter("outpost.ekf.sigma2_q_x",   s2qx_);
  outpost_s2qy_   = declare_parameter("outpost.ekf.sigma2_q_y",   s2qy_);
  outpost_s2qz_   = declare_parameter("outpost.ekf.sigma2_q_z",   s2qz_);
  outpost_s2qyaw_ = declare_parameter("outpost.ekf.sigma2_q_yaw", s2qyaw_);
  outpost_s2qr_   = declare_parameter("outpost.ekf.sigma2_q_r",   s2qr_);
  // sigma2_q_d_zc / d_za 在新的 6-D 前哨站模型中不再使用（d_zc/d_za 已从状态中移除）。
  // 保留 declare_parameter 以兼容旧 yaml 配置，读取后丢弃。
  (void)declare_parameter("outpost.ekf.sigma2_q_d_zc", s2qd_zc_);
  (void)declare_parameter("outpost.ekf.sigma2_q_d_za", 50.0);

  // ── 地面兵种 EKF（10-D，与原始完全一致，不含 d_za）──────────────────────
  auto u_q = [this]() 
  {
    Eigen::Matrix<double, X_N, X_N> q;
    double t = dt_, x = s2qx_, y = s2qy_, z = s2qz_, yaw = s2qyaw_, r = s2qr_;
    double d_zc = s2qd_zc_;
    double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx = pow(t, 2) * x;
    double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * y, q_vy_vy = pow(t, 2) * y;
    double q_z_z = pow(t, 4) / 4 * x, q_z_vz = pow(t, 3) / 2 * x, q_vz_vz = pow(t, 2) * z;
    double q_yaw_yaw   = pow(t, 4) / 4 * yaw, q_yaw_vyaw = pow(t, 3) / 2 * x,
           q_vyaw_vyaw = pow(t, 2) * yaw;
    double q_r    = pow(t, 4) / 4 * r;
    double q_d_zc = pow(t, 4) / 4 * d_zc;
    // clang-format off
    //    xc      v_xc    yc      v_yc    zc      v_zc    yaw         v_yaw       r       d_zc
    q <<  q_x_x,  q_x_vx, 0,      0,      0,      0,      0,          0,          0,      0,
          q_x_vx, q_vx_vx,0,      0,      0,      0,      0,          0,          0,      0,
          0,      0,      q_y_y,  q_y_vy, 0,      0,      0,          0,          0,      0,
          0,      0,      q_y_vy, q_vy_vy,0,      0,      0,          0,          0,      0,
          0,      0,      0,      0,      q_z_z,  q_z_vz, 0,          0,          0,      0,
          0,      0,      0,      0,      q_z_vz, q_vz_vz,0,          0,          0,      0,
          0,      0,      0,      0,      0,      0,      q_yaw_yaw,  q_yaw_vyaw, 0,      0,
          0,      0,      0,      0,      0,      0,      q_yaw_vyaw, q_vyaw_vyaw,0,      0,
          0,      0,      0,      0,      0,      0,      0,          0,          q_r,    0,
          0,      0,      0,      0,      0,      0,      0,          0,          0,      q_d_zc;
    // clang-format on
    return q;
  };

  // 测量噪声协方差矩阵（地面兵种和前哨站共用同一组 R 参数）
  r_x_   = declare_parameter("ekf.r_x",   0.05);
  r_y_   = declare_parameter("ekf.r_y",   0.05);
  r_z_   = declare_parameter("ekf.r_z",   0.05);
  r_yaw_ = declare_parameter("ekf.r_yaw", 0.02);

  // 前哨站 EKF 测量噪声（与地面兵种分开）
  outpost_r_x_   = declare_parameter("outpost.ekf.r_x",   r_x_);
  outpost_r_y_   = declare_parameter("outpost.ekf.r_y",   r_y_);
  outpost_r_z_   = declare_parameter("outpost.ekf.r_z",   r_z_);
  outpost_r_yaw_ = declare_parameter("outpost.ekf.r_yaw", r_yaw_);
  auto u_r = [this](const Eigen::Matrix<double, Z_N, 1> &z) 
  {
    Eigen::Matrix<double, Z_N, Z_N> r;
    // clang-format off
    r << r_x_ * std::abs(z[0]), 0, 0, 0,
         0, r_y_ * std::abs(z[1]), 0, 0,
         0, 0, r_z_ * std::abs(z[2]), 0,
         0, 0, 0, r_yaw_;
    // clang-format on
    return r;
  };

  // 误差估计协方差矩阵（地面兵种）
  Eigen::DiagonalMatrix<double, X_N> p0;
  p0.setIdentity();
  tracker_->ekf = std::make_unique<RobotStateEKF>(f, h, u_q, u_r, p0);

  // ── 前哨站专用 EKF（6-D，纯旋转模型，与地面兵种完全独立）───────────────
  // 状态：[xc, yc, zc, yaw, v_yaw, r]
  // 预测：CONSTANT_ROTATION（中心固定，yaw += v_yaw * dt）
  // 观测：[xa_slot0, ya_slot0, za_raw, yaw_slot0]（slot-0 等效，由 outpostPrepareEKFMeasurement 处理）
  auto outpost_u_q = [this]()
  {
    Eigen::Matrix<double, X_N_OUTPOST, X_N_OUTPOST> q;
    const double t    = dt_;
    const double sx   = outpost_s2qx_;
    const double sy   = outpost_s2qy_;
    const double sz   = outpost_s2qz_;
    const double syaw = outpost_s2qyaw_;
    const double sr   = outpost_s2qr_;
    // 中心位置（constant model，无速度耦合）：简单随机游走噪声
    const double q_xc  = sx  * pow(t, 2);
    const double q_yc  = sy  * pow(t, 2);
    const double q_zc  = sz  * pow(t, 2);
    // r（constant model）：随机游走噪声
    const double q_r   = sr  * pow(t, 2);
    // yaw + v_yaw（CWNA 模型，匀角加速度近似）
    const double q_yaw_yaw   = pow(t, 4) / 4.0 * syaw;
    const double q_yaw_vyaw  = pow(t, 3) / 2.0 * syaw;
    const double q_vyaw_vyaw = pow(t, 2)        * syaw;
    // clang-format off
    //    xc     yc     zc     yaw            v_yaw           r
    q <<  q_xc,  0,     0,     0,             0,              0,
          0,     q_yc,  0,     0,             0,              0,
          0,     0,     q_zc,  0,             0,              0,
          0,     0,     0,     q_yaw_yaw,     q_yaw_vyaw,     0,
          0,     0,     0,     q_yaw_vyaw,    q_vyaw_vyaw,    0,
          0,     0,     0,     0,             0,              q_r;
    // clang-format on
    return q;
  };

  auto outpost_u_r = [this](const Eigen::Matrix<double, Z_N, 1> &z)
  {
    Eigen::Matrix<double, Z_N, Z_N> r;
    // r_z 较大，吸收三块板 z 高度差异（各板高度不同但模型用均值 zc）
    // clang-format off
    r << outpost_r_x_ * std::abs(z[0]), 0,                              0,                              0,
         0,                              outpost_r_y_ * std::abs(z[1]), 0,                              0,
         0,                              0,                              outpost_r_z_ * std::abs(z[2]), 0,
         0,                              0,                              0,                              outpost_r_yaw_;
    // clang-format on
    return r;
  };

  Eigen::DiagonalMatrix<double, X_N_OUTPOST> outpost_p0;
  outpost_p0.setIdentity();
  OutpostPredict outpost_f(0.005);
  OutpostMeasure outpost_h;
  tracker_->outpost_ekf = std::make_unique<OutpostStateEKF>(
    outpost_f, outpost_h, outpost_u_q, outpost_u_r, outpost_p0);

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

  // Measurement publisher (for debug usage)
  measure_pub_ = this->create_publisher<rm_interfaces::msg::Measurement>("armor_solver/measurement",
                                                                         rclcpp::SensorDataQoS());

  // Publisher
  target_pub_ = this->create_publisher<rm_interfaces::msg::Target>("armor_solver/target",
                                                                   rclcpp::SensorDataQoS());
  gimbal_pub_ = this->create_publisher<rm_interfaces::msg::GimbalCmd>("armor_solver/cmd_gimbal",
                                                                      rclcpp::SensorDataQoS());
  // Timer 250 Hz
  pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(4),
                                       std::bind(&ArmorSolverNode::timerCallback, this));
  armor_target_.header.frame_id = "";

  // Enable/Disable Armor Solver
  enable_ = true;
  set_mode_srv_ = this->create_service<rm_interfaces::srv::SetMode>(
    "armor_solver/set_mode",
    std::bind(
      &ArmorSolverNode::setModeCallback, this, std::placeholders::_1, std::placeholders::_2));

  if (debug_mode_) 
  {
    initMarkers();
  }

  // Heartbeat
  heartbeat_ = HeartBeatPublisher::create(this);
}

void ArmorSolverNode::timerCallback() 
{
  if (solver_ == nullptr) 
  {
    return;
  }

  if (!enable_) 
  {
    return;
  }

  // Init message
  rm_interfaces::msg::GimbalCmd control_msg;

  // If target never detected
  if (armor_target_.header.frame_id.empty()) 
  {
    control_msg.yaw_diff   = 0;
    control_msg.pitch_diff = 0;
    control_msg.distance   = -1;
    control_msg.pitch      = 0;
    control_msg.yaw        = 0;
    control_msg.fire_advice= false;
    gimbal_pub_->publish(control_msg);
    return;
  }

  // 如果为跟踪状态
  if (armor_target_.tracking) 
  {
    try 
    {
      control_msg = solver_->solve(armor_target_, this->now(), tf2_buffer_);
    } 
    catch (...) 
    {
      PKA_ERROR("armor_solver", "Something went wrong in solver!");
      control_msg.yaw_diff   = 0;
      control_msg.pitch_diff = 0;
      control_msg.distance   = -1;
      control_msg.fire_advice= false;
    }
  } 
  else 
  {
    control_msg.yaw_diff   = 0;
    control_msg.pitch_diff = 0;
    control_msg.distance   = -1;
    control_msg.fire_advice= false;
  }
  gimbal_pub_->publish(control_msg);

  if (debug_mode_) 
  {
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
  // Lazy initialize solver owing to weak_from_this() can't be called in constructor
  if (solver_ == nullptr) 
  {
    solver_ = std::make_unique<Solver>(weak_from_this());
    solver_->debug_solver   = debug_solver_;
    solver_->outpost_params_ = outpost_params_;
    debug_outpost = debug_outpost_;
  }

  // Tranform armor position from image frame to world coordinate
  for (auto &armor : armors_msg->armors) 
  {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = armors_msg->header;
    ps.pose   = armor.pose;
    try 
    {
      armor.pose = tf2_buffer_->transform(ps, target_frame_).pose;
    } 
    catch (const tf2::TransformException &ex) 
    {
      PKA_ERROR("armor_solver", "Transform error: {}", ex.what());
      return;
    }
  }

  // [debug_filter] 输入装甲板列表（过滤前）
  if (debug_filter_) {
    PKA_DEBUG("armor_solver",
              "[Filter::input] total armors before filter: {}", armors_msg->armors.size());
    for (const auto &armor : armors_msg->armors) {
      const auto &p = armor.pose.position;
      double dist = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
      PKA_DEBUG("armor_solver",
                "[Filter::input]  id={} pos=({:.3f},{:.3f},{:.3f}) dist={:.3f}m",
                armor.number, p.x, p.y, p.z, dist);
    }
  }

  // Filter abnormal armors (|z| > 2m)
  size_t before_z = armors_msg->armors.size();
  armors_msg->armors.erase(
    std::remove_if(armors_msg->armors.begin(), armors_msg->armors.end(),
                   [this](const rm_interfaces::msg::Armor &armor) {
                     if (std::abs(armor.pose.position.z) > 2) {
                       // [debug_filter] 高度异常过滤
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

  if (debug_filter_ && armors_msg->armors.size() < before_z) {
    PKA_DEBUG("armor_solver",
              "[Filter::z_outlier] dropped {} armors by z>2m filter",
              before_z - armors_msg->armors.size());
  }

  // Filter armors that exceed max_armor_distance
  // 距离超过阈值的装甲板直接丢弃，tracker 和 solver 完全不感知
  size_t before_dist = armors_msg->armors.size();
  armors_msg->armors.erase(
    std::remove_if(armors_msg->armors.begin(), armors_msg->armors.end(),
                   [this](const rm_interfaces::msg::Armor &armor) {
                     const auto &p = armor.pose.position;
                     double dist = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
                     if (dist > max_armor_distance_) {
                       // [debug_filter] 距离过远过滤
                       if (debug_filter_) {
                         PKA_DEBUG("armor_solver",
                                   "[Filter::max_dist] id={} dist={:.3f}m > max={:.3f}m, dropped",
                                   armor.number, dist, max_armor_distance_);
                       }
                       return true;
                     }
                     return false;
                   }),
    armors_msg->armors.end());

  if (debug_filter_ && armors_msg->armors.size() < before_dist) {
    PKA_DEBUG("armor_solver",
              "[Filter::max_dist] dropped {} armors by distance filter, {} remaining",
              before_dist - armors_msg->armors.size(), armors_msg->armors.size());
  }

  // Init message
  rm_interfaces::msg::Measurement measure_msg;
  rm_interfaces::msg::Target target_msg;
  rclcpp::Time time = armors_msg->header.stamp;
  target_msg.header.stamp    = time;
  target_msg.header.frame_id = target_frame_;

  // Update tracker
  if (tracker_->tracker_state == Tracker::LOST) 
  {
    if (armors_msg->armors.empty()) {
      target_msg.header.stamp    = time;
      target_msg.header.frame_id = target_frame_;
      target_msg.tracking        = false;
      armor_target_              = target_msg;
      target_pub_->publish(target_msg);
      last_time_ = time;
      return;
    }

    // Select tracker thresholds based on target type
    // (if all stickers in this frame are outpost, use outpost-tuned thresholds)
    const bool msg_all_outpost =
      !armors_msg->armors.empty() &&
      std::all_of(armors_msg->armors.begin(), armors_msg->armors.end(),
                  [this](const rm_interfaces::msg::Armor &x) {
                    return x.number == outpost_params_.id;
                  });
    tracker_->tracking_thres = msg_all_outpost ? outpost_tracking_thres_ :
                                                  this->get_parameter("tracker.tracking_thres").as_int();
    lost_time_thres_ = msg_all_outpost ? outpost_lost_time_thres_ :
                                         this->get_parameter("tracker.lost_time_thres").as_double();

    tracker_->init(armors_msg);
    target_msg.tracking = false;
  } 
  else 
  {
    dt_ = (time - last_time_).seconds();
    // Switch tracker thresholds if currently tracking outpost
    if (tracker_->tracked_robot_type == RobotType::OUTPOST) {
      tracker_->tracking_thres = outpost_tracking_thres_;
      lost_time_thres_ = outpost_lost_time_thres_;
    }
    tracker_->lost_thres = std::abs(static_cast<int>(lost_time_thres_ / dt_));
    MotionModel motion_model = MotionModel::CONSTANT_VEL_ROT;
    if (tracker_->tracked_robot_type == RobotType::OUTPOST) {
      motion_model = static_cast<MotionModel>(outpost_motion_model_);
    }
    tracker_->setEKFDt(dt_, motion_model);

    // [debug_tracker] 帧间时间
    if (debug_tracker_) {
      PKA_DEBUG("armor_solver",
                "[Node::callback] dt={:.4f}s lost_thres={} tracked_id={}",
                dt_, tracker_->lost_thres, tracker_->tracked_id);
    }
    
    tracker_->update(armors_msg);
    // Publish measurement
    measure_msg.x   = tracker_->measurement(0);
    measure_msg.y   = tracker_->measurement(1);
    measure_msg.z   = tracker_->measurement(2);
    measure_msg.yaw = tracker_->measurement(3);
    measure_pub_->publish(measure_msg);

    if (tracker_->tracker_state == Tracker::DETECTING) 
    {
      // For outpost in DETECTING: publish single-plate fallback target so the
      // solver can compute cmd_gimbal even before the three-plate model converges.
      // For all other robot types: remain non-tracking until TRACKING is reached.
      if (tracker_->tracked_robot_type == RobotType::OUTPOST &&
          outpost_mode_ == OutpostMode::FUSION)
      {
        target_msg.id = tracker_->tracked_id;
        outpostFillDetectingTarget(outpost_params_, tracker_->target_state, &target_msg);
        // tracking flag already set to true by outpostFillDetectingTarget
      }
      else
      {
        target_msg.tracking = false;
      }
    } 
    else if (tracker_->tracker_state == Tracker::TRACKING ||
             tracker_->tracker_state == Tracker::TEMP_LOST) 
    {
      target_msg.tracking = true;
      const auto &state = tracker_->target_state;
      target_msg.id         = tracker_->tracked_id;
      target_msg.armors_num = static_cast<int>(tracker_->tracked_armors_num);

      const bool is_outpost_target = isOutpostId(tracker_->tracked_id, outpost_params_.id);

      if (!is_outpost_target) {
        // 地面兵种：10-D 状态 [xc, vx, yc, vy, zc, vz, yaw, v_yaw, r, d_zc]
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
      }
      // 前哨站：outpostApplyTargetFields 负责填充全部字段（见下方调用）

      // [debug_tracker] 发布的 target 状态摘要
      if (debug_tracker_) {
        const char * state_str =
          (tracker_->tracker_state == Tracker::TRACKING ? "TRACKING" : "TEMP_LOST");
        if (is_outpost_target && state.size() >= X_N_OUTPOST) {
          // 前哨站 6-D state: [xc, yc, zc, yaw, v_yaw, r]
          PKA_DEBUG("armor_solver",
                    "[Node::target] id={} {} outpost "
                    "center=({:.3f},{:.3f},{:.3f}) yaw={:.4f} v_yaw={:.4f} r={:.3f}",
                    target_msg.id, state_str,
                    state(0), state(1), state(2),
                    state(3), state(4), state(5));
        } else if (!is_outpost_target) {
          PKA_DEBUG("armor_solver",
                    "[Node::target] id={} {} "
                    "center=({:.3f},{:.3f},{:.3f}) vel=({:.3f},{:.3f},{:.3f}) "
                    "yaw={:.4f} v_yaw={:.4f} r1={:.3f} r2={:.3f} d_zc={:.3f}",
                    target_msg.id, state_str,
                    state(0), state(2), state(4),
                    state(1), state(3), state(5),
                    state(6), state(7), state(8),
                    tracker_->another_r, state(9));
        }
      }

      outpostApplyTargetFields(outpost_params_,
                               outpost_mode_,
                               state,
                               tracker_->d_zc,
                               tracker_->d_za,
                               tracker_->tracked_armor,
                               &target_msg);
    }
  }

  // Store and Publish the target_msg
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

  // Show markers when tracking, and also show "deferred init" outpost single-plate debug target.
  const bool show_debug_target =
    (!target_msg.tracking) && debug_mode_ && (!target_msg.id.empty()) &&
    isOutpostId(target_msg.id, outpost_params_.id) &&
    (target_msg.armors_num > 0);

  if (target_msg.tracking || show_debug_target)
  {
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
    armors_marker_.action  = (a_n > 0) ?
      visualization_msgs::msg::Marker::ADD :
      visualization_msgs::msg::Marker::DELETE;
    // Outpost is always a small armor in RViz visualization, even in deferred single-plate mode.
    // Do not rely on tracker_->tracked_armor.type here because it can be stale when tracking=false.
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
        const double pitch =
          isOutpostId(target_msg.id, outpost_params_.id) ? outpost_params_.plate_pitch_rad : 0.2618;
        q.setRPY(0.0, pitch, tmp_yaw);
        armors_marker_.pose.orientation = tf2::toMsg(q);
        marker_array.markers.emplace_back(armors_marker_);
      }
    }

    // If armors_num decreased (e.g. 3 -> 1), explicitly delete stale armor markers
    // to avoid RViz keeping old markers around and confusing the operator.
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
    for (const auto &point : solver_->getTrajectory()) 
    {
      geometry_msgs::msg::Point p;
      p.x = point.first;
      p.z = point.second;
      trajectory_marker_.points.emplace_back(p);
    }
    if (gimbal_cmd.fire_advice) 
    {
      trajectory_marker_.color.r = 0;
      trajectory_marker_.color.g = 1;
      trajectory_marker_.color.b = 0;
    } 
    else 
    {
      trajectory_marker_.color.r = 1;
      trajectory_marker_.color.g = 1;
      trajectory_marker_.color.b = 1;
    }
  } 
  else 
  {
    position_marker_.action  = visualization_msgs::msg::Marker::DELETE;
    linear_v_marker_.action  = visualization_msgs::msg::Marker::DELETE;
    angular_v_marker_.action = visualization_msgs::msg::Marker::DELETE;
    armors_marker_.action    = visualization_msgs::msg::Marker::DELETE;
    trajectory_marker_.action= visualization_msgs::msg::Marker::DELETE;
    selection_marker_.action = visualization_msgs::msg::Marker::DELETE;
    // When hiding markers, explicitly delete all armor markers that may exist in RViz
    // (appendOutpostFilteredArmorsStripMarkers publishes multiple markers with the same ns).
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
  if (mode_name == "UNKNOWN") 
  {
    PKA_ERROR("armor_solver", "Invalid mode: {}", request->mode);
    return;
  }

  switch (mode) 
  {
    case VisionMode::AUTO_AIM_RED:
    case VisionMode::AUTO_AIM_BLUE: 
    {
      enable_ = true;
      break;
    }
    default: 
    {
      enable_ = false;
      break;
    }
  }

  PKA_WARN("armor_solver", "Set Mode to {}", visionModeToString(mode));
}

}  // namespace pka::auto_aim

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pka::auto_aim::ArmorSolverNode)