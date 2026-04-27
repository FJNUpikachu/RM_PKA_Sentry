// Created by Chengfu Zou
// Maintained by Chengfu Zou, Labor
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

#include "armor_solver/armor_solver.hpp"
// std
#include <cmath>
#include <cstddef>
#include <stdexcept>
// project
#include "armor_solver/armor_solver_node.hpp"
#include "rm_utils/pkaLoggerCenter.hpp"
#include "rm_utils/math/utils.hpp"

namespace pka::auto_aim {
Solver::Solver(std::weak_ptr<rclcpp::Node> n) : node_(n) {
  auto node = node_.lock();

  shooting_range_w_ = node->declare_parameter("solver.shooting_range_width", 0.135);
  shooting_range_h_ = node->declare_parameter("solver.shooting_range_height", 0.135);
  // 最大跟踪角速度
  max_tracking_v_yaw_ = node->declare_parameter("solver.max_tracking_v_yaw", 6.0);
  // 预测延时
  prediction_delay_ = node->declare_parameter("solver.prediction_delay", 0.0);
  // 控制延时
  controller_delay_ = node->declare_parameter("solver.controller_delay", 0.0);
  // 侧装甲相对于中心的角度（单位：度，保留参数声明）
  node->declare_parameter("solver.side_angle", 15.0);
  // Coming/Leaving 非对称角度（单位：度）
  coming_angle_ = node->declare_parameter("solver.coming_angle", 55.0);
  leaving_angle_ = node->declare_parameter("solver.leaving_angle", 20.0);
  // 最小的切换角速度阈值
  min_switching_v_yaw_ = node->declare_parameter("solver.min_switching_v_yaw", 1.0);

  // 开火容差参数
  // fire_margin_: 投影宽度缩放因子（0.8 = 偏保守，减少打边缘概率）
  fire_margin_ = node->declare_parameter("solver.fire_margin", 0.8);
  double min_tol_deg = node->declare_parameter("solver.min_fire_tolerance", 1.5);
  double max_tol_deg = node->declare_parameter("solver.max_fire_tolerance", 4.0);
  min_fire_tolerance_rad_ = min_tol_deg * M_PI / 180.0;
  max_fire_tolerance_rad_ = max_tol_deg * M_PI / 180.0;

  // 瞄准偏移量（与 launch rpy 解耦）
  yaw_offset_   = node->declare_parameter("solver.yaw_offset",   0.0);
  pitch_offset_ = node->declare_parameter("solver.pitch_offset", 0.0);

  // 补偿器类型
  std::string compenstator_type = node->declare_parameter("solver.compensator_type", "ideal");
  // 创建弹道补偿器
  trajectory_compensator_ = CompensatorFactory::createCompensator(compenstator_type);
  // 迭代次数
  trajectory_compensator_->iteration_times = node->declare_parameter("solver.iteration_times", 20);
  // 弹速
  trajectory_compensator_->velocity = node->declare_parameter("solver.bullet_speed", 20.0);
  // 重力加速度
  trajectory_compensator_->gravity = node->declare_parameter("solver.gravity", 9.8);
  // 阻力
  trajectory_compensator_->resistance = node->declare_parameter("solver.resistance", 0.001);

  // 创建手动补偿器类
  manual_compensator_ = std::make_unique<ManualCompensator>();
  // 初始化参数angle_offset
  auto angle_offset = node->declare_parameter("solver.angle_offset", std::vector<std::string>{});
  if (!manual_compensator_->updateMapFlow(angle_offset)) {
    PKA_WARN("armor_solver", "Manual compensator update failed!");
  }

  // -------------------------------------------------------
  // center_mode 参数（yaml 键名与变量名对齐）
  // center_mode: 是否启用瞄准中心模式（bool）
  // center_dis:  距离上限，超出该值不开火（单位 m）
  // center_yaw:  云台与目标 yaw 误差上限，超出该值不开火（单位 rad）
  // -------------------------------------------------------
  center_mode_ = node->declare_parameter("solver.center_mode", false);
  center_dis_  = node->declare_parameter("solver.center_dis",  5.3);
  center_yaw_  = node->declare_parameter("solver.center_yaw",  0.135);

  // 初始化状态为跟踪装甲板
  state = State::TRACKING_ARMOR;
  overflow_count_ = 0;
  transfer_thresh_ = 5;

  node.reset();
}

rm_interfaces::msg::GimbalCmd Solver::solve(const rm_interfaces::msg::Target &target,
                                            const rclcpp::Time &current_time,
                                            std::shared_ptr<tf2_ros::Buffer> tf2_buffer_) {
  // Get newest parameters
  // 获得最新的参数
  try {
    auto node = node_.lock();
    max_tracking_v_yaw_  = node->get_parameter("solver.max_tracking_v_yaw").as_double();
    prediction_delay_    = node->get_parameter("solver.prediction_delay").as_double();
    controller_delay_    = node->get_parameter("solver.controller_delay").as_double();
    coming_angle_        = node->get_parameter("solver.coming_angle").as_double();
    leaving_angle_       = node->get_parameter("solver.leaving_angle").as_double();
    min_switching_v_yaw_ = node->get_parameter("solver.min_switching_v_yaw").as_double();
    fire_margin_         = node->get_parameter("solver.fire_margin").as_double();
    yaw_offset_          = node->get_parameter("solver.yaw_offset").as_double();
    pitch_offset_        = node->get_parameter("solver.pitch_offset").as_double();
    // center_mode 三个参数支持 ros2 param set 运行时动态生效
    center_mode_ = node->get_parameter("solver.center_mode").as_bool();
    center_dis_  = node->get_parameter("solver.center_dis").as_double();
    center_yaw_  = node->get_parameter("solver.center_yaw").as_double();
    node.reset();
  } catch (const std::runtime_error &e) {
    PKA_ERROR("armor_solver", "{}", e.what());
  }

  // 解析当前跟踪目标的兵种类型，用于后续装甲板大小查询
  // target.id 字段由 armor_solver_node 从 tracker_->tracked_id 填入
  const RobotType robot_type = robotTypeFromId(target.id);

  // Get current roll, yaw and pitch of gimbal
  // 获得云台最近的roll、yaw和pitch
  try {
    auto gimbal_tf = tf2_buffer_->lookupTransform(target.header.frame_id, "gimbal_link", tf2::TimePointZero);
    auto msg_q = gimbal_tf.transform.rotation;

    tf2::Quaternion tf_q;
    tf2::fromMsg(msg_q, tf_q);
    tf2::Matrix3x3(tf_q).getRPY(rpy_[0], rpy_[1], rpy_[2]);
    rpy_[1] = -rpy_[1];
  } catch (tf2::TransformException &ex) {
    PKA_ERROR("armor_solver", "{}", ex.what());
    throw ex;
  }

  // [debug_solver] 当前云台 RPY
  if (debug_solver) {
    PKA_DEBUG("armor_solver",
              "[Solver::solve] gimbal rpy: roll={:.4f} pitch={:.4f} yaw={:.4f} (rad)",
              rpy_[0], rpy_[1], rpy_[2]);
  }

  // Use flying time to approximately predict the position of target
  // 使用飞行时间大致预测目标的位置
  Eigen::Vector3d target_position(target.position.x, target.position.y, target.position.z);
  double target_yaw = target.yaw;
  // 计算飞行时间
  double flying_time = trajectory_compensator_->getFlyingTime(target_position);

  double dt = (current_time - rclcpp::Time(target.header.stamp)).seconds() + flying_time + prediction_delay_;
  target_position.x() += dt * target.velocity.x;
  target_position.y() += dt * target.velocity.y;
  target_position.z() += dt * target.velocity.z;
  target_yaw += dt * target.v_yaw;

  // [debug_solver] 弹道预测详情
  if (debug_solver) {
    PKA_DEBUG("armor_solver",
              "[Solver::predict] flying_time={:.4f}s dt={:.4f}s "
              "target_pos=({:.3f},{:.3f},{:.3f}) target_yaw={:.4f} v_yaw={:.4f}",
              flying_time, dt,
              target_position.x(), target_position.y(), target_position.z(),
              target_yaw, target.v_yaw);
  }

  // Choose the best armor to shoot
  // 选择最好的装甲板击打
  std::vector<Eigen::Vector3d> armor_positions = getArmorPositions(
    target_position, target_yaw, target.radius_1, target.radius_2, target.d_zc, target.d_za, target.armors_num);

  double selected_delta_angle = 0.0;
  int idx = selectBestArmor(
    armor_positions, target_position, target_yaw, target.v_yaw, target.armors_num, selected_delta_angle);
  Eigen::Vector3d front_armor_pos = armor_positions.at(idx);

  // [debug_solver] 装甲板选择结果
  if (debug_solver) {
    PKA_DEBUG("armor_solver",
              "[Solver::selectArmor] selected_idx={} delta_angle={:.4f}rad ({:.2f}deg) "
              "armor_pos=({:.3f},{:.3f},{:.3f}) distance={:.3f}m",
              idx, selected_delta_angle, selected_delta_angle * 180.0 / M_PI,
              front_armor_pos.x(), front_armor_pos.y(), front_armor_pos.z(),
              front_armor_pos.norm());
  }

  if (front_armor_pos.norm() < 0.1) {
    throw std::runtime_error("No valid armor to shoot");
  }

  // -------------------------------------------------------
  // center_mode：瞄准机器人整车中心，旋转陀螺时使用
  // -------------------------------------------------------
  if (center_mode_ && target.v_yaw >= 4.1) {
    // 应用 controller_delay 补偿后的位置用于解算指向角
    if (controller_delay_ != 0) {
      target_position.x() += controller_delay_ * target.velocity.x;
      target_position.y() += controller_delay_ * target.velocity.y;
      target_position.z() += controller_delay_ * target.velocity.z;
      double target_yaw_delayed = target_yaw + controller_delay_ * target.v_yaw;
      armor_positions = getArmorPositions(
        target_position, target_yaw_delayed,
        target.radius_1, target.radius_2,
        target.d_zc, target.d_za,
        target.armors_num);
      front_armor_pos = armor_positions.at(idx);
      target_yaw = target_yaw_delayed;
    }

    // 解算指向整车中心的 yaw/pitch
    double center_yaw_cmd, center_pitch_cmd;
    calcYawAndPitch(target_position, rpy_, center_yaw_cmd, center_pitch_cmd);

    // 手动补偿
    auto angle_offset_val = manual_compensator_->angleHardCorrect(
      target_position.head(2).norm(), target_position.z());
    double pitch_offset_val = angle_offset_val[0] * M_PI / 180.0;
    double yaw_offset_val   = angle_offset_val[1] * M_PI / 180.0;

    // 叠加 yaw_offset_ / pitch_offset_（与外参解耦的机械偏差补偿）
    double cmd_pitch = center_pitch_cmd + pitch_offset_val + pitch_offset_ * M_PI / 180.0;
    double cmd_yaw   = angles::normalize_angle(center_yaw_cmd + yaw_offset_val + yaw_offset_ * M_PI / 180.0);

    rm_interfaces::msg::GimbalCmd gimbal_cmd;
    gimbal_cmd.header     = target.header;
    gimbal_cmd.distance   = target_position.norm();
    gimbal_cmd.yaw        = cmd_yaw   * 180.0 / M_PI;
    gimbal_cmd.pitch      = cmd_pitch * 180.0 / M_PI;
    gimbal_cmd.yaw_diff   = (cmd_yaw   - rpy_[2]) * 180.0 / M_PI;
    gimbal_cmd.pitch_diff = (cmd_pitch - rpy_[1]) * 180.0 / M_PI;

    // 开火判断：yaw 误差在 center_yaw_ 以内 且 距离不超过 center_dis_
    double predicted_target_yaw = target.yaw + dt * target.v_yaw;
    if (controller_delay_ != 0) {
      predicted_target_yaw += controller_delay_ * target.v_yaw;
    }
    double dis = target_position.norm();
    bool in_yaw_range = std::abs(angles::normalize_angle(rpy_[2] - predicted_target_yaw)) <= center_yaw_;
    bool in_dis_range = (dis < center_dis_);
    gimbal_cmd.fire_advice = in_yaw_range && in_dis_range;

    // [debug_solver] center_mode 开火判断详情
    if (debug_solver) {
      PKA_DEBUG("armor_solver",
                "[Solver::center_mode] v_yaw={:.3f} cmd_yaw={:.3f}deg cmd_pitch={:.3f}deg "
                "dis={:.3f}m in_yaw={} in_dis={} fire={}",
                target.v_yaw,
                cmd_yaw * 180.0 / M_PI,
                cmd_pitch * 180.0 / M_PI,
                dis,
                in_yaw_range, in_dis_range,
                gimbal_cmd.fire_advice);
    }

    return gimbal_cmd;
  }

  // -------------------------------------------------------
  // 正常模式（TRACKING_ARMOR / TRACKING_CENTER 状态机）
  // -------------------------------------------------------

  auto chosen_armor_position = front_armor_pos;

  // Calculate yaw, pitch, distance
  // 计算yaw，pitch和distance
  double yaw, pitch;
  calcYawAndPitch(chosen_armor_position, rpy_, yaw, pitch);
  double distance = chosen_armor_position.norm();

  // [debug_solver] 初次解算 yaw/pitch
  if (debug_solver) {
    PKA_DEBUG("armor_solver",
              "[Solver::calcYawPitch] raw yaw={:.4f}rad ({:.2f}deg) pitch={:.4f}rad ({:.2f}deg) "
              "distance={:.3f}m",
              yaw, yaw * 180.0 / M_PI,
              pitch, pitch * 180.0 / M_PI,
              distance);
  }

  // Initialize gimbal_cmd
  // 初始化云台指令
  rm_interfaces::msg::GimbalCmd gimbal_cmd;
  gimbal_cmd.header   = target.header;
  gimbal_cmd.distance = distance;
  // 使用基于兵种装甲板实际宽度的自适应开火判断
  gimbal_cmd.fire_advice = isOnTarget(
    rpy_[2], rpy_[1], yaw, pitch, distance, robot_type, selected_delta_angle);

  // 如果目标角速度超过最大跟踪角速度，建议开火
  if (std::abs(target.v_yaw) > max_tracking_v_yaw_) {
    gimbal_cmd.fire_advice = true;
  }

  switch (state) {
    // 如果为跟踪装甲板模式
    case TRACKING_ARMOR: {
      // 如果目标转速大于阈值转速超过五次，切换到跟踪中心
      if (std::abs(target.v_yaw) > max_tracking_v_yaw_) {
        overflow_count_++;
      } else {
        overflow_count_ = 0;
      }

      if (overflow_count_ > transfer_thresh_) {
        state = TRACKING_CENTER;

        if (debug_solver) {
          PKA_DEBUG("armor_solver",
                    "[Solver::state] TRACKING_ARMOR -> TRACKING_CENTER "
                    "v_yaw={:.3f} > max={:.3f} (overflow_count={})",
                    target.v_yaw, max_tracking_v_yaw_, overflow_count_);
        }
      }

      if (controller_delay_ != 0) {
        target_position.x() += controller_delay_ * target.velocity.x;
        target_position.y() += controller_delay_ * target.velocity.y;
        target_position.z() += controller_delay_ * target.velocity.z;
        target_yaw += controller_delay_ * target.v_yaw;
        armor_positions = getArmorPositions(target_position,
                                            target_yaw,
                                            target.radius_1,
                                            target.radius_2,
                                            target.d_zc,
                                            target.d_za,
                                            target.armors_num);
        chosen_armor_position = armor_positions.at(idx);
        gimbal_cmd.distance = chosen_armor_position.norm();
        if (chosen_armor_position.norm() < 0.1) {
          throw std::runtime_error("No valid armor to shoot");
        }
        calcYawAndPitch(chosen_armor_position, rpy_, yaw, pitch);

        // [debug_solver] controller_delay 补偿后的位置
        if (debug_solver) {
          PKA_DEBUG("armor_solver",
                    "[Solver::TRACKING_ARMOR] ctrl_delay={:.4f}s "
                    "armor_pos=({:.3f},{:.3f},{:.3f}) yaw={:.4f}rad pitch={:.4f}rad",
                    controller_delay_,
                    chosen_armor_position.x(), chosen_armor_position.y(), chosen_armor_position.z(),
                    yaw, pitch);
        }
      }
      break;
    }
    // 如果为瞄准中心模式（内部状态机版本，由 max_tracking_v_yaw 触发）
    case TRACKING_CENTER: {
      if (std::abs(target.v_yaw) < max_tracking_v_yaw_) {
        overflow_count_++;
      } else {
        overflow_count_ = 0;
      }

      if (overflow_count_ > transfer_thresh_) {
        state = TRACKING_ARMOR;
        overflow_count_ = 0;

        if (debug_solver) {
          PKA_DEBUG("armor_solver",
                    "[Solver::state] TRACKING_CENTER -> TRACKING_ARMOR "
                    "v_yaw={:.3f} < max={:.3f} (overflow_count={})",
                    target.v_yaw, max_tracking_v_yaw_, overflow_count_);
        }
      }
      // 瞄准中心时持续开火
      gimbal_cmd.fire_advice = true;
      // 补充缺失的瞄准中心解算，否则 yaw/pitch 停留在上一帧装甲板位置
      calcYawAndPitch(target_position, rpy_, yaw, pitch);
      gimbal_cmd.distance = target_position.norm();

      // [debug_solver] TRACKING_CENTER 解算
      if (debug_solver) {
        PKA_DEBUG("armor_solver",
                  "[Solver::TRACKING_CENTER] center_pos=({:.3f},{:.3f},{:.3f}) "
                  "yaw={:.4f}rad pitch={:.4f}rad dis={:.3f}m",
                  target_position.x(), target_position.y(), target_position.z(),
                  yaw, pitch, gimbal_cmd.distance);
      }
      break;
    }
  }

  // Compensate angle by angle_offset_map（距离分段手动补偿）
  auto angle_offset = manual_compensator_->angleHardCorrect(
    target_position.head(2).norm(), target_position.z());
  double pitch_offset_val = angle_offset[0] * M_PI / 180.0;
  double yaw_offset_val   = angle_offset[1] * M_PI / 180.0;

  // [debug_solver] 手动分段补偿量
  if (debug_solver) {
    PKA_DEBUG("armor_solver",
              "[Solver::manualComp] dist2d={:.3f}m z={:.3f}m "
              "pitch_offset={:.4f}deg yaw_offset={:.4f}deg",
              target_position.head(2).norm(), target_position.z(),
              angle_offset[0], angle_offset[1]);
  }

  // 叠加 yaw_offset_ / pitch_offset_（与外参解耦的机械偏差补偿）
  double cmd_pitch = pitch + pitch_offset_val + pitch_offset_ * M_PI / 180.0;
  double cmd_yaw   = angles::normalize_angle(yaw + yaw_offset_val + yaw_offset_ * M_PI / 180.0);

  gimbal_cmd.yaw        = cmd_yaw   * 180.0 / M_PI;
  gimbal_cmd.pitch      = cmd_pitch * 180.0 / M_PI;
  gimbal_cmd.yaw_diff   = (cmd_yaw   - rpy_[2]) * 180.0 / M_PI;
  gimbal_cmd.pitch_diff = (cmd_pitch - rpy_[1]) * 180.0 / M_PI;

  // [debug_solver] 最终输出指令
  if (debug_solver) {
    PKA_DEBUG("armor_solver",
              "[Solver::output] yaw={:.3f}deg pitch={:.3f}deg "
              "yaw_diff={:.3f}deg pitch_diff={:.3f}deg fire={}",
              gimbal_cmd.yaw, gimbal_cmd.pitch,
              gimbal_cmd.yaw_diff, gimbal_cmd.pitch_diff,
              gimbal_cmd.fire_advice);
  }

  // ── Outpost single-plate fire constraint ────────────────────────────────
  // When using single-plate model (armors_num==1, either DETECTING fallback
  // or permanent SINGLE_PLATE mode), apply the yaw_diff limit configured in
  // outpost.fire_yaw_max_deg.  All constraint logic lives in outpost_solver.
  if (isOutpostId(target.id, outpost_params_.id) && target.armors_num == 1) {
    outpostApplySinglePlateFireConstraint(outpost_params_, &gimbal_cmd);
  }

  if (gimbal_cmd.fire_advice) {
    //PKA_DEBUG("armor_solver", "Fire!");
  }
  return gimbal_cmd;
}

bool Solver::isOnTarget(const double cur_yaw,
                        const double cur_pitch,
                        const double target_yaw,
                        const double target_pitch,
                        const double distance,
                        const RobotType robot_type,
                        const double armor_delta_angle) const noexcept {
  // -------------------------------------------------------
  // 装甲板宽度查表：根据兵种枚举决定使用大/小装甲板半宽
  //
  //   大装甲板：英雄(HERO_1)、基地(BASE)
  //   小装甲板：工程(ENGINEER_2)、步兵(INFANTRY_3/4)、
  //             哨兵(SENTRY_5)
  //
  // 使用 isLargeArmor() 工具函数，与 tracker 保持一致的判断逻辑
  // -------------------------------------------------------
  double armor_half_w = isLargeArmor(robot_type) ? LARGE_ARMOR_HALF_W : SMALL_ARMOR_HALF_W;

  // 根据装甲板与相机视线的夹角计算投影宽度
  // armor_delta_angle 越大（装甲板越侧对），投影宽度越小，容差越小
  double cos_incidence    = std::abs(std::cos(armor_delta_angle));
  double projected_half_w = armor_half_w * cos_incidence;

  // 角度容差 = atan(投影半宽 / 距离) × 缩放因子
  // fire_margin_ < 1：保守（只有对准装甲板中心才开火）
  // fire_margin_ > 1：宽松（瞄偏一点也开火）
  double tolerance_rad = std::atan2(projected_half_w, distance) * fire_margin_;
  
  // 钳制到 [min_fire_tolerance_rad_, max_fire_tolerance_rad_]
  // 防止近距离容差过大、远距离容差退化为零
  tolerance_rad = std::clamp(tolerance_rad, min_fire_tolerance_rad_, max_fire_tolerance_rad_);

  // 采用最短角进行相减，避免角度周期性导致的误判
  double yaw_err = std::abs(angles::shortest_angular_distance(cur_yaw, target_yaw));
  bool on_target = yaw_err < tolerance_rad;

  // 连续两帧在目标内才开火（稳定性滤波）
  bool stable      = on_target && last_on_target_;
  last_on_target_  = on_target;

  // [debug_solver] 开火判断详情
  if (debug_solver) {
    PKA_DEBUG("armor_solver",
              "[Solver::isOnTarget] large={} half_w={:.4f} cos={:.4f} proj_hw={:.4f} "
              "distance={:.3f}m tolerance={:.4f}rad ({:.2f}deg) yaw_err={:.4f}rad "
              "on_target={} last_on_target={} fire={}",
              isLargeArmor(robot_type),
              armor_half_w, cos_incidence, projected_half_w,
              distance,
              tolerance_rad, tolerance_rad * 180.0 / M_PI,
              yaw_err,
              on_target, last_on_target_,
              stable);
  }

  return stable;
}

std::vector<Eigen::Vector3d> Solver::getArmorPositions(const Eigen::Vector3d &target_center,
                                                       const double target_yaw,
                                                       const double r1,
                                                       const double r2,
                                                       const double d_zc,
                                                       const double d_za,
                                                       const size_t armors_num) const noexcept {
  auto armor_positions = std::vector<Eigen::Vector3d>(armors_num, Eigen::Vector3d::Zero());
  bool is_current_pair = true;
  double r = 0., target_dz = 0.;
  for (size_t i = 0; i < armors_num; i++) {
    if (armors_num == 3) {
      armor_positions[i] =
        outpostThreePlateSlotWorldPosition(target_center, target_yaw, i, r1, d_zc, d_za);
      continue;
    }
    const double temp_yaw = target_yaw + static_cast<double>(i) * (2 * M_PI / armors_num);
    if (armors_num == 4) {
      r = is_current_pair ? r1 : r2;
      target_dz = d_zc + (is_current_pair ? 0 : d_za);
      is_current_pair = !is_current_pair;
    } else {
      r = r1;
      target_dz = d_zc;
    }
    armor_positions[i] =
      target_center + Eigen::Vector3d(-r * cos(temp_yaw), -r * sin(temp_yaw), target_dz);
  }
  return armor_positions;
}

int Solver::selectBestArmor(const std::vector<Eigen::Vector3d> &armor_positions,
                            const Eigen::Vector3d &target_center,
                            const double target_yaw,
                            const double target_v_yaw,
                            const size_t armors_num,
                            double &selected_delta_angle) noexcept {
  // Angle between the car's center and the X-axis
  double alpha = std::atan2(target_center.y(), target_center.x());

  // Compute delta_angle for each armor relative to camera-center line
  std::vector<double> delta_angles(armors_num);
  for (size_t i = 0; i < armors_num; i++) {
    double armor_yaw = target_yaw + i * (2 * M_PI / armors_num);
    double delta = armor_yaw - alpha;
    // Normalize to [-pi, pi]
    while (delta > M_PI) delta -= 2 * M_PI;
    while (delta < -M_PI) delta += 2 * M_PI;
    delta_angles[i] = delta;
  }

  // [debug_solver] 各装甲板 delta_angle 列表
  if (debug_solver) {
    std::string da_str;
    for (size_t i = 0; i < armors_num; i++) {
      da_str += std::to_string(i) + ":" + std::to_string(delta_angles[i] * 180.0 / M_PI).substr(0, 6) + "deg ";
    }
    PKA_DEBUG("armor_solver",
              "[Solver::selectArmor] alpha={:.4f}rad v_yaw={:.3f} delta_angles: {}",
              alpha, target_v_yaw, da_str);
  }

  int selected_id = 0;

  if (std::abs(target_v_yaw) < min_switching_v_yaw_) {
    // Low speed: select armor with smallest |delta_angle|
    double min_abs_delta = std::abs(delta_angles[0]);
    for (size_t i = 1; i < armors_num; i++) {
      double abs_delta = std::abs(delta_angles[i]);
      if (abs_delta < min_abs_delta) {
        min_abs_delta = abs_delta;
        selected_id = static_cast<int>(i);
      }
    }
    // Lock mechanism: if the previous selection is still within 60°, keep it
    if (lock_id_ >= 0 && lock_id_ < static_cast<int>(armors_num)) {
      double lock_delta = std::abs(delta_angles[lock_id_]);
      if (lock_delta < M_PI / 3 && std::abs(lock_delta - min_abs_delta) < M_PI / 6) {
        if (debug_solver && selected_id != lock_id_) {
          PKA_DEBUG("armor_solver",
                    "[Solver::selectArmor] low_speed lock kept: lock_id={} "
                    "lock_delta={:.4f}rad (would switch to {})",
                    lock_id_, lock_delta, selected_id);
        }
        selected_id = lock_id_;
      }
    }

    if (debug_solver) {
      PKA_DEBUG("armor_solver",
                "[Solver::selectArmor] low_speed selected_id={} delta={:.4f}rad ({:.2f}deg)",
                selected_id, delta_angles[selected_id],
                delta_angles[selected_id] * 180.0 / M_PI);
    }
  } else {
    // High speed spinning: use asymmetric coming/leaving angles
    // coming_angle_: 向"迎面转来"方向多看（提前锁定）
    // leaving_angle_: 向"转走"方向少看（快速放弃，切换下一块）
    double coming_rad  = coming_angle_  / 180.0 * M_PI;
    double leaving_rad = leaving_angle_ / 180.0 * M_PI;

    double best_score = 1e9;
    for (size_t i = 0; i < armors_num; i++) {
      double delta = delta_angles[i];
      bool in_coming_zone;
      if (target_v_yaw < 0) {
        in_coming_zone = (delta > -leaving_rad && delta < coming_rad);
      } else {
        in_coming_zone = (delta > -coming_rad && delta < leaving_rad);
      }

      if (in_coming_zone) {
        double score = std::abs(delta);
        if (score < best_score) {
          best_score  = score;
          selected_id = static_cast<int>(i);
        }
      }
    }

    // Fallback: if no armor in coming zone, pick closest
    if (best_score >= 1e9) {
      if (debug_solver) {
        PKA_DEBUG("armor_solver",
                  "[Solver::selectArmor] high_speed fallback: no armor in coming zone, "
                  "coming_rad={:.3f} leaving_rad={:.3f}",
                  coming_rad, leaving_rad);
      }
      double min_abs_delta = 1e9;
      for (size_t i = 0; i < armors_num; i++) {
        double abs_delta = std::abs(delta_angles[i]);
        if (abs_delta < min_abs_delta) {
          min_abs_delta = abs_delta;
          selected_id   = static_cast<int>(i);
        }
      }
    }

    if (debug_solver) {
      PKA_DEBUG("armor_solver",
                "[Solver::selectArmor] high_speed selected_id={} delta={:.4f}rad ({:.2f}deg) "
                "coming_zone=[{:.2f},{:.2f}]deg",
                selected_id,
                delta_angles[selected_id],
                delta_angles[selected_id] * 180.0 / M_PI,
                (target_v_yaw < 0 ? -leaving_angle_ : -coming_angle_),
                (target_v_yaw < 0 ?  coming_angle_  :  leaving_angle_));
    }
  }

  lock_id_             = selected_id;
  selected_delta_angle = delta_angles[selected_id];
  return selected_id;
}

void Solver::calcYawAndPitch(const Eigen::Vector3d &p,
                             const std::array<double, 3> rpy,
                             double &yaw,
                             double &pitch) const noexcept {
  yaw   = atan2(p.y(), p.x());
  pitch = atan2(p.z(), p.head(2).norm());

  if (double temp_pitch = pitch; trajectory_compensator_->compensate(p, temp_pitch)) {
    pitch = temp_pitch;
  }
}

std::vector<std::pair<double, double>> Solver::getTrajectory() const noexcept {
  auto trajectory = trajectory_compensator_->getTrajectory(15, rpy_[1]);
  for (auto &p : trajectory) {
    double x = p.first;
    double y = p.second;
    p.first  = x * cos(rpy_[1]) + y * sin(rpy_[1]);
    p.second = -x * sin(rpy_[1]) + y * cos(rpy_[1]);
  }
  return trajectory;
}

}  // namespace pka::auto_aim