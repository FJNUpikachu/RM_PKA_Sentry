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

#ifndef ARMOR_SOLVER_SOLVER_HPP_
#define ARMOR_SOLVER_SOLVER_HPP_

// std
#include <memory>
// ros2
#include <tf2_ros/buffer.h>
#include <angles/angles.h>

#include <rclcpp/time.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// 3rd party
#include <Eigen/Dense>
// project
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/msg/target.hpp"
#include "rm_utils/math/trajectory_compensator.hpp"
#include "rm_utils/math/manual_compensator.hpp"
// 引入兵种枚举，供 isOnTarget 查询装甲板大小
#include "armor_solver/armor_tracker.hpp"
#include "armor_solver/outpost_solver.hpp"

namespace pka::auto_aim {

// 各兵种装甲板实际半宽（单位：m）
constexpr double SMALL_ARMOR_HALF_W = 0.133 / 2.0;
constexpr double LARGE_ARMOR_HALF_W = 0.225 / 2.0;

// Solver class used to solve the gimbal command from tracked target
// Solver类用于求解云台跟踪目标的指令
class Solver {
public:
  explicit Solver(std::weak_ptr<rclcpp::Node> node);
  ~Solver() = default;

  // Solve the gimbal command from tracked target
  // 解算跟踪目标的 gimbal 命令
  // Throw: tf2::TransformException if the transform from "odom" to "gimbal_link" is not available
  rm_interfaces::msg::GimbalCmd solve(const rm_interfaces::msg::Target &target_msg,
                                      const rclcpp::Time &current_time,
                                      std::shared_ptr<tf2_ros::Buffer> tf2_buffer_);

  // 状态枚举
  /*
  跟踪装甲板：0
  跟踪机器人中心：1
  */
  enum State { TRACKING_ARMOR = 0, TRACKING_CENTER = 1 } state;

  std::vector<std::pair<double, double>> getTrajectory() const noexcept;

  // -------------------------------------------------------
  // 分组调试开关（由 armor_solver_node 从 yaml 注入）
  // debug_solver: 弹道、装甲板选择、开火判断、yaw/pitch 解算
  // -------------------------------------------------------
  bool debug_solver{false};

  /// Outpost parameters injected by armor_solver_node.
  /// Used to apply single-plate fire constraint for outpost armors_num==1.
  OutpostParams outpost_params_{};

private:
  // Get the armor positions from the target robot
  std::vector<Eigen::Vector3d> getArmorPositions(const Eigen::Vector3d &target_center,
                                                 const double yaw,
                                                 const double r1,
                                                 const double r2,
                                                 const double d_zc,
                                                 const double d_za,
                                                 const size_t armors_num) const noexcept;

  // Select the best armor to shoot
  // Return: selected idx in {0, 1, ..., armors_num - 1}
  int selectBestArmor(const std::vector<Eigen::Vector3d> &armor_positions,
                      const Eigen::Vector3d &target_center,
                      const double target_yaw,
                      const double target_v_yaw,
                      const size_t armors_num,
                      double &selected_delta_angle) noexcept;

  void calcYawAndPitch(const Eigen::Vector3d &p,
                       const std::array<double, 3> rpy,
                       double &yaw,
                       double &pitch) const noexcept;

  // 开火判断：基于兵种装甲板实际宽度的自适应角度容差
  // robot_type:        当前跟踪目标的兵种（决定装甲板大小）
  // armor_delta_angle: 装甲板面法向与相机视线的夹角（影响投影宽度）
  bool isOnTarget(const double cur_yaw,
                  const double cur_pitch,
                  const double target_yaw,
                  const double target_pitch,
                  const double distance,
                  const RobotType robot_type,
                  const double armor_delta_angle) const noexcept;

  std::unique_ptr<TrajectoryCompensator> trajectory_compensator_;
  std::unique_ptr<ManualCompensator> manual_compensator_;

  std::array<double, 3> rpy_;

  double prediction_delay_;
  double controller_delay_;

  double shooting_range_w_;
  double shooting_range_h_;

  double max_tracking_v_yaw_;
  int overflow_count_;
  int transfer_thresh_;

  double coming_angle_;
  double leaving_angle_;
  double min_switching_v_yaw_;

  // 开火容差参数（基于装甲板投影宽度）
  double fire_margin_;             // 投影宽度的缩放因子（<1 保守，>1 宽松）
  double min_fire_tolerance_rad_;  // 开火容差下界（rad）
  double max_fire_tolerance_rad_;  // 开火容差上界（rad）
  mutable bool last_on_target_ = false;  // 稳定性滤波：连续两帧在目标内才开火

  // 瞄准偏移量（与 launch rpy 解耦，不影响 yaw 优化）
  double yaw_offset_;    // 单位：度
  double pitch_offset_;  // 单位：度

  // 装甲板选择锁定 ID（低速时防止抖动切换）
  int lock_id_ = -1;

  // center_mode 相关参数（命名与 .cpp 保持一致）
  bool center_mode_;
  double center_dis_;   // 距离阈值（超出后不开火）
  double center_yaw_;   // yaw 误差阈值（用于 fire_advice 判断）

  std::weak_ptr<rclcpp::Node> node_;
};
}  // namespace pka::auto_aim
#endif  // ARMOR_SOLVER_SOLVER_HPP_