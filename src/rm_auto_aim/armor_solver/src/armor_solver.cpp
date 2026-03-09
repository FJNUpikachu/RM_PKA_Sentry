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
  // 跳转到下一装甲板的角度阈值
  side_angle_ = node->declare_parameter("solver.side_angle", 15.0);

  // 最小的切换角速度阈值（！！！）
  min_switching_v_yaw_ = node->declare_parameter("solver.min_switching_v_yaw", 1.0);

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
  if(!manual_compensator_->updateMapFlow(angle_offset)) 
  {
    // 打印手动补偿器更新失败
    PKA_WARN("armor_solver", "Manual compensator update failed!");
  }

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
  try 
  {
    auto node = node_.lock();
    max_tracking_v_yaw_ = node->get_parameter("solver.max_tracking_v_yaw").as_double();
    prediction_delay_ = node->get_parameter("solver.prediction_delay").as_double();
    controller_delay_ = node->get_parameter("solver.controller_delay").as_double();
    side_angle_ = node->get_parameter("solver.side_angle").as_double();
    min_switching_v_yaw_ = node->get_parameter("solver.min_switching_v_yaw").as_double();
    // 重置智能指针，释放资源
    node.reset();
  } catch (const std::runtime_error &e) 
  {
    PKA_ERROR("armor_solver", "{}", e.what());
  }

  // Get current roll, yaw and pitch of gimbal
  // 获得云台最近的roll、yaw和pitch
  try 
  {
    // 得到从gimbal_link到target的转换关系
    auto gimbal_tf = tf2_buffer_->lookupTransform(target.header.frame_id, "gimbal_link", tf2::TimePointZero);
    auto msg_q = gimbal_tf.transform.rotation;

    tf2::Quaternion tf_q;
    tf2::fromMsg(msg_q, tf_q);
    // 
    tf2::Matrix3x3(tf_q).getRPY(rpy_[0], rpy_[1], rpy_[2]);
    rpy_[1] = -rpy_[1];
  } 
  catch (tf2::TransformException &ex) 
  {
    PKA_ERROR("armor_solver", "{}", ex.what());
    throw ex;
  }

  // Use flying time to approximately predict the position of target
  // 使用飞行时间大致预测目标的位置
  Eigen::Vector3d target_position(target.position.x, target.position.y, target.position.z);
  double target_yaw = target.yaw;
  // 计算飞行时间
  double flying_time = trajectory_compensator_->getFlyingTime(target_position);

  double dt =(current_time - rclcpp::Time(target.header.stamp)).seconds() + flying_time + prediction_delay_;
  target_position.x() += dt * target.velocity.x;
  target_position.y() += dt * target.velocity.y;
  target_position.z() += dt * target.velocity.z;
  target_yaw += dt * target.v_yaw;

  // Choose the best armor to shoot
  // 选择最好的装甲板击打
  std::vector<Eigen::Vector3d> armor_positions = getArmorPositions(
    target_position, target_yaw, target.radius_1, target.radius_2, target.d_zc, target.d_za, target.armors_num);
  int idx =
    selectBestArmor(armor_positions, target_position, target_yaw, target.v_yaw, target.armors_num);
  auto chosen_armor_position = armor_positions.at(idx);
  if (chosen_armor_position.norm() < 0.1) 
  {
    throw std::runtime_error("No valid armor to shoot");
  }

  // Calculate yaw, pitch, distance
  // 计算yaw，pitch和distance
  double yaw, pitch;
  calcYawAndPitch(chosen_armor_position, rpy_, yaw, pitch);
  double distance = chosen_armor_position.norm();

  // Initialize gimbal_cmd
  // 初始化云台指令
  rm_interfaces::msg::GimbalCmd gimbal_cmd;
  gimbal_cmd.header = target.header;
  gimbal_cmd.distance = distance;
  gimbal_cmd.fire_advice = isOnTarget(rpy_[2], rpy_[1], yaw, pitch, distance);

  switch (state) 
  {
    case TRACKING_ARMOR: 
    {
      // 如果目标转速大于阈值转速超过五次，云台不跟随
      if (std::abs(target.v_yaw) > max_tracking_v_yaw_) 
      {
        overflow_count_++;
      } 
      else 
      {
        overflow_count_ = 0;
      }

      if (overflow_count_ > transfer_thresh_) 
      {
        state = TRACKING_CENTER;
      }

      // If isOnTarget() never returns true, adjust controller_delay to force the gimbal to   move
      if (controller_delay_ != 0) 
      {
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
      }
      break;
    }
    // 如果为瞄准中心模式
    case TRACKING_CENTER: 
    {
      // 如果目标转速小于阈值转速超过五次
      if (std::abs(target.v_yaw) < max_tracking_v_yaw_) 
      {
        overflow_count_++;
      } 
      else 
      {
        overflow_count_ = 0;
      }

      if (overflow_count_ > transfer_thresh_) 
      {
        state = TRACKING_ARMOR;
        overflow_count_ = 0;
      }
      // 因为为瞄准中心所以一直为开火状态
      gimbal_cmd.fire_advice = true;
      calcYawAndPitch(target_position, rpy_, yaw, pitch);
      break;
    }
  }

  // Compensate angle by angle_offset_map
  // 按angle_offset_map补偿角度
  // target_position.head(2).norm() = sqrt(pow(target_position.x,2)+pow(target_position.y,2));
  auto angle_offset = manual_compensator_->angleHardCorrect(target_position.head(2).norm(), target_position.z());
  double pitch_offset = angle_offset[0] * M_PI / 180;
  double yaw_offset = angle_offset[1] * M_PI / 180;
  double cmd_pitch = pitch + pitch_offset;
  double cmd_yaw = angles::normalize_angle(yaw + yaw_offset);


  gimbal_cmd.yaw = cmd_yaw * 180 / M_PI;
  gimbal_cmd.pitch = cmd_pitch * 180 / M_PI;  
  gimbal_cmd.yaw_diff = (cmd_yaw - rpy_[2]) * 180 / M_PI;
  gimbal_cmd.pitch_diff = (cmd_pitch - rpy_[1]) * 180 / M_PI;

  // if (gimbal_cmd.fire_advice) 
  // {
  //   PKA_DEBUG("armor_solver", "You Need Fire!");
  // }
  return gimbal_cmd;
}

bool Solver::isOnTarget(const double cur_yaw,
                        const double cur_pitch,
                        const double target_yaw,
                        const double target_pitch,
                        const double distance) const noexcept {
  // Judge whether to shoot
  double shooting_range_yaw = std::abs(atan2(shooting_range_w_ / 2, distance));
  double shooting_range_pitch = std::abs(atan2(shooting_range_h_ / 2, distance));
  // Limit the shooting area to 1 degree to avoid not shooting when distance is
  // too large
  shooting_range_yaw = std::max(shooting_range_yaw, 1.0 * M_PI / 180);
  shooting_range_pitch = std::max(shooting_range_pitch, 1.0 * M_PI / 180);
  if (std::abs(cur_yaw - target_yaw) < shooting_range_yaw &&
      std::abs(cur_pitch - target_pitch) < shooting_range_pitch) {
    return true;
  }

  return false;
}

std::vector<Eigen::Vector3d> Solver::getArmorPositions(const Eigen::Vector3d &target_center,
                                                       const double target_yaw,
                                                       const double r1,
                                                       const double r2,
                                                       const double d_zc,
                                                       const double d_za,
                                                       const size_t armors_num) const noexcept {
  auto armor_positions = std::vector<Eigen::Vector3d>(armors_num, Eigen::Vector3d::Zero());
  // Calculate the position of each armor
  // 计算每一个装甲板那的位置
  bool is_current_pair = true;
  double r = 0., target_dz = 0.;
  for (size_t i = 0; i < armors_num; i++) 
  {
    double temp_yaw = target_yaw + i * (2 * M_PI / armors_num);
    if (armors_num == 4) 
    {
      r = is_current_pair ? r1 : r2;
      target_dz = d_zc + (is_current_pair ? 0 : d_za);
      is_current_pair = !is_current_pair;
    } 
    else 
    {
      r = r1;
      target_dz = d_zc;
    }
    armor_positions[i] =
      target_center + Eigen::Vector3d(-r * cos(temp_yaw), -r * sin(temp_yaw), target_dz);
  }
  return armor_positions;
}

// 选择最好的装甲板进行击打
int Solver::selectBestArmor(const std::vector<Eigen::Vector3d> &armor_positions,
                            const Eigen::Vector3d &target_center,
                            const double target_yaw,
                            const double target_v_yaw,
                            const size_t armors_num) const noexcept {
  // Angle between the car's center and the X-axis
  // 机器人中心与x轴之间的角度
  double alpha = std::atan2(target_center.y(), target_center.x());
  // Angle between the front of observed armor and the X-axis
  // 观察到的装甲板前部与x轴之间的角度
  double beta = target_yaw;

  // clang-format off
  // 2x2的矩阵
  Eigen::Matrix2d R_odom2center;
  Eigen::Matrix2d R_odom2armor;
  R_odom2center << std::cos(alpha), std::sin(alpha), 
                  -std::sin(alpha), std::cos(alpha);
  R_odom2armor << std::cos(beta), std::sin(beta), 
                 -std::sin(beta), std::cos(beta);
  // clang-format on
  Eigen::Matrix2d R_center2armor = R_odom2center.transpose() * R_odom2armor;

  // Equal to (alpha - beta) in most cases
  // 大多数情况下等于alpha-beta
  double decision_angle = -std::asin(R_center2armor(0, 1));

  // Angle thresh of the armor jump
  double theta = (target_v_yaw > 0 ? side_angle_ : -side_angle_) / 180.0 * M_PI;

  // Avoid the frequent switch between two armor
  // 避免在两块装甲之间频繁切换
  if (std::abs(target_v_yaw) < min_switching_v_yaw_) 
  {
    theta = 0;
  }

  double temp_angle = decision_angle + M_PI / armors_num - theta;

  if (temp_angle < 0) {
    temp_angle += 2 * M_PI;
  }

  int selected_id = static_cast<int>(temp_angle / (2 * M_PI / armors_num));
  return selected_id;
}

void Solver::calcYawAndPitch(const Eigen::Vector3d &p,
                             const std::array<double, 3> rpy,
                             double &yaw,
                             double &pitch) const noexcept {
  // Calculate yaw and pitch
  yaw = atan2(p.y(), p.x());
  pitch = atan2(p.z(), p.head(2).norm());

  if (double temp_pitch = pitch; trajectory_compensator_->compensate(p, temp_pitch)) 
  {
    // 进行角度的迭代
    pitch = temp_pitch;
  }
}

std::vector<std::pair<double, double>> Solver::getTrajectory() const noexcept {
  auto trajectory = trajectory_compensator_->getTrajectory(15, rpy_[1]);
  // Rotate
  for (auto &p : trajectory) {
    double x = p.first;
    double y = p.second;
    p.first = x * cos(rpy_[1]) + y * sin(rpy_[1]);
    p.second = -x * sin(rpy_[1]) + y * cos(rpy_[1]);
  }
  return trajectory;
}

}  // namespace pka::auto_aim
