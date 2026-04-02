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

#include "armor_solver/armor_tracker.hpp"
// std
// DBL_MAX 在 #include <cfloat> 中的宏定义为一个非常大的双精度浮点数
#include <cfloat>
#include <algorithm>
#include <memory>
#include <string>
// ros2
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// third party
#include <angles/angles.h>
#include <vector>
// project
#include "rm_utils/pkaLoggerCenter.hpp"

namespace pka::auto_aim {

// -------------------------------------------------------
// 内部辅助：根据 tracked_id 更新 tracked_robot_type 和 tracked_armors_num
//
// 兵种 → 装甲板数量映射：
//   前哨站 (OUTPOST) → OUTPOST_3 (3块)
//   其余正常机器人   → NORMAL_4  (4块)
//   （BALANCE_2 为旧赛季平衡步兵，新赛季保留枚举但不应匹配到任何 id）
// -------------------------------------------------------
void Tracker::updateTrackedType() noexcept {
  tracked_robot_type = robotTypeFromId(tracked_id);

  switch (tracked_robot_type) {
    case RobotType::OUTPOST:
      tracked_armors_num = ArmorsNum::OUTPOST_3;
      break;
    default:
      tracked_armors_num = ArmorsNum::NORMAL_4;
      break;
  }
}

// 跟踪器构造函数
Tracker::Tracker(double max_match_distance, double max_match_yaw_diff)
: tracker_state(LOST)
, tracked_id(std::string(""))
, tracked_robot_type(RobotType::UNKNOWN)
, measurement(Eigen::VectorXd::Zero(4))
, target_state(Eigen::VectorXd::Zero(9))
, max_match_distance_(max_match_distance)
, max_match_yaw_diff_(max_match_yaw_diff)
, detect_count_(0)
, lost_count_(0)
, vel_clamp_count_(0)
, last_yaw_(0)
// 限幅参数默认值，由外部 armor_solver_node 通过参数覆盖
// [Fix 1] vel_clamp_frames 默认设为 0（即关闭限幅），
//         避免硬编码的默认值在 yaml 未正确加载时误伤小陀螺的 v_yaw 收敛。
//         实际生产使用时由 yaml vel_clamp.frames 控制。
, vel_clamp_frames(0)
// [Fix 2] vel_clamp_linear_max 改为合理的线速度保护上限（3.0 m/s）
, vel_clamp_linear_max(3.0)
// [Fix 3] vel_clamp_yaw_max 改为能覆盖最大陀螺转速的上限（20.0 rad/s）。
//         原值 4.0 rad/s 低于小陀螺正常工作转速（5~15 rad/s），
//         导致 EKF 的 v_yaw 分量在初始化后被持续截断，
//         造成"无角速度、线速度方向随装甲板旋转"的异常现象。
, vel_clamp_yaw_max(20.0) {}

void Tracker::init(const Armors::SharedPtr &armors_msg) noexcept
{
  if (armors_msg->armors.empty())
  {
    return;
  }

  /***************选板逻辑***************/
  // 只需选择最靠近图像中心的装甲板
  // Simply choose the armor that is closest to image center
  double min_distance = DBL_MAX;
  tracked_armor = armors_msg->armors[0];
  for (const auto &armor : armors_msg->armors)
  {
    if (armor.distance_to_image_center < min_distance)
    {
      min_distance = armor.distance_to_image_center;
      tracked_armor = armor;
    }
  }

  // 初始化EKF
  initEKF(tracked_armor);
  PKA_INFO("armor_solver", "Init EKF!");

  // 重置速度限幅计数器
  // 每次 tracker 从 LOST 重新识别时归零，保证新一轮限幅从第 0 帧开始生效
  // handleArmorJump 不调用 init()，所以装甲板跳跃不会误触发
  vel_clamp_count_ = 0;

  // tracked_id 即为装甲板贴纸的标签，同时解析兵种类型和装甲板数量
  tracked_id = tracked_armor.number;
  updateTrackedType();
  tracker_state = DETECTING;

  PKA_INFO("armor_solver", "Init armor: id={} type={} large_armor={} armors_num={}",
           tracked_id,
           static_cast<int>(tracked_robot_type),
           isLargeArmor(tracked_robot_type),
           static_cast<int>(tracked_armors_num));
}

// 更新跟踪器
void Tracker::update(const Armors::SharedPtr &armors_msg) noexcept
{
  // KF predict
  Eigen::VectorXd ekf_prediction = ekf->predict();

  bool matched = false;
  // Use KF prediction as default target state if no matched armor is found
  // 在没有匹配的装甲板时，使用 KF 的预测作为默认的目标状态
  target_state = ekf_prediction;

  if (!armors_msg->armors.empty())
  {
    // Find the closest armor with the same id
    // 寻找相同标签贴纸且最近的装甲板
    Armor same_id_armor;
    std::vector<Armor> same_id_armors;
    int same_id_armors_count = 0;
    auto predicted_position  = getArmorPositionFromState(ekf_prediction);
    double min_position_diff = DBL_MAX;
    double yaw_diff          = DBL_MAX;

    for (const auto &armor : armors_msg->armors)
    {
      if (armor.number == tracked_id)
      {
        same_id_armor = armor;
        same_id_armors_count++;
        same_id_armors.emplace_back(same_id_armor);

        auto p = armor.pose.position;
        Eigen::Vector3d position_vec(p.x, p.y, p.z);
        double position_diff = (predicted_position - position_vec).norm();
        if (position_diff < min_position_diff)
        {
          min_position_diff = position_diff;
          yaw_diff = abs(orientationToYaw(armor.pose.orientation) - ekf_prediction(6));
          tracked_armor = armor;
          // id 不变时兵种类型不会改变，无需重新调用 updateTrackedType()
        }
      }
    }

    // Check if the distance and yaw difference of closest armor are within the threshold
    if (min_position_diff < max_match_distance_ && yaw_diff < max_match_yaw_diff_)
    {
      // Matched armor found
      matched = true;
      auto p  = tracked_armor.pose.position;
      double measured_yaw = orientationToYaw(tracked_armor.pose.orientation);
      measurement  = Eigen::Vector4d(p.x, p.y, p.z, measured_yaw);
      target_state = ekf->update(measurement);

      // ── 速度限幅（Velocity Clamp）──────────────────────────────────────────
      // EKF 初始化后协方差处于初始状态，前几帧 update() 时若观测位置与
      // 初始位置存在偏差（坐标系切换抖动/tf2 时间戳误差等），EKF 会产生
      // 极大的速度修正量，导致云台抽风。
      //
      // 策略：在 vel_clamp_frames 帧内，对 update() 输出的速度分量
      //   (v_x:1, v_y:3, v_z:5, v_yaw:7) 做硬限幅，超出阈值则截断。
      //   限幅后将截断后的状态写回 EKF，使后续 predict() 基于合理速度传播。
      //
      // 关键：限幅只在 matched 分支内、update() 之后执行，
      //   predict() 的自由传播完全不受影响，EKF 协方差可以正常扩张；
      //   vel_clamp_yaw_max 必须高于最大陀螺转速，
      //   只截断初始化瞬间的异常冲量，不会压死角速度收敛。
      // ───────────────────────────────────────────────────────────────────────
      if (vel_clamp_count_ < vel_clamp_frames)
      {
        clampVelocity(target_state);
        ekf->setState(target_state);
        vel_clamp_count_++;
        PKA_DEBUG("armor_solver",
                  "Vel clamp active [{}/{}]: vx={:.3f} vy={:.3f} vz={:.3f} vyaw={:.3f}",
                  vel_clamp_count_, vel_clamp_frames,
                  target_state(1), target_state(3), target_state(5), target_state(7));
      }
    }
    else if (same_id_armors_count == 1 && yaw_diff >= max_match_yaw_diff_)
    {
      std::cout << "yaw_diff: " << yaw_diff << std::endl;
      // Matched armor not found, but yaw has jumped — target is spinning
      handleArmorJump(same_id_armor);
    }
    else
    {
      std::cout << "No matched armor found, yaw_diff: " << yaw_diff
                << ", min_position_diff: " << min_position_diff << std::endl;
    }
  }

  // Prevent radius from spreading
  // 防止半径扩散
  if (target_state(8) < 0.18)
  {
    target_state(8) = 0.18;
    ekf->setState(target_state);
  }
  else if (target_state(8) > 0.35)
  {
    target_state(8) = 0.35;
    ekf->setState(target_state);
  }

  // Tracking state machine
  if (tracker_state == DETECTING)
  {
    if (matched)
    {
      detect_count_++;
      if (detect_count_ > tracking_thres)
      {
        detect_count_ = 0;
        tracker_state = TRACKING;
        PKA_DEBUG("armor_solver", "Tracker state: TRACKING {}", tracked_id);
      }
    }
    else
    {
      detect_count_ = 0;
      tracker_state = LOST;
      PKA_DEBUG("armor_solver", "Tracker state: LOST {}", tracked_id);
    }
  }
  else if (tracker_state == TRACKING)
  {
    if (!matched)
    {
      tracker_state = TEMP_LOST;
      lost_count_++;
      PKA_DEBUG("armor_solver", "Tracker state: TEMP_LOST {}", tracked_id);
    }
  }
  else if (tracker_state == TEMP_LOST)
  {
    if (!matched)
    {
      lost_count_++;
      if (lost_count_ > lost_thres)
      {
        lost_count_ = 0;
        tracker_state = LOST;
        PKA_DEBUG("armor_solver", "Tracker state: LOST {}", tracked_id);
      }
    }
    else
    {
      tracker_state = TRACKING;
      lost_count_   = 0;
      PKA_DEBUG("armor_solver", "Tracker state: TRACKING {}", tracked_id);
    }
  }
}

void Tracker::initEKF(const Armor &a) noexcept
{
  // xa : x_armor
  double xa = a.pose.position.x;
  double ya = a.pose.position.y;
  double za = a.pose.position.z;
  last_yaw_ = 0;
  double yaw = orientationToYaw(a.pose.orientation);

  target_state = Eigen::VectorXd::Zero(X_N);

  // 机器人半径
  double r  = 0.26;
  double xc = xa + r * cos(yaw);
  double yc = ya + r * sin(yaw);
  double zc = za;

  d_za = 0, d_zc = 0, another_r = r;

  /*
  target_state 向量
  [0]=xc  [1]=v_x
  [2]=yc  [3]=v_y
  [4]=zc  [5]=v_z
  [6]=yaw [7]=v_yaw
  [8]=r   [9]=d_zc
  */
  target_state << xc, 0, yc, 0, zc, 0, yaw, 0, r, d_zc;

  ekf->setState(target_state);
}

// 处理装甲板跳跃的情况
void Tracker::handleArmorJump(const Armor &current_armor) noexcept
{
  double last_yaw = target_state(6);
  double yaw      = orientationToYaw(current_armor.pose.orientation);

  if (abs(yaw - last_yaw) > 0.4)
  {
    target_state(6) = yaw;
    if (tracked_armors_num == ArmorsNum::NORMAL_4)
    {
      d_za = target_state(4) + target_state(9) - current_armor.pose.position.z;
      std::swap(target_state(8), another_r);
      d_zc = d_zc == 0 ? -d_za : 0;
      target_state(9) = d_zc;
    }
    PKA_DEBUG("armor_solver", "Armor Jump!");
  }

  auto p = current_armor.pose.position;
  Eigen::Vector3d current_p(p.x, p.y, p.z);
  Eigen::Vector3d infer_p = getArmorPositionFromState(target_state);

  if ((current_p - infer_p).norm() > max_match_distance_)
  {
    d_zc = 0;
    double r        = target_state(8);
    target_state(0) = p.x + r * cos(yaw);  // xc
    target_state(1) = 0;                   // vxc
    target_state(2) = p.y + r * sin(yaw);  // yc
    target_state(3) = 0;                   // vyc
    target_state(4) = p.z;                 // zc
    target_state(5) = 0;                   // vzc
    target_state(9) = d_zc;                // d_zc
    PKA_WARN("armor_solver", "State wrong!");
  }

  ekf->setState(target_state);
}

void Tracker::clampVelocity(Eigen::VectorXd &state) const noexcept
{
  // target_state 各速度分量索引：
  //   state(1) = v_x,  state(3) = v_y,
  //   state(5) = v_z,  state(7) = v_yaw
  const double lin_max = vel_clamp_linear_max;
  const double yaw_max = vel_clamp_yaw_max;

  state(1) = std::clamp(state(1), -lin_max, lin_max);  // v_x
  state(3) = std::clamp(state(3), -lin_max, lin_max);  // v_y
  state(5) = std::clamp(state(5), -lin_max, lin_max);  // v_z
  state(7) = std::clamp(state(7), -yaw_max, yaw_max);  // v_yaw
}

// 用于将四元数转换为 yaw 角
double Tracker::orientationToYaw(const geometry_msgs::msg::Quaternion &q) noexcept
{
  tf2::Quaternion tf_q;
  tf2::fromMsg(q, tf_q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  // Make yaw change continuous (-pi~pi to -inf~inf)
  yaw       = last_yaw_ + angles::shortest_angular_distance(last_yaw_, yaw);
  last_yaw_ = yaw;
  return yaw;
}

// 返回一个三维列向量，包括装甲板 x y z 的姿态
Eigen::Vector3d Tracker::getArmorPositionFromState(const Eigen::VectorXd &x) noexcept
{
  double xc  = x(0), yc = x(2), za = x(4) + x(9);
  double yaw = x(6), r  = x(8);
  double xa  = xc - r * cos(yaw);
  double ya  = yc - r * sin(yaw);
  return Eigen::Vector3d(xa, ya, za);
}

}  // namespace pka::auto_aim