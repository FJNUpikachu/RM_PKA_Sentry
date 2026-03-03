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
// DBL_MAX在#include <cfloat>中的宏定义为一个非常的的双精度浮点数
#include <cfloat>
#include <memory>
#include <string>
// ros2
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// third party
#include <angles/angles.h>
// project
#include "rm_utils/pkaLoggerCenter.hpp"

namespace pka::auto_aim {
  // 跟踪器构造函数
Tracker::Tracker(double max_match_distance, double max_match_yaw_diff)
: tracker_state(LOST)
, tracked_id(std::string(""))
, measurement(Eigen::VectorXd::Zero(4))
, target_state(Eigen::VectorXd::Zero(9))
, max_match_distance_(max_match_distance)
, max_match_yaw_diff_(max_match_yaw_diff)
, detect_count_(0)
, lost_count_(0)
, last_yaw_(0) {}

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
    // 选取离图像中心最近的装甲板
    if (armor.distance_to_image_center < min_distance) 
    {
      min_distance = armor.distance_to_image_center;
      tracked_armor = armor;
    }
  }

  // 初始化EKF
  // 把选择的装甲板放入EKF初始化
  initEKF(tracked_armor);
  PKA_INFO("armor_solver", "Init EKF!");

  // tracked_id即为装甲板贴纸的标签
  tracked_id = tracked_armor.number;
  // tracker_state 为跟踪器状态
  tracker_state = DETECTING;
  
  // 判断装甲板类型
  if (tracked_armor.type == "large" && (tracked_id == "3" || tracked_id == "4" || tracked_id == "5")) 
  {
    // 装甲板数量：2
    tracked_armors_num = ArmorsNum::BALANCE_2;
  }
  else if (tracked_id == "outpost") 
  {
    // 装甲板数量：3
    tracked_armors_num = ArmorsNum::OUTPOST_3;
  } 
  else 
  {
    // 装甲板数量：4
    tracked_armors_num = ArmorsNum::NORMAL_4;
  }
}

// 更新跟踪器
void Tracker::update(const Armors::SharedPtr &armors_msg) noexcept 
{
  // KF predict
  Eigen::VectorXd ekf_prediction = ekf->predict();

  bool matched = false;
  // Use KF prediction as default target state if no matched armor is found
  // 在没有匹配的装甲板时，使用KF的预测作为默认的目标状态
  target_state = ekf_prediction;

  if (!armors_msg->armors.empty()) 
  {
    // Find the closest armor with the same id
    // 寻找相同标签贴纸且最近的装甲板
    Armor same_id_armor;
    int same_id_armors_count = 0;
    // 获得装甲板x y z方向的位姿
    auto predicted_position = getArmorPositionFromState(ekf_prediction);
    double min_position_diff = DBL_MAX;
    double yaw_diff = DBL_MAX;
    // 遍历装甲板信息
    for (const auto &armor : armors_msg->armors) 
    {
      // Only consider armors with the same id
      // 只考虑具有相同标签贴纸的装甲板
      if (armor.number == tracked_id) 
      {
        same_id_armor = armor;
        same_id_armors_count++;
        // 观测一下count是否++
        std::cout<<"same_id_armors:"<<same_id_armors_count<<std::endl;
        // Calculate the difference between the predicted position and the
        // current armor position
        // 计算预测位置与当前装甲板位置的差异
        auto p = armor.pose.position;
        // 三维列向量
        // position_vec[0] = p.x
        // position_vec[1] = p.y
        // position_vec[2] = p.z
        Eigen::Vector3d position_vec(p.x, p.y, p.z);
        // 计算两次位置的distance即为position_diff
        double position_diff = (predicted_position - position_vec).norm();
        if (position_diff < min_position_diff) 
        {
          // Find the closest armor
          // 寻找距离最近的装甲板
          min_position_diff = position_diff;
          // 计算两次位置的yaw角差值，并取绝对值
          yaw_diff = abs(orientationToYaw(armor.pose.orientation) - ekf_prediction(6));
          tracked_armor = armor;
          // Update tracked armor type
          // 更新跟踪的装甲板类型
          if (tracked_armor.type == "large" && (tracked_id == "3" || tracked_id == "4" || tracked_id == "5")) 
          {
            tracked_armors_num = ArmorsNum::BALANCE_2;
          } 
          else if (tracked_id == "outpost") 
          {
            tracked_armors_num = ArmorsNum::OUTPOST_3;
          } 
          else 
          {
            tracked_armors_num = ArmorsNum::NORMAL_4;
          }
        }
      }
    }

    // Check if the distance and yaw difference of closest armor are within the threshold
    // 检查最近装甲板的距离和偏航差是否在阈值范围内
    if (min_position_diff < max_match_distance_ && yaw_diff < max_match_yaw_diff_) 
    {
      // Matched armor found
      // 匹配的装甲板被找到
      matched = true;
      auto p = tracked_armor.pose.position;
      // Update EKF
      // 更新EKF
      double measured_yaw = orientationToYaw(tracked_armor.pose.orientation);
      // 四行一列
      measurement = Eigen::Vector4d(p.x, p.y, p.z, measured_yaw);
      // 更新后的状态估计
      target_state = ekf->update(measurement);
    } 
    else if (same_id_armors_count == 1 && yaw_diff > max_match_yaw_diff_) 
    {
      // Matched armor not found, but there is only one armor with the same id
      // and yaw has jumped, take this case as the target is spinning and armor jumped
      // 未找到匹配的护甲，但只有一个装甲板具有相同的标签贴纸，yaw角已经跳跃，以这种情况为例，目标正在旋转，装甲板跳跃
      handleArmorJump(same_id_armor);
    } 
    else 
    {
      // 没有找到匹配的装甲板
      // No matched armor found
      PKA_WARN("armor_solver", "No matched armor found!");
    }
  }

  // Prevent radius from spreading
  // 防止半径扩散
  if (target_state(8) < 0.12) 
  {
    target_state(8) = 0.12;
    ekf->setState(target_state);
  } 
  else if (target_state(8) > 0.4) 
  {
    target_state(8) = 0.4;
    ekf->setState(target_state);
  }

  // Tracking state machine
  // 跟踪器的状态
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
      lost_count_ = 0;
      PKA_DEBUG("armor_solver", "Tracker state: TRACKING {}", tracked_id);
    }
  }
}

void Tracker::initEKF(const Armor &a) noexcept 
{
  //xa : x_armor
  // 装甲板的x、y、z分量值
  double xa = a.pose.position.x;
  double ya = a.pose.position.y;
  double za = a.pose.position.z;
  last_yaw_ = 0; 
  // 计算装甲板的yaw角（将四元数转换成yaw角）
  double yaw = orientationToYaw(a.pose.orientation);

  // Set initial position at 0.2m behind the target
  // 初始位置设置在目标后方0.2m处

  // 初始化元素为0的列向量长度为X_N,存储目标状态
  target_state = Eigen::VectorXd::Zero(X_N);
  
  //机器人半径
  double r = 0.26;
  double xc = xa + r * cos(yaw);
  double yc = ya + r * sin(yaw);
  // 机器人中心的z即为装甲板的z
  double zc = za;

  // 初始化d_za和d_zc为0，并且初始化另一对装甲板组成的半径another_r也为r
  d_za = 0, d_zc = 0, another_r = r;
  /*
  target_state向量
  target_state[0]=xc
  target_state[1]=0(v_x)
  target_state[2]=yc
  target_state[3]=0(v_y)
  target_state[4]=zc
  target_state[5]=0(v_z)
  target_state[6]=yaw
  target_state[7]=0(v_yaw)
  target_state[8]=r
  target_state[9]=d_zc
  */

  target_state << xc, 0, yc, 0, zc, 0, yaw, 0, r, d_zc;

  // 设置初始状态
  ekf->setState(target_state);
}

// 处理装甲板跳跃的情况
void Tracker::handleArmorJump(const Armor &current_armor) noexcept 
{
  double last_yaw = target_state(6);
  // 计算最近一块装甲板的yaw角
  double yaw = orientationToYaw(current_armor.pose.orientation);

  // 如果两次yaw角差值大于0.4弧度
  if (abs(yaw - last_yaw) > 0.4) 
  {
    // Armor angle also jumped, take this case as target spinning
    // 装甲板角度发生跳变，此时目标在旋转
    target_state(6) = yaw;
    // Only 4 armors has 2 radius and height
    // 只有4块装甲板具有两个半径和高度
    if (tracked_armors_num == ArmorsNum::NORMAL_4) 
    {
      d_za = target_state(4) + target_state(9) - current_armor.pose.position.z;
      // 进行半径交换
      std::swap(target_state(8), another_r);
      d_zc = d_zc == 0 ? -d_za : 0;
      target_state(9) = d_zc;
    }
    PKA_DEBUG("armor_solver", "Armor Jump!");
  }

  auto p = current_armor.pose.position;
  Eigen::Vector3d current_p(p.x, p.y, p.z);
  // 获得目标装甲板的xa、ya、za
  Eigen::Vector3d infer_p = getArmorPositionFromState(target_state);

  if ((current_p - infer_p).norm() > max_match_distance_) 
  {
    // If the distance between the current armor and the inferred armor is too
    // large, the state is wrong, reset center position and velocity in the
    // state
    // 如果当前的装甲板与推断出来的装甲板的距离过大，则state错误，重置state的中心位置和速度
    d_zc = 0;
    double r = target_state(8);
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

// 用于将四元数转换为yaw角
double Tracker::orientationToYaw(const geometry_msgs::msg::Quaternion &q) noexcept 
{
  // Get armor yaw
  // 获得装甲板的yaw
  tf2::Quaternion tf_q;
  // 将q转化为tf_q
  tf2::fromMsg(q, tf_q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  // Make yaw change continuous (-pi~pi to -inf~inf)
  // 使偏航变化连续（-pi~pi 更改为 -inf~inf）

  // 计算last_yaw_ 到yaw的最短旋转角度
  yaw = last_yaw_ + angles::shortest_angular_distance(last_yaw_, yaw);
  last_yaw_ = yaw;
  return yaw;
}

// 返回一个三维列向量即3x1的矩阵，包括装甲板x y z的姿态
Eigen::Vector3d Tracker::getArmorPositionFromState(const Eigen::VectorXd &x) noexcept 
{
  // Calculate predicted position of the current armor
  // 计算当前装甲板的预测位置
  double xc = x(0), yc = x(2), za = x(4) + x(9);
  double yaw = x(6), r = x(8);
  double xa = xc - r * cos(yaw);
  double ya = yc - r * sin(yaw);
  return Eigen::Vector3d(xa, ya, za);
}

}  // namespace pka::auto_aim
