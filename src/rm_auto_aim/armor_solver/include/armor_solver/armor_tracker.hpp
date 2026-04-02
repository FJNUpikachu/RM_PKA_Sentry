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


#ifndef ARMOR_SOLVER_TRACKER_HPP_
#define ARMOR_SOLVER_TRACKER_HPP_

// std
#include <memory>
#include <string>
#include <unordered_map>
// ros2
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
// third party
#include <Eigen/Eigen>
#include <vector>
// project
#include "rm_interfaces/msg/armors.hpp"
#include "rm_interfaces/msg/target.hpp"
#include "rm_utils/math/extended_kalman_filter.hpp"
#include "armor_solver/motion_model.hpp"

namespace pka::auto_aim {

// -------------------------------------------------------
// 兵种枚举
// 与装甲板贴纸上的 number 字段一一对应：
//   "1" → HERO_1       大装甲板
//   "2" → ENGINEER_2   小装甲板
//   "3" → INFANTRY_3   小装甲板
//   "4" → INFANTRY_4   小装甲板
//   "5" → SENTRY_5     小装甲板
//   "outpost" → OUTPOST 小装甲板
//   "base"    → BASE    大装甲板
// -------------------------------------------------------
enum class RobotType {
  HERO_1     = 1,
  ENGINEER_2 = 2,
  INFANTRY_3 = 3,
  INFANTRY_4 = 4,
  SENTRY_5   = 5,
  OUTPOST    = 6,
  BASE       = 7,
  UNKNOWN    = 0,
};

// 根据装甲板贴纸 number 字符串解析对应兵种
inline RobotType robotTypeFromId(const std::string &id) {
  if (id == "1")       return RobotType::HERO_1;
  if (id == "2")       return RobotType::ENGINEER_2;
  if (id == "3")       return RobotType::INFANTRY_3;
  if (id == "4")       return RobotType::INFANTRY_4;
  if (id == "5")       return RobotType::SENTRY_5;
  if (id == "outpost") return RobotType::OUTPOST;
  if (id == "base")    return RobotType::BASE;
  return RobotType::UNKNOWN;
}

// 判断该兵种是否使用大装甲板
// 英雄(1) 和 基地(base) 使用大装甲板，其余兵种使用小装甲板
inline bool isLargeArmor(RobotType type) {
  return type == RobotType::HERO_1 || type == RobotType::BASE;
}

// 兵种 → 该机器人携带的装甲板数量
// 前哨站 3 块，其余正常机器人 4 块
inline int armorCount(RobotType type) {
  if (type == RobotType::OUTPOST) return 3;
  return 4;
}

// -------------------------------------------------------
// 枚举装甲板数量（保留，供 tracker 状态机使用）
// -------------------------------------------------------
enum class ArmorsNum { NORMAL_4 = 4, BALANCE_2 = 2, OUTPOST_3 = 3 };

class Tracker {
public:
  // 有参跟踪器构造函数
  Tracker(double max_match_distance, double max_match_yaw);

  using Armors = rm_interfaces::msg::Armors;
  using Armor  = rm_interfaces::msg::Armor;

  // 初始化跟踪器
  void init(const Armors::SharedPtr &armors_msg) noexcept;

  // 更新跟踪器
  void update(const Armors::SharedPtr &armors_msg) noexcept;

  // 枚举状态tracker_state
  /*  状态分为：
  丢失、识别中、跟踪、临时丢失
  */
  enum State {
    LOST,
    DETECTING,
    TRACKING,
    TEMP_LOST,
  } tracker_state;

  // 创建一个ekf类
  std::unique_ptr<RobotStateEKF> ekf;

  int tracking_thres;  // frame
  int lost_thres;      // second

  // 跟踪的装甲板
  Armor tracked_armor;

  // 跟踪的装甲板标签（对应 RobotType）
  std::string tracked_id;

  // 当前跟踪目标的兵种类型（由 tracked_id 解析，初始化时更新）
  RobotType tracked_robot_type;

  // 跟踪的装甲板数量
  ArmorsNum tracked_armors_num;

  // Eigen::VectorXd 用于动态表示列向量
  Eigen::VectorXd measurement;
  Eigen::VectorXd target_state;

  // To store another pair of armors message
  // 储存另一对装甲板的信息
  double d_za, another_r;

  // To store offset relative to the reference plane
  // 存储相对于参照平面的偏移
  double d_zc;

  // -------------------------------------------------------
  // 速度限幅（Velocity Clamp）参数
  // 由外部 armor_solver_node 通过 yaml 参数注入
  //
  // 设计说明：
  //   tracker 从 LOST 重新识别后，init() 将 vel_clamp_count_ 归零。
  //   此后前 vel_clamp_frames 帧内，每次 update() 有观测匹配时，
  //   对 EKF update() 输出的速度分量做硬限幅并写回 EKF。
  //   限幅仅作用于 matched 后的 update() 输出，不影响 predict()；
  //   handleArmorJump 不重置计数器，不会误触发二次限幅。
  //
  // [Fix] vel_clamp_yaw_max 必须高于最大陀螺转速（推荐 ≥ 20 rad/s），
  //       否则会压制 EKF v_yaw 收敛，导致小陀螺跟踪时角速度为零、
  //       线速度方向随装甲板旋转的异常现象。
  //   vel_clamp_frames 默认设为 0（关闭），由 yaml 按需开启。
  // -------------------------------------------------------
  int    vel_clamp_frames{0};        // 限幅持续帧数（0 = 关闭）
  double vel_clamp_linear_max{3.0};  // 线速度上限 (m/s)
  double vel_clamp_yaw_max{20.0};    // 角速度上限 (rad/s)，须 > 最大陀螺转速

private:
  void initEKF(const Armor &a) noexcept;

  void handleArmorJump(const Armor &a) noexcept;

  // 对 state 中的速度分量 (v_x, v_y, v_z, v_yaw) 做硬限幅
  void clampVelocity(Eigen::VectorXd &state) const noexcept;

  double orientationToYaw(const geometry_msgs::msg::Quaternion &q) noexcept;

  static Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd &x) noexcept;

  // 根据当前 tracked_id 更新 tracked_robot_type 和 tracked_armors_num
  void updateTrackedType() noexcept;

  double max_match_distance_;
  double max_match_yaw_diff_;

  int detect_count_;
  int lost_count_;

  // 速度限幅计数器
  // init() 里归零；handleArmorJump 不重置
  int vel_clamp_count_{0};

  double last_yaw_;
};

}  // namespace pka::auto_aim

#endif  // ARMOR_SOLVER_ARMOR_TRACKER_HPP_