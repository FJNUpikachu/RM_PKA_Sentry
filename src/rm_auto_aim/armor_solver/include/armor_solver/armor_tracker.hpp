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
#include "armor_solver/outpost_solver.hpp"

namespace pka::auto_aim {

// -------------------------------------------------------
// 兵种枚举
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

inline bool isLargeArmor(RobotType type) {
  return type == RobotType::HERO_1 || type == RobotType::BASE;
}

// -------------------------------------------------------
// 装甲板数量枚举（与建模几何一致）
// BASE 仅 1 块大装甲；HERO 等为 4；前哨站为 3
// -------------------------------------------------------
enum class ArmorsNum { BASE_1 = 1, OUTPOST_3 = 3, NORMAL_4 = 4 };

class Tracker {
public:
  Tracker(double max_match_distance, double max_match_yaw);

  using Armors = rm_interfaces::msg::Armors;
  using Armor  = rm_interfaces::msg::Armor;

  void init(const Armors::SharedPtr &armors_msg) noexcept;
  void update(const Armors::SharedPtr &armors_msg) noexcept;

  enum State {
    LOST,
    DETECTING,
    TRACKING,
    TEMP_LOST,
  } tracker_state;

  // 地面兵种与前哨站共用同一个 10-D EKF
  // 运动模型在 armor_solver_node 中根据兵种类型切换：
  //   - 地面兵种：CONSTANT_VEL_ROT
  //   - 前哨站：CONSTANT_ROTATION（中心固定，只有 yaw 匀速旋转）
  std::unique_ptr<RobotStateEKF> ekf;

  int tracking_thres;
  int lost_thres;

  Armor tracked_armor;
  std::string tracked_id;
  RobotType tracked_robot_type;
  ArmorsNum tracked_armors_num;

  Eigen::VectorXd measurement;
  Eigen::VectorXd target_state;

  double d_za, another_r;
  double d_zc;

  // -------------------------------------------------------
  // 速度限幅（Velocity Clamp）参数
  // -------------------------------------------------------
  int    vel_clamp_frames{7};
  double vel_clamp_linear_max{3.0};
  double vel_clamp_yaw_max{20.0};

  // -------------------------------------------------------
  // 分组调试开关
  // -------------------------------------------------------
  bool debug_tracker{false};
  bool debug_ekf{false};

  // -------------------------------------------------------
  // EKF 状态钳位参数（地面兵种）
  // -------------------------------------------------------
  bool   ground_state_clamp_enable{true};
  double ground_radius_min{0.18};
  double ground_radius_max{0.35};
  double ground_d_zc_min{-0.35};
  double ground_d_zc_max{0.35};
  double ground_v_yaw_min{-20.0};
  double ground_v_yaw_max{20.0};

  // -------------------------------------------------------
  // EKF 状态钳位参数（前哨站，复用 10-D 状态的同一组索引）
  // -------------------------------------------------------
  bool   outpost_state_clamp_enable{true};
  double outpost_radius_min{0.18};
  double outpost_radius_max{0.40};
  double outpost_v_yaw_min{-4.0};
  double outpost_v_yaw_max{4.0};

  // -------------------------------------------------------
  // 前哨站显示参数（id、plate_pitch）
  // -------------------------------------------------------
  OutpostParams outpost_cfg_{};

  // -------------------------------------------------------
  // 前哨站专用 tracker 匹配阈值（仅 yaw 检查）
  // 由 armor_solver_node 从 yaml outpost.tracker 注入。
  // -------------------------------------------------------
  void setOutpostTrackerGates(const double max_match_yaw_diff) noexcept
  {
    outpost_max_match_yaw_diff_ = max_match_yaw_diff;
  }

  /// 更新 EKF 预测函数的 dt 和运动模型
  void setEKFDt(double dt, MotionModel model = MotionModel::CONSTANT_VEL_ROT) noexcept
  {
    if (ekf) {
      ekf->setPredictFunc(Predict{dt, model});
    }
  }

private:
  void initEKF(const Armor &a) noexcept;
  void handleArmorJump(const Armor &a) noexcept;
  void clampVelocity(Eigen::VectorXd &state) const noexcept;
  void clampTargetState(bool is_outpost) noexcept;

  double orientationToYaw(const geometry_msgs::msg::Quaternion &q) noexcept;
  static Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd &x) noexcept;

  double max_match_distance_;
  double max_match_yaw_diff_;

  // 前哨站专用 yaw 匹配阈值（不使用 position 距离）
  double outpost_max_match_yaw_diff_{1.8};

  int detect_count_;
  int lost_count_;
  int vel_clamp_count_{0};

  double last_yaw_;
};

}  // namespace pka::auto_aim

#endif  // ARMOR_SOLVER_ARMOR_TRACKER_HPP_
