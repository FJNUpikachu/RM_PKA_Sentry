#include "armor_solver/armor_tracker.hpp"
// std
#include <cfloat>
#include <algorithm>
#include <cmath>
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
, vel_clamp_frames(0)
, vel_clamp_linear_max(3.0)
, vel_clamp_yaw_max(20.0)
, vel_clamp_count_(0)
, last_yaw_(0) {}

void Tracker::init(const Armors::SharedPtr &armors_msg) noexcept
{
  if (armors_msg->armors.empty()) {
    return;
  }

  // 选取相对己方（target 坐标系原点）3D 距离最近的装甲板，用于多目标时的初始锁定
  double min_distance = DBL_MAX;
  tracked_armor = armors_msg->armors[0];
  for (const auto &armor : armors_msg->armors) {
    const auto &p = armor.pose.position;
    const double dist = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    if (dist < min_distance) {
      min_distance  = dist;
      tracked_armor = armor;
    }
  }

  tracked_id         = tracked_armor.number;
  tracked_robot_type = robotTypeFromId(tracked_id);

  // 判断装甲板数量
  if (isOutpostId(tracked_id, outpost_cfg_.id)) {
    tracked_armors_num = ArmorsNum::OUTPOST_3;
  } else if (tracked_id == "base") {
    tracked_armors_num = ArmorsNum::BASE_1;
  } else {
    tracked_armors_num = ArmorsNum::NORMAL_4;
  }

  initEKF(tracked_armor);
  PKA_INFO("armor_solver", "Init EKF!");

  vel_clamp_count_ = 0;
  tracker_state    = DETECTING;

  PKA_INFO("armor_solver", "Init armor: id={} type={} large_armor={} armors_num={}",
           tracked_id,
           static_cast<int>(tracked_robot_type),
           isLargeArmor(tracked_robot_type),
           static_cast<int>(tracked_armors_num));

  if (debug_tracker) {
    PKA_DEBUG("armor_solver",
              "[Tracker::init] selected armor: id={} dist_3d={:.4f}m "
              "pos=({:.3f},{:.3f},{:.3f})",
              tracked_armor.number, min_distance,
              tracked_armor.pose.position.x,
              tracked_armor.pose.position.y,
              tracked_armor.pose.position.z);
  }
}

void Tracker::update(const Armors::SharedPtr &armors_msg) noexcept
{
  bool is_outpost = isOutpostId(tracked_id, outpost_cfg_.id);

  // KF predict
  Eigen::VectorXd ekf_prediction = ekf->predict();

  if (debug_ekf) {
    PKA_DEBUG("armor_solver",
              "[EKF::predict] xc={:.3f} vx={:.3f} yc={:.3f} vy={:.3f} "
              "zc={:.3f} vz={:.3f} yaw={:.4f} vyaw={:.4f} r={:.3f} d_zc={:.3f}",
              ekf_prediction(0), ekf_prediction(1),
              ekf_prediction(2), ekf_prediction(3),
              ekf_prediction(4), ekf_prediction(5),
              ekf_prediction(6), ekf_prediction(7),
              ekf_prediction(8), ekf_prediction(9));
  }

  // ---------- 跨 ID 最近目标切换（仅在 TRACKING 或 TEMP_LOST 时考虑）----------
  bool switched = false;
  bool matched = false;
  if ((tracker_state == TRACKING || tracker_state == TEMP_LOST) && !armors_msg->armors.empty()) {
    // 当前跟踪目标的预测位置（用于距离比较）
    Eigen::Vector3d current_armor_pos = getArmorPositionFromState(ekf_prediction);
    double current_dist_sq = current_armor_pos.squaredNorm();

    std::string best_id;
    Armor best_armor;
    double best_dist_sq = std::numeric_limits<double>::max();

    for (const auto &armor : armors_msg->armors) {
      if (armor.number == tracked_id) continue;  // 跳过当前跟踪目标

      const Eigen::Vector3d pos(armor.pose.position.x,
                                 armor.pose.position.y,
                                 armor.pose.position.z);
      double dist_sq = pos.squaredNorm();
      if (dist_sq < best_dist_sq) {
        best_dist_sq = dist_sq;
        best_id = armor.number;
        best_armor = armor;
      }
    }

    const double switch_ratio = 0.7;  // 候选比当前目标近 30% 以上即切换，可调
    if (best_dist_sq < DBL_MAX && best_dist_sq < current_dist_sq * switch_ratio) {
      PKA_INFO("armor_solver",
               "Switching target from %s (dist %.2fm) to %s (dist %.2fm)",
               tracked_id.c_str(), std::sqrt(current_dist_sq),
               best_id.c_str(), std::sqrt(best_dist_sq));

      // 切换到新目标
      tracked_id = best_id;
      tracked_robot_type = robotTypeFromId(tracked_id);
      initEKF(best_armor);                               // 重新初始化 EKF
      tracker_state = DETECTING;                         // 重新进入检测阶段
      detect_count_ = 0;
      lost_count_ = 0;
      vel_clamp_count_ = 0;                              // 重置速度限幅计数

      // 更新装甲板数量
      if (isOutpostId(tracked_id, outpost_cfg_.id)) {
        tracked_armors_num = ArmorsNum::OUTPOST_3;
      } else if (tracked_id == "base") {
        tracked_armors_num = ArmorsNum::BASE_1;
      } else {
        tracked_armors_num = ArmorsNum::NORMAL_4;
      }

      // 用当前观测进行一次 EKF 更新，使状态立即对准新目标
      const auto &p = best_armor.pose.position;
      double measured_yaw = orientationToYaw(best_armor.pose.orientation);
      measurement = Eigen::Vector4d(p.x, p.y, p.z, measured_yaw);
      target_state = ekf->update(measurement);

      // 更新 is_outpost 标志（因为目标类型可能改变）
      is_outpost = isOutpostId(tracked_id, outpost_cfg_.id);

      // 标记已切换，直接跳过后面的同 ID 匹配逻辑
      switched = true;
      matched = true;        // 让状态机认为本次有匹配（进入 DETECTING 后能计数）
    }
  }

  // 如果发生了切换，直接跳转到状态机部分（跳过原有的同 ID 匹配流程）
  if (switched) {
    goto after_matching;
  }

  // ---------- 原有的同 ID 匹配逻辑 ----------
  target_state = ekf_prediction;

  if (!armors_msg->armors.empty()) {
    Armor same_id_armor;
    int same_id_armors_count = 0;

    // predicted_position 仅用于地面兵种的 position 差值排序
    auto predicted_position = is_outpost ? Eigen::Vector3d::Zero()
                                         : getArmorPositionFromState(ekf_prediction);
    double min_sort_key     = DBL_MAX;
    double yaw_diff         = DBL_MAX;

    for (const auto &armor : armors_msg->armors) {
      if (armor.number != tracked_id) continue;

      same_id_armor = armor;
      same_id_armors_count++;

      const auto p = armor.pose.position;
      const Eigen::Vector3d position_vec(p.x, p.y, p.z);

      double sort_key      = DBL_MAX;
      double local_yaw_diff = DBL_MAX;

      if (!is_outpost) {
        sort_key       = (predicted_position - position_vec).norm();
        local_yaw_diff = std::abs(orientationToYaw(armor.pose.orientation) - ekf_prediction(6));
      } else {
        local_yaw_diff = std::abs(orientationToYaw(armor.pose.orientation) - ekf_prediction(6));
        sort_key       = local_yaw_diff;
      }

      if (debug_tracker) {
        PKA_DEBUG("armor_solver",
                  "[Tracker::match] candidate id={} pos=({:.3f},{:.3f},{:.3f}) "
                  "sort_key={:.4f} yaw_diff={:.4f}",
                  armor.number, p.x, p.y, p.z, sort_key, local_yaw_diff);
      }

      if (sort_key < min_sort_key) {
        min_sort_key  = sort_key;
        yaw_diff      = local_yaw_diff;
        tracked_armor = armor;
        if (isOutpostId(tracked_id, outpost_cfg_.id)) {
          tracked_armors_num = ArmorsNum::OUTPOST_3;
        } else if (tracked_id == "base") {
          tracked_armors_num = ArmorsNum::BASE_1;
        } else {
          tracked_armors_num = ArmorsNum::NORMAL_4;
        }
      }
    }

    if (debug_tracker) {
      PKA_DEBUG("armor_solver",
                "[Tracker::match] best: same_id_count={} sort_key={:.4f} "
                "yaw_diff={:.4f} | thres: dist={:.3f} yaw={:.3f}",
                same_id_armors_count, min_sort_key, yaw_diff,
                max_match_distance_, max_match_yaw_diff_);
    }

    bool match_ok = false;
    if (is_outpost) {
      match_ok = outpostIsYawMatched(
        orientationToYaw(tracked_armor.pose.orientation),
        ekf_prediction(6),
        outpost_max_match_yaw_diff_);
    } else {
      match_ok = (min_sort_key < max_match_distance_) && (yaw_diff < max_match_yaw_diff_);
    }

    if (match_ok) {
      matched = true;
      const auto p = tracked_armor.pose.position;
      double measured_yaw = orientationToYaw(tracked_armor.pose.orientation);
      measurement = Eigen::Vector4d(p.x, p.y, p.z, measured_yaw);

      if (debug_ekf) {
        PKA_DEBUG("armor_solver",
                  "[EKF::measurement] xa={:.3f} ya={:.3f} za={:.3f} yaw={:.4f}",
                  measurement(0), measurement(1), measurement(2), measurement(3));
      }

      target_state = ekf->update(measurement);

      if (debug_ekf) {
        PKA_DEBUG("armor_solver",
                  "[EKF::update] xc={:.3f} vx={:.3f} yc={:.3f} vy={:.3f} "
                  "zc={:.3f} vz={:.3f} yaw={:.4f} vyaw={:.4f} r={:.3f} d_zc={:.3f}",
                  target_state(0), target_state(1),
                  target_state(2), target_state(3),
                  target_state(4), target_state(5),
                  target_state(6), target_state(7),
                  target_state(8), target_state(9));
      }

      if (!is_outpost && vel_clamp_count_ < vel_clamp_frames) {
        Eigen::VectorXd state_before_clamp = target_state;
        clampVelocity(target_state);
        ekf->setState(target_state);
        vel_clamp_count_++;
        //std::cout<<1<<std::endl;
        if (debug_ekf) {
          PKA_DEBUG("armor_solver",
                    "[EKF::vel_clamp] [{}/{}] "
                    "vx: {:.3f}->{:.3f}  vy: {:.3f}->{:.3f}  "
                    "vz: {:.3f}->{:.3f}  vyaw: {:.3f}->{:.3f}",
                    vel_clamp_count_, vel_clamp_frames,
                    state_before_clamp(1), target_state(1),
                    state_before_clamp(3), target_state(3),
                    state_before_clamp(5), target_state(5),
                    state_before_clamp(7), target_state(7));
        }
      }
    }
    else if (!is_outpost && same_id_armors_count == 1 && yaw_diff >= max_match_yaw_diff_) {
      if (debug_tracker) {
        PKA_DEBUG("armor_solver",
                  "[Tracker::ArmorJump] yaw_diff={:.4f} >= thres={:.3f}, triggering jump",
                  yaw_diff, max_match_yaw_diff_);
      }
      handleArmorJump(same_id_armor);
    } else {
      if (debug_tracker) {
        PKA_DEBUG("armor_solver",
                  "[Tracker::match] NO MATCH: sort_key={:.4f} yaw_diff={:.4f}",
                  min_sort_key, yaw_diff);
      }
    }
  } else {
    if (debug_tracker) {
      PKA_DEBUG("armor_solver", "[Tracker::update] armors_msg is empty, using EKF prediction");
    }
  }

after_matching:
  clampTargetState(is_outpost);

  // Tracking state machine
  if (tracker_state == DETECTING) {
    if (matched) {
      detect_count_++;
      if (detect_count_ > tracking_thres) {
        detect_count_ = 0;
        tracker_state = TRACKING;
        PKA_DEBUG("armor_solver", "Tracker state: TRACKING {}", tracked_id);
      } else {
        if (debug_tracker) {
          PKA_DEBUG("armor_solver",
                    "[Tracker::state] DETECTING id={} detect_count={}/{}",
                    tracked_id, detect_count_, tracking_thres);
        }
      }
    } else {
      detect_count_ = 0;
      tracker_state = LOST;
      PKA_DEBUG("armor_solver", "Tracker state: LOST {}", tracked_id);
    }
  } else if (tracker_state == TRACKING) {
    if (!matched) {
      tracker_state = TEMP_LOST;
      lost_count_++;
      PKA_DEBUG("armor_solver", "Tracker state: TEMP_LOST {}", tracked_id);
    } else {
      if (debug_tracker) {
        PKA_DEBUG("armor_solver",
                  "[Tracker::state] TRACKING id={} matched",
                  tracked_id);
      }
    }
  } else if (tracker_state == TEMP_LOST) {
    if (!matched) {
      lost_count_++;
      if (lost_count_ > lost_thres) {
        lost_count_   = 0;
        tracker_state = LOST;
        PKA_DEBUG("armor_solver", "Tracker state: LOST {}", tracked_id);
      } else {
        if (debug_tracker) {
          PKA_DEBUG("armor_solver",
                    "[Tracker::state] TEMP_LOST id={} lost_count={}/{}",
                    tracked_id, lost_count_, lost_thres);
        }
      }
    } else {
      tracker_state = TRACKING;
      lost_count_   = 0;
      PKA_DEBUG("armor_solver", "Tracker state: TRACKING {}", tracked_id);
    }
  }
}

void Tracker::initEKF(const Armor &a) noexcept
{
  last_yaw_ = 0;

  double xa  = a.pose.position.x;
  double ya  = a.pose.position.y;
  double za  = a.pose.position.z;
  double yaw = orientationToYaw(a.pose.orientation);

  // 10-D 状态：[xc, vx, yc, vy, zc, vz, yaw, v_yaw, r, d_zc]（地面兵种和前哨站通用）
  target_state = Eigen::VectorXd::Zero(X_N);

  double r  = 0.26;
  double xc = xa + r * cos(yaw);
  double yc = ya + r * sin(yaw);
  double zc = za;

  d_za = 0, d_zc = 0, another_r = r;

  target_state << xc, 0, yc, 0, zc, 0, yaw, 0, r, d_zc;
  ekf->setState(target_state);

  if (debug_ekf) {
    PKA_DEBUG("armor_solver",
              "[EKF::initEKF] armor=({:.3f},{:.3f},{:.3f}) yaw={:.4f} r={:.3f} "
              "-> center=({:.3f},{:.3f},{:.3f})",
              xa, ya, za, yaw, r, xc, yc, zc);
  }
}

void Tracker::handleArmorJump(const Armor &current_armor) noexcept
{
  double last_yaw = target_state(6);
  double yaw      = orientationToYaw(current_armor.pose.orientation);

  if (debug_tracker) {
    PKA_DEBUG("armor_solver",
              "[Tracker::ArmorJump] last_yaw={:.4f} new_yaw={:.4f} diff={:.4f}",
              last_yaw, yaw, std::abs(yaw - last_yaw));
  }

  if (abs(yaw - last_yaw) > 0.4) {
    target_state(6) = yaw;
    if (tracked_armors_num == ArmorsNum::NORMAL_4) {
      d_za = target_state(4) + target_state(9) - current_armor.pose.position.z;
      std::swap(target_state(8), another_r);
      d_zc = d_zc == 0 ? -d_za : 0;
      target_state(9) = d_zc;

      if (debug_tracker) {
        PKA_DEBUG("armor_solver",
                  "[Tracker::ArmorJump] swap r: r={:.3f} another_r={:.3f} d_za={:.3f} d_zc={:.3f}",
                  target_state(8), another_r, d_za, d_zc);
      }
    }
    PKA_DEBUG("armor_solver", "Armor Jump!");
  }

  auto p = current_armor.pose.position;
  Eigen::Vector3d current_p(p.x, p.y, p.z);
  Eigen::Vector3d infer_p = getArmorPositionFromState(target_state);

  if ((current_p - infer_p).norm() > max_match_distance_) {
    d_zc = 0;
    double r        = target_state(8);
    target_state(0) = p.x + r * cos(yaw);
    target_state(1) = 0;
    target_state(2) = p.y + r * sin(yaw);
    target_state(3) = 0;
    target_state(4) = p.z;
    target_state(5) = 0;
    target_state(9) = d_zc;
    PKA_WARN("armor_solver", "State wrong!");
  }

  ekf->setState(target_state);
}

void Tracker::clampVelocity(Eigen::VectorXd &state) const noexcept
{
  const double lin_max = vel_clamp_linear_max;
  const double yaw_max = vel_clamp_yaw_max;
  state(1) = std::clamp(state(1), -lin_max, lin_max);
  state(3) = std::clamp(state(3), -lin_max, lin_max);
  state(5) = std::clamp(state(5), -lin_max, lin_max);
  state(7) = std::clamp(state(7), -yaw_max, yaw_max);
}

void Tracker::clampTargetState(bool is_outpost) noexcept
{
  if (is_outpost) {
    if (!outpost_state_clamp_enable || target_state.size() < 10 || !ekf) return;
    const Eigen::VectorXd before = target_state;
    target_state(7) = std::clamp(target_state(7), outpost_v_yaw_min, outpost_v_yaw_max);
    target_state(8) = std::clamp(target_state(8), outpost_radius_min, outpost_radius_max);
    ekf->setState(target_state);

    if (debug_ekf) {
      PKA_DEBUG("armor_solver",
                "[EKF::state_clamp outpost] "
                "v_yaw {:.3f}->{:.3f}  r {:.3f}->{:.3f}",
                before(7), target_state(7),
                before(8), target_state(8));
    }
    return;
  }

  if (!ground_state_clamp_enable || target_state.size() < 10 || !ekf) return;
  const Eigen::VectorXd before = target_state;
  target_state(7) = std::clamp(target_state(7), ground_v_yaw_min, ground_v_yaw_max);
  target_state(8) = std::clamp(target_state(8), ground_radius_min, ground_radius_max);
  target_state(9) = std::clamp(target_state(9), ground_d_zc_min,   ground_d_zc_max);
  ekf->setState(target_state);

  if (debug_ekf) {
    PKA_DEBUG("armor_solver",
              "[EKF::state_clamp ground] "
              "vyaw {:.3f}->{:.3f} r {:.3f}->{:.3f} d_zc {:.3f}->{:.3f}",
              before(7), target_state(7),
              before(8), target_state(8),
              before(9), target_state(9));
  }
}

double Tracker::orientationToYaw(const geometry_msgs::msg::Quaternion &q) noexcept
{
  tf2::Quaternion tf_q;
  tf2::fromMsg(q, tf_q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  yaw       = last_yaw_ + angles::shortest_angular_distance(last_yaw_, yaw);
  last_yaw_ = yaw;
  return yaw;
}

Eigen::Vector3d Tracker::getArmorPositionFromState(const Eigen::VectorXd &x) noexcept
{
  double xc  = x(0), yc = x(2), za = x(4) + x(9);
  double yaw = x(6), r  = x(8);
  double xa  = xc - r * cos(yaw);
  double ya  = yc - r * sin(yaw);
  return Eigen::Vector3d(xa, ya, za);
}

}  // namespace pka::auto_aim
