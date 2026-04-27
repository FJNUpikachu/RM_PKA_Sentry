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
, vel_clamp_frames(0)
, vel_clamp_linear_max(3.0)
, vel_clamp_yaw_max(20.0)
, vel_clamp_count_(0)
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
    if (armor.distance_to_image_center < min_distance)
    {
      min_distance = armor.distance_to_image_center;
      tracked_armor = armor;
    }
  }

  // tracked_id 即为装甲板贴纸的标签，同时解析兵种类型和装甲板数量
  tracked_id         = tracked_armor.number;
  tracked_robot_type = robotTypeFromId(tracked_id);
  tracked_armors_num =
    tracked_robot_type == RobotType::OUTPOST ? ArmorsNum::OUTPOST_3 : ArmorsNum::NORMAL_4;

  // 初始化EKF
  initEKF(tracked_armor);
  PKA_INFO("armor_solver", "Init EKF!");

  // 重置速度限幅计数器
  // 每次 tracker 从 LOST 重新识别时归零，保证新一轮限幅从第 0 帧开始生效
  // handleArmorJump 不调用 init()，所以装甲板跳跃不会误触发
  vel_clamp_count_ = 0;

  tracker_state = DETECTING;

  PKA_INFO("armor_solver", "Init armor: id={} type={} large_armor={} armors_num={}",
           tracked_id,
           static_cast<int>(tracked_robot_type),
           isLargeArmor(tracked_robot_type),
           static_cast<int>(tracked_armors_num));

  // [debug_tracker] 初始化时选择的装甲板信息
  if (debug_tracker) {
    PKA_DEBUG("armor_solver",
              "[Tracker::init] selected armor: id={} dist_to_center={:.4f} "
              "pos=({:.3f}, {:.3f}, {:.3f})",
              tracked_armor.number,
              min_distance,
              tracked_armor.pose.position.x,
              tracked_armor.pose.position.y,
              tracked_armor.pose.position.z);
  }

  // [debug_ekf] 初始化后的 EKF 状态向量
  if (debug_ekf) {
    if (tracked_robot_type == RobotType::OUTPOST && target_state.size() >= X_N_OUTPOST) {
      PKA_DEBUG("armor_solver",
                "[EKF::init outpost] xc={:.3f} yc={:.3f} zc={:.3f} "
                "yaw={:.4f} v_yaw={:.4f} r={:.3f} (DETECTING, accumulating plates)",
                target_state(0), target_state(1), target_state(2),
                target_state(3), target_state(4), target_state(5));
    } else if (tracked_robot_type != RobotType::OUTPOST) {
      PKA_DEBUG("armor_solver",
                "[EKF::init] state: xc={:.3f} vx={:.3f} yc={:.3f} vy={:.3f} "
                "zc={:.3f} vz={:.3f} yaw={:.3f} vyaw={:.3f} r={:.3f} d_zc={:.3f}",
                target_state(0), target_state(1),
                target_state(2), target_state(3),
                target_state(4), target_state(5),
                target_state(6), target_state(7),
                target_state(8), target_state(9));
    }
  }
}

// 更新跟踪器
void Tracker::update(const Armors::SharedPtr &armors_msg) noexcept
{
  const bool is_outpost_now = (tracked_robot_type == RobotType::OUTPOST);

  // ── 前哨站 DETECTING 阶段（三板积累）早期返回 ──────────────────────────
  // 在 EKF 正式初始化（三板到齐 + 第四次出现）之前，
  // 跳过 EKF predict/update，只做板积累逻辑，由 outpostHandleDetecting 驱动。
  if (is_outpost_now && tracker_state == DETECTING) {
    bool has_armor = false;
    for (const auto &armor : armors_msg->armors) {
      if (armor.number == tracked_id) {
        has_armor    = true;
        tracked_armor = armor;
        break;
      }
    }
    const bool outpost_detect_debug = debug_tracker || debug_outpost;
    const auto result = outpostHandleDetecting(
      outpost_plate_buffer_, lost_thres, *outpost_ekf,
      outpost_cfg_, has_armor, tracked_armor, &target_state, outpost_detect_debug);

    clampTargetState(true);

    if (result == OutpostDetectingResult::START_TRACKING) {
      outpostComputeHeightOffsetsFromBuffer(outpost_plate_buffer_, &d_zc, &d_za);
      if (outpost_cfg_.detect_rebuild_on_invalid_height &&
          std::abs(d_za) < outpost_cfg_.detect_min_valid_dza_m)
      {
        PKA_WARN("armor_solver",
                 "Outpost init rejected: |d_za|={:.3f} < min_valid_dza={:.3f}, re-detecting",
                 std::abs(d_za), outpost_cfg_.detect_min_valid_dza_m);
        outpost_plate_buffer_.reset();
        d_zc = 0.0;
        d_za = 0.0;
        detect_count_ = 0;
        tracker_state = LOST;
        return;
      }
      detect_count_ = 0;
      tracker_state = TRACKING;
      PKA_INFO("armor_solver",
               "Outpost TRACKING: {} plates accumulated, EKF initialized (d_zc={:.3f}, d_za={:.3f})",
               OutpostPlateBuffer::NUM_PLATES, d_zc, d_za);
      if (debug_tracker) {
        PKA_DEBUG("armor_solver",
                  "[Tracker::state] DETECTING -> TRACKING (outpost 3-plate init) "
                  "center=({:.3f},{:.3f},{:.3f}) yaw={:.4f} v_yaw={:.4f} r={:.3f}",
                  target_state(0), target_state(1), target_state(2),
                  target_state(3), target_state(4), target_state(5));
      }
    } else if (result == OutpostDetectingResult::GO_LOST) {
      detect_count_ = 0;
      tracker_state = LOST;
      PKA_INFO("armor_solver", "Outpost LOST: timeout during plate accumulation");
    } else {
      if (debug_tracker || debug_outpost) {
        PKA_DEBUG("armor_solver",
                  "[Tracker::state] DETECTING (outpost accumulating) "
                  "plates={}/{} has_armor={}",
                  outpost_plate_buffer_.plates.size(),
                  OutpostPlateBuffer::NUM_PLATES,
                  has_armor);
      }
    }
    return;  // 跳过本帧的 EKF predict/update 和地面兵种状态机
  }
  // ── 前哨站 DETECTING 早期返回结束 ──────────────────────────────────────

  // KF predict（按兵种分发到对应 EKF）
  Eigen::VectorXd ekf_prediction;
  if (is_outpost_now && outpost_ekf) {
    ekf_prediction = outpost_ekf->predict();
  } else {
    ekf_prediction = ekf->predict();
  }

  // [debug_ekf] 输出 EKF predict 后的预测状态
  if (debug_ekf) {
    if (is_outpost_now && ekf_prediction.size() >= X_N_OUTPOST) {
      // 前哨站 6-D state: [xc, yc, zc, yaw, v_yaw, r]
      PKA_DEBUG("armor_solver",
                "[EKF::predict outpost] xc={:.3f} yc={:.3f} zc={:.3f} "
                "yaw={:.4f} v_yaw={:.4f} r={:.3f}",
                ekf_prediction(0), ekf_prediction(1), ekf_prediction(2),
                ekf_prediction(3), ekf_prediction(4), ekf_prediction(5));
    } else if (!is_outpost_now) {
      PKA_DEBUG("armor_solver",
                "[EKF::predict] xc={:.3f} vx={:.3f} yc={:.3f} vy={:.3f} "
                "zc={:.3f} vz={:.3f} yaw={:.4f} vyaw={:.4f} r={:.3f} d_zc={:.3f}",
                ekf_prediction(0), ekf_prediction(1),
                ekf_prediction(2), ekf_prediction(3),
                ekf_prediction(4), ekf_prediction(5),
                ekf_prediction(6), ekf_prediction(7),
                ekf_prediction(8), ekf_prediction(9));
    }
  }

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
    const bool is_outpost = is_outpost_now;
    // getArmorPositionFromState 假设 10-D 状态布局，仅供地面兵种使用。
    // 前哨站匹配由 outpostFindBestSlotMatch 处理，不使用此值。
    auto predicted_position = is_outpost ? Eigen::Vector3d::Zero()
                                         : getArmorPositionFromState(ekf_prediction);
    double min_position_diff = DBL_MAX;
    double yaw_diff          = DBL_MAX;

    // Select matching gates based on robot type
    const double gate_dist = is_outpost ? outpost_max_match_distance_ : max_match_distance_;
    const double gate_yaw  = is_outpost ? outpost_max_match_yaw_diff_ : max_match_yaw_diff_;

    for (const auto &armor : armors_msg->armors)
    {
      if (armor.number == tracked_id)
      {
        same_id_armor = armor;
        same_id_armors_count++;
        same_id_armors.emplace_back(same_id_armor);

        auto p = armor.pose.position;
        Eigen::Vector3d position_vec(p.x, p.y, p.z);
        double position_diff = DBL_MAX;
        double local_yaw_diff = DBL_MAX;

        if (!is_outpost) {
          position_diff = (predicted_position - position_vec).norm();
          local_yaw_diff = std::abs(orientationToYaw(armor.pose.orientation) - ekf_prediction(6));
        } else {
          outpostFindBestSlotMatch(ekf_prediction, armor,
                                   &position_diff, &local_yaw_diff);
        }

        // [debug_tracker] 每块同 id 装甲板的匹配距离
        if (debug_tracker) {
          PKA_DEBUG("armor_solver",
                    "[Tracker::match] candidate id={} pos=({:.3f},{:.3f},{:.3f}) "
                    "pos_diff={:.4f} yaw_diff={:.4f}",
                    armor.number, p.x, p.y, p.z,
                    position_diff,
                    local_yaw_diff);
        }

        if (position_diff < min_position_diff)
        {
          min_position_diff = position_diff;
          yaw_diff = local_yaw_diff;
          tracked_armor = armor;
          // id 不变时兵种类型不会改变，无需重新调用 updateTrackedType()
        }
      }
    }

    // [debug_tracker] 最终匹配结果汇总
    if (debug_tracker) {
      PKA_DEBUG("armor_solver",
                "[Tracker::match] best: same_id_count={} min_pos_diff={:.4f} "
                "yaw_diff={:.4f} | thres: pos={:.3f} yaw={:.3f}",
                same_id_armors_count, min_position_diff, yaw_diff,
                max_match_distance_, max_match_yaw_diff_);
    }

    // Check if the distance and yaw difference of closest armor are within the threshold
    if (min_position_diff < gate_dist && yaw_diff < gate_yaw)
    {
      // Matched armor found
      matched = true;
      auto p  = tracked_armor.pose.position;
      double measured_yaw = orientationToYaw(tracked_armor.pose.orientation);
      measurement  = Eigen::Vector4d(p.x, p.y, p.z, measured_yaw);

      // OUTPOST: prepare slot-0-equivalent EKF measurement (6-D model).
      //   - Find best matching slot k (0/1/2) by predicted position
      //   - Remap XY/yaw to slot-0 equivalent; pass raw za (EKF r_z absorbs height variation)
      if (is_outpost && outpost_ekf) {
        measurement = outpostPrepareEKFMeasurement(
          ekf_prediction, tracked_armor, measured_yaw);
      }

      // [debug_ekf] EKF update 前的测量值
      if (debug_ekf) {
        PKA_DEBUG("armor_solver",
                  "[EKF::measurement] xa={:.3f} ya={:.3f} za={:.3f} yaw={:.4f}",
                  p.x, p.y, p.z, measured_yaw);
      }

      // update：按兵种分发到对应 EKF
      if (is_outpost && outpost_ekf) {
        target_state = outpost_ekf->update(measurement);
      } else {
        target_state = ekf->update(measurement);
      }

      // [debug_ekf] EKF update 后的状态向量
      if (debug_ekf) {
        if (is_outpost && target_state.size() >= X_N_OUTPOST) {
          // 前哨站 6-D state: [xc, yc, zc, yaw, v_yaw, r]
          PKA_DEBUG("armor_solver",
                    "[EKF::update outpost] xc={:.3f} yc={:.3f} zc={:.3f} "
                    "yaw={:.4f} v_yaw={:.4f} r={:.3f}",
                    target_state(0), target_state(1), target_state(2),
                    target_state(3), target_state(4), target_state(5));
        } else if (!is_outpost) {
          PKA_DEBUG("armor_solver",
                    "[EKF::update] xc={:.3f} vx={:.3f} yc={:.3f} vy={:.3f} "
                    "zc={:.3f} vz={:.3f} yaw={:.4f} vyaw={:.4f} r={:.3f} d_zc={:.3f}",
                    target_state(0), target_state(1),
                    target_state(2), target_state(3),
                    target_state(4), target_state(5),
                    target_state(6), target_state(7),
                    target_state(8), target_state(9));
        }
      }

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
      // OUTPOST: vel_clamp 不适用前哨站 EKF
      if (!is_outpost && vel_clamp_count_ < vel_clamp_frames)
      {
        Eigen::VectorXd state_before_clamp = target_state;
        clampVelocity(target_state);
        ekf->setState(target_state);
        vel_clamp_count_++;

        // [debug_ekf] 速度限幅前后对比
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
        } else {
          // 保留原有 DEBUG（debug_ekf 关闭时也可通过原有级别查看）
          PKA_DEBUG("armor_solver",
                    "Vel clamp active [{}/{}]: vx={:.3f} vy={:.3f} vz={:.3f} vyaw={:.3f}",
                    vel_clamp_count_, vel_clamp_frames,
                    target_state(1), target_state(3), target_state(5), target_state(7));
        }
      }
    }
    // OUTPOST: do not use "armor jump" fallback.
    // Outpost has 3 slots and we already match against 3 predicted slots + remap measurement to slot0.
    // Using yaw-diff based jump here will cause frequent false jumps (and then LOST/reinit), which
    // manifests as "center point jitter" in RViz.
    else if (!is_outpost && same_id_armors_count == 1 && yaw_diff >= max_match_yaw_diff_)
    {
      // [debug_tracker] 装甲板跳跃触发
      if (debug_tracker) {
        PKA_DEBUG("armor_solver",
                  "[Tracker::ArmorJump] yaw_diff={:.4f} >= thres={:.3f}, triggering jump",
                  yaw_diff, max_match_yaw_diff_);
      }
      // Matched armor not found, but yaw has jumped — target is spinning
      handleArmorJump(same_id_armor);
    }
    else
    {
      // [debug_tracker] 无匹配装甲板
      if (debug_tracker) {
        PKA_DEBUG("armor_solver",
                  "[Tracker::match] NO MATCH: pos_diff={:.4f} yaw_diff={:.4f}",
                  min_position_diff, yaw_diff);
      }
    }
  }
  else
  {
    // [debug_tracker] 本帧无装甲板输入
    if (debug_tracker) {
      PKA_DEBUG("armor_solver", "[Tracker::update] armors_msg is empty, using EKF prediction");
    }
  }

  // 防止关键状态量发散：半径/高度差/角速度（前哨站还包含中心线速度）
  clampTargetState(is_outpost_now);

  // Tracking state machine
  // 注意：前哨站 DETECTING 状态已在函数开头早期返回，不会到达此处。
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

        if (debug_tracker) {
          PKA_DEBUG("armor_solver",
                    "[Tracker::state] DETECTING -> TRACKING id={} (detect_count thres={})",
                    tracked_id, tracking_thres);
        }
      }
      else
      {
        if (debug_tracker) {
          PKA_DEBUG("armor_solver",
                    "[Tracker::state] DETECTING id={} detect_count={}/{}",
                    tracked_id, detect_count_, tracking_thres);
        }
      }
    }
    else
    {
      detect_count_ = 0;
      tracker_state = LOST;
      PKA_DEBUG("armor_solver", "Tracker state: LOST {}", tracked_id);

      if (debug_tracker) {
        PKA_DEBUG("armor_solver",
                  "[Tracker::state] DETECTING -> LOST id={} (no match)", tracked_id);
      }
    }
  }
  else if (tracker_state == TRACKING)
  {
    if (!matched)
    {
      tracker_state = TEMP_LOST;
      lost_count_++;
      PKA_DEBUG("armor_solver", "Tracker state: TEMP_LOST {}", tracked_id);

      if (debug_tracker) {
        PKA_DEBUG("armor_solver",
                  "[Tracker::state] TRACKING -> TEMP_LOST id={} lost_count={}",
                  tracked_id, lost_count_);
      }
    }
    else
    {
      // [debug_tracker] TRACKING 正常匹配
      if (debug_tracker) {
        PKA_DEBUG("armor_solver",
                  "[Tracker::state] TRACKING id={} matched, pos=({:.3f},{:.3f},{:.3f})",
                  tracked_id,
                  tracked_armor.pose.position.x,
                  tracked_armor.pose.position.y,
                  tracked_armor.pose.position.z);
      }
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

        if (debug_tracker) {
          PKA_DEBUG("armor_solver",
                    "[Tracker::state] TEMP_LOST -> LOST id={} (lost_count > thres={})",
                    tracked_id, lost_thres);
        }
      }
      else
      {
        // [debug_tracker] TEMP_LOST 持续计数
        if (debug_tracker) {
          PKA_DEBUG("armor_solver",
                    "[Tracker::state] TEMP_LOST id={} lost_count={}/{}",
                    tracked_id, lost_count_, lost_thres);
        }
      }
    }
    else
    {
      tracker_state = TRACKING;
      lost_count_   = 0;
      PKA_DEBUG("armor_solver", "Tracker state: TRACKING {}", tracked_id);

      if (debug_tracker) {
        PKA_DEBUG("armor_solver",
                  "[Tracker::state] TEMP_LOST -> TRACKING id={} (recovered)", tracked_id);
      }
    }
  }
}

void Tracker::initEKF(const Armor &a) noexcept
{
  last_yaw_ = 0;
  const RobotType rt = robotTypeFromId(a.number);

  if (rt == RobotType::OUTPOST) {
    // 前哨站：重置三板积累缓冲区，加入第一块板，用单板粗估初始化 EKF。
    // 真正的 EKF 初始化（三板均值）在 outpostHandleDetecting 收到第四次出现时完成。
    outpost_plate_buffer_.reset();
    d_zc = 0.0;
    d_za = 0.0;
    const double xa  = a.pose.position.x;
    const double ya  = a.pose.position.y;
    const double za  = a.pose.position.z;
    const double yaw = orientationToYaw(a.pose.orientation);
    last_yaw_ = yaw;
    outpost_plate_buffer_.tryAdd(
      xa, ya, za, yaw, outpost_cfg_.detect_same_plate_yaw_thresh_rad);

    // 单板粗估中心（用于 DETECTING 阶段的 debug 显示）
    const double r = outpost_cfg_.radius_m;
    target_state.resize(X_N_OUTPOST);
    target_state << (xa + r * std::cos(yaw)), (ya + r * std::sin(yaw)), za,
                    yaw, 0.0, r;
    if (outpost_ekf) {
      outpost_ekf->setState(target_state);
    }
    if (debug_ekf) {
      PKA_DEBUG("armor_solver",
                "[EKF::initEKF outpost] 1st plate pos=({:.3f},{:.3f},{:.3f}) "
                "yaw={:.4f} r={:.3f} → accumulating plates (1/{})",
                xa, ya, za, yaw, r,
                OutpostPlateBuffer::NUM_PLATES);
    }
    return;
  }

  // Non-outpost: standard center estimation
  double xa  = a.pose.position.x;
  double ya  = a.pose.position.y;
  double za  = a.pose.position.z;
  double yaw = orientationToYaw(a.pose.orientation);

  // 地面兵种：10-D 状态（原始，不含 d_za，与前哨站 11-D EKF 完全独立）
  target_state = Eigen::VectorXd::Zero(X_N);  // X_N = 10

  double r  = 0.26;
  double xc = xa + r * cos(yaw);
  double yc = ya + r * sin(yaw);
  double zc = za;

  d_za = 0, d_zc = 0, another_r = r;

  // 10-D state: [xc, vx, yc, vy, zc, vz, yaw, v_yaw, r, d_zc]
  target_state << xc, 0, yc, 0, zc, 0, yaw, 0, r, d_zc;
  ekf->setState(target_state);

  if (debug_ekf) {
    PKA_DEBUG("armor_solver",
              "[EKF::initEKF] armor=({:.3f},{:.3f},{:.3f}) yaw={:.4f} r={:.3f} "
              "-> center=({:.3f},{:.3f},{:.3f})",
              xa, ya, za, yaw, r, xc, yc, zc);
  }
}

// 处理装甲板跳跃的情况
void Tracker::handleArmorJump(const Armor &current_armor) noexcept
{
  double last_yaw = target_state(6);
  double yaw      = orientationToYaw(current_armor.pose.orientation);

  // [debug_tracker] 跳跃前后 yaw 对比
  if (debug_tracker) {
    PKA_DEBUG("armor_solver",
              "[Tracker::ArmorJump] last_yaw={:.4f} new_yaw={:.4f} diff={:.4f}",
              last_yaw, yaw, std::abs(yaw - last_yaw));
  }

  if (abs(yaw - last_yaw) > 0.4)
  {
    target_state(6) = yaw;
    if (tracked_armors_num == ArmorsNum::NORMAL_4) {
      d_za = target_state(4) + target_state(9) - current_armor.pose.position.z;
      std::swap(target_state(8), another_r);
      d_zc = d_zc == 0 ? -d_za : 0;
      target_state(9) = d_zc;

      // [debug_tracker] 半径与高度切换详情
      if (debug_tracker) {
        PKA_DEBUG("armor_solver",
                  "[Tracker::ArmorJump] swap r: r={:.3f} another_r={:.3f} "
                  "d_za={:.3f} d_zc={:.3f}",
                  target_state(8), another_r, d_za, d_zc);
      }
    }
    PKA_DEBUG("armor_solver", "Armor Jump!");
  }

  auto p = current_armor.pose.position;
  Eigen::Vector3d current_p(p.x, p.y, p.z);
  Eigen::Vector3d infer_p = getArmorPositionFromState(target_state);

  double infer_diff = (current_p - infer_p).norm();

  // [debug_tracker] 跳跃后推算位置与实测位置误差
  if (debug_tracker) {
    PKA_DEBUG("armor_solver",
              "[Tracker::ArmorJump] infer_pos=({:.3f},{:.3f},{:.3f}) "
              "current_pos=({:.3f},{:.3f},{:.3f}) diff={:.4f}",
              infer_p.x(), infer_p.y(), infer_p.z(),
              p.x, p.y, p.z, infer_diff);
  }

  if (infer_diff > max_match_distance_)
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

    if (debug_tracker) {
      PKA_DEBUG("armor_solver",
                "[Tracker::ArmorJump] state reset: new center=({:.3f},{:.3f},{:.3f}) r={:.3f}",
                target_state(0), target_state(2), target_state(4), r);
    }
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

void Tracker::clampTargetState(bool is_outpost) noexcept
{
  if (is_outpost) {
    if (!outpost_state_clamp_enable || target_state.size() < X_N_OUTPOST || !outpost_ekf) {
      return;
    }
    // 新 6-D 状态：[xc, yc, zc, yaw, v_yaw, r]
    // 只钳位 v_yaw(4) 和 r(5)；xc/yc/zc/yaw 无需钳位
    outpostClampTargetState(*outpost_ekf, target_state,
                            outpost_v_yaw_min, outpost_v_yaw_max,
                            outpost_radius_min, outpost_radius_max,
                            debug_ekf);
    return;
  }

  if (!ground_state_clamp_enable || target_state.size() < 10 || !ekf) {
    return;
  }
  Eigen::VectorXd before = target_state;
  target_state(7) = std::clamp(target_state(7), ground_v_yaw_min, ground_v_yaw_max);
  target_state(8) = std::clamp(target_state(8), ground_radius_min, ground_radius_max);
  target_state(9) = std::clamp(target_state(9), ground_d_zc_min, ground_d_zc_max);
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