// Outpost solver — RMUC 2026 outpost model
//
// Model: three armor plates evenly spaced 120° apart on a circle of radius R,
// rotating around a fixed vertical axis at constant angular velocity.
// All plates are modeled at the same height zc (average of three observed
// plate heights); individual height differences are not tracked.
//
// EKF state (6-D):  x = [xc, yc, zc, yaw, v_yaw, r]
//   xc, yc:   rotation-axis horizontal position (fixed)
//   zc:       rotation-axis height  = mean(za_0, za_1, za_2) — fixed after init
//   yaw:      current slot-0 yaw angle
//   v_yaw:    angular velocity (EKF-tracked)
//   r:        rotation radius (EKF-tracked, initialized from yaml)
//
// Tracking strategy:
//   Phase 1 – DETECTING (plate accumulation)
//     Collect up to three distinct armor-plate observations (identified by
//     yaw difference > 0.7 rad ≈ 40°).  When the buffer holds three plates
//     and a fourth observation appears (the first plate cycling back), the
//     EKF is initialized from the buffer and tracking begins.
//
//   Phase 2 – TRACKING / TEMP_LOST
//     Standard EKF predict + three-slot match + slot-0 remap + EKF update.
//
// All outpost logic is implemented in outpost_solver.cpp.
// Other modules (armor_tracker, armor_solver_node, armor_solver) only call
// the interfaces declared here.
//
// Licensed under the Apache License, Version 2.0

#ifndef ARMOR_SOLVER_OUTPOST_SOLVER_HPP_
#define ARMOR_SOLVER_OUTPOST_SOLVER_HPP_

#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include "rm_interfaces/msg/armor.hpp"
#include "rm_interfaces/msg/armors.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/msg/target.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "armor_solver/motion_model.hpp"

namespace pka::auto_aim {

// ===========================================================================
// 前哨站专用 EKF（6-D，纯旋转模型，独立于地面兵种 10-D EKF）
// ===========================================================================

/// 前哨站 EKF 状态维度（6-D）
/// x = [xc, yc, zc, yaw, v_yaw, r]
constexpr int X_N_OUTPOST = 6;

/// 前哨站预测函数（CONSTANT_ROTATION：中心固定，yaw 匀速旋转）
struct OutpostPredict {
  explicit OutpostPredict(double dt = 0.005) : dt(dt) {}

  template <typename T>
  void operator()(const T x0[X_N_OUTPOST], T x1[X_N_OUTPOST]) const
  {
    x1[0] = x0[0];                     // xc — constant
    x1[1] = x0[1];                     // yc — constant
    x1[2] = x0[2];                     // zc — constant
    x1[3] = x0[3] + x0[4] * T(dt);    // yaw += v_yaw * dt
    x1[4] = x0[4];                     // v_yaw — constant
    x1[5] = x0[5];                     // r — constant
  }

  double dt;
};

/// 前哨站观测函数
/// 输入测量已由 outpostPrepareEKFMeasurement() 重映射到 slot-0 等效：
///   z[0] = xc - r·cos(yaw)   (xa_slot0)
///   z[1] = yc - r·sin(yaw)   (ya_slot0)
///   z[2] = zc                 (旋转轴高度，三板均值)
///   z[3] = yaw                (yaw_slot0)
struct OutpostMeasure {
  template <typename T>
  void operator()(const T x[X_N_OUTPOST], T z[Z_N]) const
  {
    z[0] = x[0] - ceres::cos(x[3]) * x[5];   // xa = xc - r·cos(yaw)
    z[1] = x[1] - ceres::sin(x[3]) * x[5];   // ya = yc - r·sin(yaw)
    z[2] = x[2];                               // za = zc
    z[3] = x[3];                               // yaw_a = yaw
  }
};

using OutpostStateEKF =
  pka::ExtendedKalmanFilter<X_N_OUTPOST, Z_N, OutpostPredict, OutpostMeasure>;

// ---------------------------------------------------------------------------
// Global debug switch
// ---------------------------------------------------------------------------
extern bool debug_outpost;

// ---------------------------------------------------------------------------
// Physical parameters
// ---------------------------------------------------------------------------
struct OutpostParams {
  /// Sticker id, must match Armor.number from the detector
  std::string id{"outpost"};

  /// Rotation radius (m) — initial value for EKF state(5)
  double radius_m{0.275};

  /// Plate pitch angle (rad) — used only for RViz visualization
  double plate_pitch_rad{-0.2617993877991494};

  /// Maximum |yaw_diff| (deg) for single-plate fire advice
  double fire_yaw_max_deg{8.0};

  /// DETECTING: same-plate yaw threshold (rad)
  double detect_same_plate_yaw_thresh_rad{0.7};

  /// DETECTING: require empty-armors gap before accepting next plate
  bool detect_require_gap_for_new_plate{true};

  /// DETECTING: minimum consecutive empty-armors frames as "gap"
  int detect_min_gap_frames{1};

  /// DETECTING: minimum yaw jump (rad) to accept a new plate
  /// (ArmorJump-like guard, but for 120 deg outpost plate switching)
  double detect_new_plate_min_yaw_jump_rad{1.2};

  /// DETECTING: when armors count changes 1->0->1, force the first new
  /// observation to be treated as next plate (ignore known gate).
  bool detect_force_next_on_gap{true};

  /// DETECTING: optional safety guard for forced next-plate add.
  /// Set to 0 to disable; recommended 0.1~0.3 rad to avoid pure jitter.
  double detect_force_next_min_yaw_jump_rad{0.15};

  /// DETECTING(force-gap): when bypassing known gate, require candidate plate z
  /// to differ from all cached plates by at least this threshold (m).
  double detect_force_next_min_z_diff_m{0.03};

  /// DETECTING->TRACKING: after 3 plates are cached, require the next
  /// observed plate to match the first cached plate ("4th == 1st").
  bool detect_require_return_to_first_plate{true};

  /// Yaw threshold (rad) used by the return-to-first-plate check.
  double detect_return_to_first_yaw_thresh_rad{0.7};

  /// If true, reject outpost model init when |d_za| is too small and restart.
  bool detect_rebuild_on_invalid_height{true};

  /// Minimum valid |d_za| (m) after three-plate init.
  double detect_min_valid_dza_m{0.02};
};

// ---------------------------------------------------------------------------
// Aiming mode
// ---------------------------------------------------------------------------
enum class OutpostMode { SINGLE_PLATE = 0, FUSION = 1 };

// ---------------------------------------------------------------------------
// Plate accumulation buffer（三板收集策略）
//
// 工作原理：
//   前哨站三块板依次出现（中间有 armors==0 的短暂间隔）。
//   当缓冲区尚未满时，每次新板的 yaw 与已存板相差 > PLATE_YAW_THRESHOLD
//   则视为新板并加入缓冲区。
//   缓冲区满（三块板均收集到）后，下一次出现的板即为"第四次出现"
//   （第一块板循环回来），此时初始化 EKF 并进入 TRACKING。
// ---------------------------------------------------------------------------
struct OutpostPlateBuffer {
  struct PlateObs {
    double xa, ya, za, yaw;
  };

  static constexpr int    NUM_PLATES           = 3;
  /// 两块不同板 yaw 差 ≥ 120°；以 0.7 rad（≈40°）为门限区分不同板
  static constexpr double PLATE_YAW_THRESHOLD  = 0.7;

  std::vector<PlateObs> plates;   ///< 已积累的板，至多 3 块
  int lost_count{0};              ///< DETECTING 阶段连续无装甲帧数
  int gap_count{0};               ///< DETECTING 阶段连续空帧计数
  bool had_armor_last_frame{false};
  bool has_last_yaw{false};
  double last_yaw{0.0};

  void reset();
  bool isFull() const { return static_cast<int>(plates.size()) >= NUM_PLATES; }

  /// 尝试将该观测加为新板。
  /// 若与已存板 yaw 差 < PLATE_YAW_THRESHOLD，视为同一块，返回 false；
  /// 否则加入缓冲区并返回 true（缓冲区已满时直接返回 false）。
  bool tryAdd(double xa, double ya, double za, double yaw, double same_yaw_thresh_rad);

  /// 强制加入新板（用于 1->0->1 规则），忽略 same-plate 判定。
  bool forceAdd(double xa, double ya, double za, double yaw);

  /// 检查候选板 z 与当前缓存板是否足够分离。
  bool hasEnoughZSeparation(double za, double min_z_diff_m) const;

  /// 检查 yaw 是否与已知某块板接近（用于判断"第四次出现"）。
  bool matchesKnown(double yaw, double same_yaw_thresh_rad) const;
};

/// DETECTING 阶段积累逻辑的处理结果
enum class OutpostDetectingResult {
  KEEP_DETECTING,  ///< 继续积累，保持 DETECTING
  START_TRACKING,  ///< 三板集齐且第四次出现 → 初始化 EKF → TRACKING
  GO_LOST,         ///< 连续无装甲超时 → 重置 → LOST
};

// ---------------------------------------------------------------------------
// Parameter management
// ---------------------------------------------------------------------------
void declareOutpostParameters(rclcpp::Node & node);
OutpostParams loadOutpostParams(rclcpp::Node & node);
OutpostMode   loadOutpostMode  (rclcpp::Node & node);

// ---------------------------------------------------------------------------
// Filtering helpers
// ---------------------------------------------------------------------------

/// 从 Armors 消息中筛选前哨站装甲板
std::vector<rm_interfaces::msg::Armor> filterOutpostArmors(
  const rm_interfaces::msg::Armors::SharedPtr & msg,
  const std::string & outpost_id);

/// 判断 id 是否为前哨站
bool isOutpostId(const std::string & id, const std::string & outpost_id);

// ---------------------------------------------------------------------------
// EKF initialization from 3-plate buffer
//
// 从三板缓冲区初始化 6-D EKF 状态：
//   xc  = mean(xa_i + r·cos(yaw_i))   — 三板反推中心 x 均值
//   yc  = mean(ya_i + r·sin(yaw_i))   — 三板反推中心 y 均值
//   zc  = mean(za_i)                   — 三板 z 高度均值
//   yaw = current_obs_yaw              — 当前（第四次）观测的 yaw 作为 slot-0 参考
//   v_yaw = 0.0                        — 初值，EKF 快速收敛
//   r   = p.radius_m                   — 初值，EKF 在线估计
// ---------------------------------------------------------------------------
void outpostInitStateFromBuffer(
  const OutpostParams & p,
  const OutpostPlateBuffer & buf,
  double current_obs_yaw,
  Eigen::VectorXd * state_out);

/// 从三板缓存计算高度参数（按 slot0/1/2 排序）：
///   z_slot0 = zc + d_zc
///   z_slot1 = zc + d_zc + d_za
///   z_slot2 = zc + d_zc + 2*d_za
/// 该函数用于把“初始三板各自高度”传递到 target / RViz / solver。
void outpostComputeHeightOffsetsFromBuffer(
  const OutpostPlateBuffer & buf,
  double * d_zc_out,
  double * d_za_out);

// ---------------------------------------------------------------------------
// Detecting phase handler（DETECTING 状态每帧调用一次）
//
// 管理三板积累逻辑：
//   - has_armor = false：lost_count++；超时返回 GO_LOST（重置缓冲区）
//   - has_armor = true, buf 未满：tryAdd，更新粗略 EKF 状态，KEEP_DETECTING
//   - has_armor = true, buf 已满：outpostInitStateFromBuffer → ekf.setState，START_TRACKING
//
// state_out: KEEP_DETECTING 时为单板粗估；START_TRACKING 时为完整初始状态。
// ---------------------------------------------------------------------------
OutpostDetectingResult outpostHandleDetecting(
  OutpostPlateBuffer & buf,
  int lost_thres,
  OutpostStateEKF & ekf,
  const OutpostParams & p,
  bool has_armor,
  const rm_interfaces::msg::Armor & armor,
  Eigen::VectorXd * state_out,
  bool debug) noexcept;

// ---------------------------------------------------------------------------
// Slot geometry（TRACKING/TEMP_LOST 阶段使用）
// ---------------------------------------------------------------------------

/// 返回前哨站第 slot_k 块板的世界坐标（所有板高度 = zc）：
///   pos = [xc - r·cos(yaw + k·2π/3),  yc - r·sin(yaw + k·2π/3),  zc]
/// 内部使用（接受 6-D EKF 预测状态）
Eigen::Vector3d outpostSlotWorldPosition(
  const Eigen::VectorXd & state_6d,
  std::size_t slot_k) noexcept;

/// 前哨站板槽世界坐标（公共接口，供 armor_solver 使用）
/// 新模型中 d_zc = d_za = 0（所有板高度 = center.z），保持接口兼容性。
Eigen::Vector3d outpostThreePlateSlotWorldPosition(
  const Eigen::Vector3d & center,
  double target_yaw,
  std::size_t slot_index,
  double r,
  double d_zc = 0.0,
  double d_za = 0.0) noexcept;

// ---------------------------------------------------------------------------
// Tracker helpers（从 armor_tracker 在 TRACKING/TEMP_LOST 状态调用）
// ---------------------------------------------------------------------------

/// 对 EKF 预测状态和当前观测，在三个 slot 中找最近 slot，
/// 输出 pos_diff 和 yaw_diff，用于 tracker 匹配门限判断。
void outpostFindBestSlotMatch(
  const Eigen::VectorXd & ekf_pred,
  const rm_interfaces::msg::Armor & armor,
  double * pos_diff_out,
  double * yaw_diff_out) noexcept;

/// 将观测重映射到 slot-0 等效后，构造 4-D EKF 测量向量。
/// 返回: [xa_slot0, ya_slot0, za_raw, yaw_slot0]
Eigen::Vector4d outpostPrepareEKFMeasurement(
  const Eigen::VectorXd & ekf_pred,
  const rm_interfaces::msg::Armor & armor,
  double measured_yaw) noexcept;

// ---------------------------------------------------------------------------
// State clamp（从 Tracker::clampTargetState 调用）
// ---------------------------------------------------------------------------
void outpostClampTargetState(
  OutpostStateEKF & ekf,
  Eigen::VectorXd & state,
  double v_yaw_min, double v_yaw_max,
  double radius_min, double radius_max,
  bool debug_ekf) noexcept;

// ---------------------------------------------------------------------------
// Node helpers（从 armor_solver_node 调用）
// ---------------------------------------------------------------------------

/// 用 6-D EKF 状态填充 target_msg 全部字段（仅在 target->id == outpost_id 时生效）。
/// SINGLE_PLATE 模式：armors_num=1，直接瞄准当前板估计位置。
/// FUSION 模式：armors_num=3，EKF 追踪三板旋转，solver 选最佳板。
void outpostApplyTargetFields(
  const OutpostParams & p,
  OutpostMode mode,
  const Eigen::VectorXd & state,
  double d_zc,
  double d_za,
  const rm_interfaces::msg::Armor & tracked_armor,
  rm_interfaces::msg::Target * target);

/// DETECTING 阶段 target 填充。
/// 新模型中三板积累完成前不发布 tracking 目标（设 tracking=false 后返回）。
void outpostFillDetectingTarget(
  const OutpostParams & p,
  const Eigen::VectorXd & ekf_state,
  rm_interfaces::msg::Target * target);

/// 单板开火约束：|yaw_diff| > fire_yaw_max_deg 时关闭 fire_advice。
void outpostApplySinglePlateFireConstraint(
  const OutpostParams & p,
  rm_interfaces::msg::GimbalCmd * gimbal_cmd) noexcept;

// ---------------------------------------------------------------------------
// RViz visualization（从 armor_solver_node::publishMarkers 调用）
// ---------------------------------------------------------------------------

/// 追加三块装甲板 CUBE Marker（target.armors_num==3 且 id 为前哨站时生效）。
/// 所有板高度均设为 target.position.z（= zc，三板均值）。
void appendOutpostFilteredArmorsStripMarkers(
  const rm_interfaces::msg::Target & target,
  const OutpostParams & p,
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  visualization_msgs::msg::MarkerArray * markers,
  visualization_msgs::msg::Marker * last_marker_out);

}  // namespace pka::auto_aim

#endif  // ARMOR_SOLVER_OUTPOST_SOLVER_HPP_
