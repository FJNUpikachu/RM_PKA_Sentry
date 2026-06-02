#ifndef ARMOR_SOLVER_OUTPOST_SOLVER_HPP_
#define ARMOR_SOLVER_OUTPOST_SOLVER_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

#include "rm_interfaces/msg/armor.hpp"
#include "rm_interfaces/msg/armors.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/msg/target.hpp"
#include "rm_interfaces/msg/measurement.hpp"
// Outpost-specific debug messages
#include "rm_interfaces/msg/outpost_target.hpp"
#include "rm_interfaces/msg/outpost_measurement.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "rm_utils/math/trajectory_compensator.hpp"
#include "rm_utils/math/manual_compensator.hpp"

namespace pka::auto_aim {

// ---------------------------------------------------------------------------
// Outpost operating mode
// ---------------------------------------------------------------------------
/// 前哨站工作模式（对应 outpost.solver.mode 参数）
enum class OutpostMode : int {
  NORMAL = 0,  // 正常 EKF 追踪：yaw→装甲板，pitch→原始观测，全条件开火
  CENTER = 1,  // 中心模式：yaw→预测中心，pitch→原始观测，仅 yaw 判断开火
  SINGLE = 2,  // 单板：装甲 [x,y,z,yaw] 进 4D EKF，cmd 用滤波估计；掉识别时 predict 保持连续
};

extern bool debug_outpost;

struct OutpostParams {
  std::string id{"outpost"};
  double plate_pitch_rad{-0.2617993877991494};
};

struct OutpostSolverParams {
  // Ballistic compensator
  std::string compensator_type{"ideal"};
  double bullet_speed    = 20.0;
  double gravity         = 9.8;
  double resistance      = 0.001;
  int    iteration_times = 20;

  double prediction_delay = 0.0;
  double controller_delay = 0.0;

  double max_tracking_v_yaw  = 6.0;
  double min_switching_v_yaw = 1.0;

  double coming_angle        = 55.0;  
  double leaving_angle       = 20.0;  

  double fire_margin         = 0.8;
  double min_fire_tolerance  = 1.5;  
  double max_fire_tolerance  = 4.0;  

  double yaw_offset   = 0.0;  
  double pitch_offset = 0.0;  

  /// 工作模式
  OutpostMode mode      = OutpostMode::NORMAL;
  double      center_yaw = 0.2;  // CENTER mode: fire yaw threshold (rad)

  /// SINGLE 模式：|yaw_diff|（度）不大于此值时 fire_advice=true，否则 false
  double single_fire_max_yaw = 2.0;
  double single_fire_max_pitch = 2.0;

  /// SINGLE 模式 4D EKF：状态/观测均为 [x,y,z,yaw]（对角 Q、R 由 YAML 调节）
  double single_ekf_sigma2_q_x    = 0.5;
  double single_ekf_sigma2_q_y    = 0.5;
  double single_ekf_sigma2_q_z    = 0.5;
  double single_ekf_sigma2_q_yaw  = 0.05;
  double single_ekf_r_x           = 0.02;
  double single_ekf_r_y           = 0.02;
  double single_ekf_r_z           = 0.02;
  double single_ekf_r_yaw         = 0.02;
  double single_ekf_p0_x          = 1.0;
  double single_ekf_p0_y          = 1.0;
  double single_ekf_p0_z          = 1.0;
  double single_ekf_p0_yaw        = 0.5;

  // SINGLE 模式下，无测量时进入超时（秒）后视为丢失，要求发布零 cmd
  double single_lost_time_thres = 1.5;

  std::vector<std::string> angle_offset{};

  // Formation EKF (8-state) params: [xc,yc,r,omega,theta1,z1,z2,z3]
  // Individual P0 entries
  double formation_ekf_p0_xc     = 2.0;
  double formation_ekf_p0_yc     = 2.0;
  double formation_ekf_p0_r      = 1.0;
  double formation_ekf_p0_omega  = 0.2;
  double formation_ekf_p0_theta1 = 0.5;
  double formation_ekf_p0_z1     = 5.0;
  double formation_ekf_p0_z2     = 5.0;
  double formation_ekf_p0_z3     = 5.0;

  // Individual Q diagonal entries
  double formation_ekf_q_xc     = 1e-4;
  double formation_ekf_q_yc     = 1e-4;
  double formation_ekf_q_r      = 1e-6;
  double formation_ekf_q_omega  = 1e-6;
  double formation_ekf_q_theta1 = 1e-4;
  double formation_ekf_q_z1     = 1e-10;
  double formation_ekf_q_z2     = 1e-10;
  double formation_ekf_q_z3     = 1e-10;

  // Measurement noise (R diag) for plate observation [x, y, z]
  double formation_ekf_r_x = 0.0064;
  double formation_ekf_r_y = 0.0064;
  double formation_ekf_r_z = 0.0225;
  double formation_ekf_r_yaw = 0.02;

  // Matching thresholds between frames (x,y,z in meters, yaw in degrees)
  double matching_thresh_x = 0.2;
  double matching_thresh_y = 0.2;
  double matching_thresh_z = 0.3;
  double matching_thresh_yaw_deg = 5.0;

  // Tracker thresholds (outpost-specific)
  int    tracking_thres = 1;       // frames required to promote DETECTING -> TRACKING
  double lost_time_thres = 1.7;    // seconds of consecutive miss to consider LOST

  // Formation state clamp / deadzone / reset
  bool   formation_clamp_enable = true;
  double formation_r_min = 0.18;
  double formation_r_max = 0.40;
  double formation_omega_deadzone = 0.01; // rad/s, below this omega -> 0
  double formation_omega_max = 4.0;      // rad/s, symmetric clamp
  int    formation_reset_after_temp_lost = 3; // frames to reset after consecutive mismatches
};

// Register debug publishers (optional): allows outpost solver to publish predicted
// formation `Target` and incoming `Measurement` for debugging/visualization.
void setOutpostDebugPublishers(
  rclcpp::Publisher<rm_interfaces::msg::OutpostTarget>::SharedPtr target_pub,
  rclcpp::Publisher<rm_interfaces::msg::OutpostMeasurement>::SharedPtr measurement_pub) noexcept;

struct OutpostSolverState {
  int  lock_id        = -1;   
  bool last_on_target = false; 
  // Per-outpost lightweight tracker state
  enum class TrackerState : int { LOST = 0, DETECTING = 1, TRACKING = 2, TEMP_LOST = 3 };
  TrackerState tracker_state = TrackerState::LOST;
  int detect_count = 0;           // consecutive matched frames while DETECTING
  double lost_time_accum = 0.0;   // accumulated seconds of consecutive misses while TEMP_LOST
};


void declareOutpostParameters(rclcpp::Node & node);
OutpostParams loadOutpostParams(rclcpp::Node & node);

void declareOutpostSolverParameters(rclcpp::Node & node);
OutpostSolverParams loadOutpostSolverParams(rclcpp::Node & node);

// ---------------------------------------------------------------------------
// 前哨站 SINGLE 模式
// ---------------------------------------------------------------------------
class OutpostSinglePlateRuntime {
public:
  OutpostSinglePlateRuntime();
  ~OutpostSinglePlateRuntime();
  OutpostSinglePlateRuntime(OutpostSinglePlateRuntime &&) noexcept;
  OutpostSinglePlateRuntime & operator=(OutpostSinglePlateRuntime &&) noexcept;
  OutpostSinglePlateRuntime(const OutpostSinglePlateRuntime &)            = delete;
  OutpostSinglePlateRuntime & operator=(const OutpostSinglePlateRuntime &) = delete;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  friend bool outpostSinglePlateSolve(
    const std::vector<rm_interfaces::msg::Armor> & armors,
    double dt_sec,
    const std::string & target_frame,
    const std::shared_ptr<tf2_ros::Buffer> & tf2_buffer,
    const OutpostSolverParams & params,
    const OutpostParams & id_params,
    TrajectoryCompensator * compensator,
    ManualCompensator * manual_compensator,
    bool debug_solver,
    OutpostSinglePlateRuntime * runtime,
    rm_interfaces::msg::GimbalCmd * cmd_out,
    rm_interfaces::msg::Target * target_out) noexcept;
  friend void resetOutpostSinglePlateRuntime(OutpostSinglePlateRuntime & r) noexcept;
};

void resetOutpostSinglePlateRuntime(OutpostSinglePlateRuntime & r) noexcept;

bool isOutpostId(const std::string & id, const std::string & outpost_id);

// ---------------------------------------------------------------------------
// Matching helper — yaw-only matching for outpost (TRACKING / TEMP_LOST)
//
// 前哨站匹配条件：直接比较观测 yaw 与 EKF 预测 yaw（均为连续值）的绝对差值。
// EKF 使用 CONSTANT_ROTATION 模型连续追踪当前可见装甲板的 yaw，不做 slot 折叠。
// 不使用 position_diff，只靠 yaw 对齐判断匹配。
// ---------------------------------------------------------------------------

/// 当 |obs_yaw − predicted_yaw| < max_yaw_diff 时返回 true。
bool outpostIsYawMatched(double obs_yaw,
                         double predicted_yaw,
                         double max_yaw_diff) noexcept;

// ---------------------------------------------------------------------------
// Node helper — raw armor position for pitch（无预测延时）
//
// 对于前哨站，cmd_gimbal 的 pitch 由原始观测装甲板坐标计算（不使用 EKF 预测）。
// 本函数将 tracker_->tracked_armor.pose.position 写入 target_msg.tracked_armor_x/y/z，
// 供 armor_solver 在 solve() 中直接用于 pitch 解算，不叠加 prediction_delay 和
// controller_delay。
// ---------------------------------------------------------------------------
void outpostFillRawArmorPosition(
  const rm_interfaces::msg::Armor & tracked_armor,
  rm_interfaces::msg::Target * target) noexcept;

// ---------------------------------------------------------------------------
// Slot geometry（供 armor_solver::getArmorPositions 在 armors_num == 3 时调用）
//
// 返回前哨站第 slot_index 块板的世界坐标：
//   pos = center + (-r·cos(yaw + slot·2π/3), -r·sin(yaw + slot·2π/3), d_zc + slot·d_za)
// 新模型中 d_zc = d_za = 0（所有板高度 = zc），参数保留以保持接口兼容性。
// ---------------------------------------------------------------------------
Eigen::Vector3d outpostThreePlateSlotWorldPosition(
  const Eigen::Vector3d & center,
  double target_yaw,
  std::size_t slot_index,
  double r,
  double d_zc = 0.0,
  double d_za = 0.0) noexcept;

// ---------------------------------------------------------------------------
// RViz visualization（从 armor_solver_node::publishMarkers 调用）
//
// 追加三块装甲板 CUBE Marker（仅在 target.armors_num == 3 且 id 为前哨站时生效）。
// ---------------------------------------------------------------------------
void appendOutpostFilteredArmorsStripMarkers(
  const rm_interfaces::msg::Target & target,
  const OutpostParams & p,
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  visualization_msgs::msg::MarkerArray * markers,
  visualization_msgs::msg::Marker * last_marker_out);

// ---------------------------------------------------------------------------
// Center mode solve（内部辅助函数，由 outpostSolve 调用）
//
// 前哨站 center mode 解算：
//   yaw   → predicted_center（飞行时间补偿后的中心坐标）
//   pitch → target.tracked_armor_x/y/z（原始观测装甲板，带弹道重力补偿）
//   fire  → 仅检查 yaw 误差（< params.center_yaw），不检查 distance
//
// 返回 true 并填充 cmd_out 当且仅当 isOutpostId(target.id) && params.mode == CENTER。
// ---------------------------------------------------------------------------
bool outpostCenterModeSolve(
  const rm_interfaces::msg::Target & target,
  const rm_interfaces::msg::Measurement & measurement,
  const Eigen::Vector3d & predicted_center,
  const std::array<double, 3> & rpy,
  const OutpostSolverParams & params,
  const OutpostParams & id_params,
  TrajectoryCompensator * compensator,
  ManualCompensator * manual_compensator,
  bool debug_solver,
  rm_interfaces::msg::GimbalCmd * cmd_out) noexcept;

// ---------------------------------------------------------------------------
// Single-plate mode solve（单板 + 4D EKF）
//
// 状态/观测均为装甲板位姿 [x,y,z,yaw]（世界系），使用 rm_utils::ExtendedKalmanFilter；
// 有识别时对 EKF update；无识别时仅 predict，保持 cmd_gimbal 与 Target 连续。
// cmd_gimbal 始终由 EKF 后验（无测量时为预测）位置计算，不直接使用原始 armors。
// target_out 在已初始化滤波器时填充 tracking=true、armors_num=1，供话题连续性。
//
// 仅当 params.mode == OutpostMode::SINGLE 时处理；未初始化且无测量时返回 false。
// ---------------------------------------------------------------------------
bool outpostSinglePlateSolve(
  const std::vector<rm_interfaces::msg::Armor> & armors,
  double dt_sec,
  const std::string & target_frame,
  const std::shared_ptr<tf2_ros::Buffer> & tf2_buffer,
  const OutpostSolverParams & params,
  const OutpostParams & id_params,
  TrajectoryCompensator * compensator,
  ManualCompensator * manual_compensator,
  bool debug_solver,
  OutpostSinglePlateRuntime * runtime,
  rm_interfaces::msg::GimbalCmd * cmd_out,
  rm_interfaces::msg::Target * target_out) noexcept;

// ---------------------------------------------------------------------------
// Unified outpost solve（全部实现在 outpost_solver.cpp，唯一对外接口）
//
// 统一处理前哨站两种模式：
//
//   mode == CENTER（瞄中心模式）：
//     yaw   → 预测中心（经 outpost 弹道补偿 + prediction_delay）
//     pitch → 原始观测装甲板（无 delay，经弹道重力补偿）
//     fire  → 仅 yaw 误差检查（< params.center_yaw）
//
//   mode == NORMAL（跟踪装甲板模式）：
//     yaw   → 选定装甲板（经 outpost prediction_delay + controller_delay）
//     pitch → 原始观测装甲板（无 delay，经弹道重力补偿）
//     fire  → 与同包 Solver::isOnTarget 一致：小装甲板半宽投影 + fire_margin + 容差上下界 + 两帧稳定
//
// 注意：SINGLE 模式由 outpostSinglePlateSolve 在 armorsCallback 中独立处理（4D EKF），
// 本函数不涉及 SINGLE 模式（调用方需提前 early-return）。
// 全程使用 outpost.solver.* 独立参数，与地面兵种完全解耦。
// 返回 true 并填充 cmd_out 当且仅当 isOutpostId(target.id, id_params.id)。
// ---------------------------------------------------------------------------
bool outpostSolve(
  const rm_interfaces::msg::Target & target,
  const rm_interfaces::msg::Measurement & measurement,
  double elapsed_sec,
  const std::array<double, 3> & rpy,
  const OutpostSolverParams & params,
  const OutpostParams & id_params,
  TrajectoryCompensator * compensator,
  ManualCompensator * manual_compensator,
  bool debug_solver,
  OutpostSolverState & state,
  rm_interfaces::msg::GimbalCmd * cmd_out) noexcept;

}  // namespace pka::auto_aim

#endif  // ARMOR_SOLVER_OUTPOST_SOLVER_HPP_
