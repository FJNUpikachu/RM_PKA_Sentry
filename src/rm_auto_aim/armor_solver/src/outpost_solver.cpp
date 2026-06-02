#include "armor_solver/outpost_solver.hpp"

#include <cmath>
#include <algorithm>

#include <angles/angles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "rm_utils/pkaLoggerCenter.hpp"
#include "rm_utils/math/extended_kalman_filter.hpp"
// ceres for Jet trig functions
#include <ceres/ceres.h>

namespace pka::auto_aim {

bool debug_outpost = false;

// Optional debug publishers (set by node). Allow solver code to publish
// predicted formation `Target` and incoming `Measurement` for inspection.
static rclcpp::Publisher<rm_interfaces::msg::OutpostTarget>::SharedPtr outpost_debug_target_pub = nullptr;
static rclcpp::Publisher<rm_interfaces::msg::OutpostMeasurement>::SharedPtr outpost_debug_measure_pub = nullptr;
// consecutive mismatch counter for TEMP_LOST/reset logic
static int formation_temp_lost_count = 0;

void setOutpostDebugPublishers(
  rclcpp::Publisher<rm_interfaces::msg::OutpostTarget>::SharedPtr target_pub,
  rclcpp::Publisher<rm_interfaces::msg::OutpostMeasurement>::SharedPtr measurement_pub) noexcept
{
  outpost_debug_target_pub = std::move(target_pub);
  outpost_debug_measure_pub = std::move(measurement_pub);
}

//EKF封装
namespace {

constexpr int kSingle4 = 4;

struct Single4Predict {
  template<typename T>
  void operator()(const T x0[kSingle4], T x1[kSingle4]) const
  {
    for (int i = 0; i < kSingle4; ++i) {
      x1[i] = x0[i];
    }
  }
};

struct Single4Measure {
  template<typename T>
  void operator()(const T x[kSingle4], T z[kSingle4]) const
  {
    for (int i = 0; i < kSingle4; ++i) {
      z[i] = x[i];
    }
  }
};

using Single4EKF = pka::ExtendedKalmanFilter<
  kSingle4, kSingle4, Single4Predict, Single4Measure>;

// 8-state formation EKF: [xc, yc, r, omega, theta1, z1, z2, z3]
constexpr int kOutpost8 = 8;
struct Outpost8Predict {
  double dt;
  explicit Outpost8Predict(double dt_ = 0.0) noexcept : dt(dt_) {}
  template<typename T>
  void operator()(const T x0[kOutpost8], T x1[kOutpost8]) const
  {
    for (int i = 0; i < kOutpost8; ++i) {
      x1[i] = x0[i];
    }
    // theta1 += omega * dt
    x1[4] = x0[4] + x0[3] * T(dt);
  }
};

struct Outpost8Measure {
  int slot_idx;
  explicit Outpost8Measure(int idx = 0) noexcept : slot_idx(idx) {}
  template<typename T>
  void operator()(const T x[kOutpost8], T z[4]) const
  {
    // x: [xc, yc, r, omega, theta1, z1, z2, z3]
    const T xc = x[0];
    const T yc = x[1];
    const T r  = x[2];
    const T theta1 = x[4];
    const T th = theta1 + T(static_cast<double>(slot_idx) * 2.0 * M_PI / 3.0);
    // follow same geometry as outpostThreePlateSlotWorldPosition
    z[0] = xc - ceres::cos(th) * r;
    z[1] = yc - ceres::sin(th) * r;
    // z-measure is the per-slot height stored in state
    z[2] = x[5 + slot_idx];
    // predicted plate yaw (theta1 + slot offset)
    z[3] = th;
  }
};

using Outpost8EKF = pka::ExtendedKalmanFilter<kOutpost8, 4, Outpost8Predict, Outpost8Measure>;

}  // namespace

// Formation EKF runtime (file-local)
namespace {
struct FormationRuntime {
  bool initialized{false};
  Eigen::Matrix<double, 8, 1> q_diag;
  Eigen::Matrix<double, 4, 1> r_diag;
  Eigen::Matrix<double, 8, 1> p0_diag;
  std::unique_ptr<Outpost8EKF> ekf;

  void syncNoise(const OutpostSolverParams &p) noexcept {
    p0_diag(0) = p.formation_ekf_p0_xc;
    p0_diag(1) = p.formation_ekf_p0_yc;
    p0_diag(2) = p.formation_ekf_p0_r;
    p0_diag(3) = p.formation_ekf_p0_omega;
    p0_diag(4) = p.formation_ekf_p0_theta1;
    p0_diag(5) = p.formation_ekf_p0_z1;
    p0_diag(6) = p.formation_ekf_p0_z2;
    p0_diag(7) = p.formation_ekf_p0_z3;

    q_diag(0) = p.formation_ekf_q_xc;
    q_diag(1) = p.formation_ekf_q_yc;
    q_diag(2) = p.formation_ekf_q_r;
    q_diag(3) = p.formation_ekf_q_omega;
    q_diag(4) = p.formation_ekf_q_theta1;
    q_diag(5) = p.formation_ekf_q_z1;
    q_diag(6) = p.formation_ekf_q_z2;
    q_diag(7) = p.formation_ekf_q_z3;

    r_diag(0) = p.formation_ekf_r_x;
    r_diag(1) = p.formation_ekf_r_y;
    r_diag(2) = p.formation_ekf_r_z;
    r_diag(3) = p.formation_ekf_r_yaw;
  }

  Eigen::Matrix<double, 8, 8> makeP0() const noexcept {
    Eigen::Matrix<double, 8, 8> P0 = Eigen::Matrix<double, 8, 8>::Zero();
    for (int i = 0; i < 8; ++i) P0(i, i) = p0_diag(i);
    return P0;
  }

  void ensureFilter(const OutpostSolverParams &p) {
    syncNoise(p);
    if (ekf) return;
    Eigen::Matrix<double, 8, 8> P0 = makeP0();
    auto u_q = [this]() {
      Eigen::Matrix<double, 8, 8> Q = Eigen::Matrix<double, 8, 8>::Zero();
      Q.diagonal() = q_diag;
      return Q;
    };
    auto u_r = [this](const Eigen::Matrix<double, 4, 1> &z) {
      Eigen::Matrix<double, 4, 4> R = Eigen::Matrix<double, 4, 4>::Zero();
      // for x,y,z keep previous scaling by abs(z) for x/y/z, yaw uses fixed diag
      R(0,0) = r_diag(0) * std::abs(z(0));
      R(1,1) = r_diag(1) * std::abs(z(1));
      R(2,2) = r_diag(2) * std::abs(z(2));
      R(3,3) = r_diag(3);
      return R;
    };
    Outpost8Predict fp;
    Outpost8Measure hm(0);
    ekf = std::make_unique<Outpost8EKF>(fp, hm, u_q, u_r, P0);
  }
};

static FormationRuntime formation_rt;
// previous-frame simple matcher
static bool formation_prev_has_meas = false;
static double formation_prev_x = 0.0, formation_prev_y = 0.0, formation_prev_z = 0.0;
static int formation_prev_idx = -1;
}

//EKF初始化
struct OutpostSinglePlateRuntime::Impl {
  bool                         initialized{false};
  Eigen::Matrix<double, 4, 1>  q_diag;
  Eigen::Matrix<double, 4, 1>  r_diag;
  std::unique_ptr<Single4EKF>  ekf;
  OutpostSolverState           tracker{};

  void syncNoise(const OutpostSolverParams & p) noexcept
  {
    q_diag << p.single_ekf_sigma2_q_x, p.single_ekf_sigma2_q_y, p.single_ekf_sigma2_q_z,
      p.single_ekf_sigma2_q_yaw;
    r_diag << p.single_ekf_r_x, p.single_ekf_r_y, p.single_ekf_r_z, p.single_ekf_r_yaw;
  }

  Eigen::Matrix4d makeP0(const OutpostSolverParams & p) const noexcept
  {
    Eigen::Matrix4d P0 = Eigen::Matrix4d::Zero();
    P0(0, 0) = p.single_ekf_p0_x;
    P0(1, 1) = p.single_ekf_p0_y;
    P0(2, 2) = p.single_ekf_p0_z;
    P0(3, 3) = p.single_ekf_p0_yaw;
    return P0;
  }

  void ensureFilter(const OutpostSolverParams & p)
  {
    syncNoise(p);
    if (ekf) {
      return;
    }
    Eigen::Matrix4d   P0   = makeP0(p);
    auto              u_q = [this]() {
      Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
      Q.diagonal()      = q_diag;
      return Q;
    };
    auto u_r = [this](const Eigen::Matrix<double, 4, 1> &) {
      Eigen::Matrix4d R = Eigen::Matrix4d::Zero();
      R.diagonal()      = r_diag;
      return R;
    };
    Single4Predict fp;
    Single4Measure hm;
    ekf = std::make_unique<Single4EKF>(fp, hm, u_q, u_r, P0);
  }
};

namespace {

// 从tf中提取yaw
double quatYaw(const geometry_msgs::msg::Quaternion & q)
{
  tf2::Quaternion tf_q;
  tf2::fromMsg(q, tf_q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  return yaw;
}

// ---------------------------------------------------------------------------
// outpostSolve（NORMAL）
// ---------------------------------------------------------------------------

void outpostNormalPredictCenter(
  const rm_interfaces::msg::Target & target,
  const double elapsed_sec,
  const OutpostSolverParams & params,
  TrajectoryCompensator * compensator,
  Eigen::Vector3d * center_out,
  double * yaw_out) noexcept
{
  Eigen::Vector3d pos(target.position.x, target.position.y, target.position.z);
  double y = target.yaw;
  const double flying_time = compensator ? compensator->getFlyingTime(pos) : 0.0;
  const double dt          = elapsed_sec + flying_time + params.prediction_delay;
  pos.x() += dt * target.velocity.x;
  pos.y() += dt * target.velocity.y;
  pos.z() += dt * target.velocity.z;
  y += dt * target.v_yaw;
  *center_out = pos;
  *yaw_out     = y;
}

void outpostNormalFillArmorPositions(
  const Eigen::Vector3d & center,
  const double yaw,
  const rm_interfaces::msg::Target & target,
  std::array<Eigen::Vector3d, 3> * out) noexcept
{
  for (std::size_t i = 0; i < 3; ++i) {
    (*out)[i] = outpostThreePlateSlotWorldPosition(
      center, yaw, i, target.radius_1, target.d_zc, target.d_za);
  }
}

void outpostNormalComputeDeltaAngles(
  const Eigen::Vector3d & center,
  const double target_yaw,
  std::array<double, 3> * deltas_out) noexcept
{
  const double alpha = std::atan2(center.y(), center.x());
  for (std::size_t i = 0; i < 3; ++i) {
    double delta = target_yaw + static_cast<double>(i) * (2.0 * M_PI / 3.0) - alpha;
    while (delta > M_PI) delta -= 2.0 * M_PI;
    while (delta < -M_PI) delta += 2.0 * M_PI;
    (*deltas_out)[i] = delta;
  }
}

int outpostNormalSelectArmorSlot(
  const std::array<double, 3> & delta_angles,
  const double v_yaw,
  const OutpostSolverParams & params,
  OutpostSolverState & state) noexcept
{
  int selected_id = 0;
  if (std::abs(v_yaw) < params.min_switching_v_yaw) {
    double min_abs = std::abs(delta_angles[0]);
    for (int i = 1; i < 3; ++i) {
      if (std::abs(delta_angles[static_cast<std::size_t>(i)]) < min_abs) {
        min_abs     = std::abs(delta_angles[static_cast<std::size_t>(i)]);
        selected_id = i;
      }
    }
    if (state.lock_id >= 0 && state.lock_id < 3) {
      const double lock_d = std::abs(delta_angles[static_cast<std::size_t>(state.lock_id)]);
      if (lock_d < M_PI / 3.0 && std::abs(lock_d - min_abs) < M_PI / 6.0) {
        selected_id = state.lock_id;
      }
    }
  } else {
    const double coming_rad  = params.coming_angle * M_PI / 180.0;
    const double leaving_rad = params.leaving_angle * M_PI / 180.0;
    double       best_score  = 1e9;
    for (int i = 0; i < 3; ++i) {
      const double d         = delta_angles[static_cast<std::size_t>(i)];
      const bool   in_zone = (v_yaw < 0) ? (d > -leaving_rad && d < coming_rad)
                                         : (d > -coming_rad && d < leaving_rad);
      if (in_zone && std::abs(d) < best_score) {
        best_score  = std::abs(d);
        selected_id = i;
      }
    }
    if (best_score >= 1e9) {
      double min_abs = 1e9;
      for (int i = 0; i < 3; ++i) {
        if (std::abs(delta_angles[static_cast<std::size_t>(i)]) < min_abs) {
          min_abs     = std::abs(delta_angles[static_cast<std::size_t>(i)]);
          selected_id = i;
        }
      }
    }
  }
  state.lock_id = selected_id;
  return selected_id;
}

Eigen::Vector3d outpostNormalChosenArmorWithDelay(
  const Eigen::Vector3d & predicted_center,
  const double predicted_yaw,
  const std::array<Eigen::Vector3d, 3> & armor_positions,
  const int selected_id,
  const rm_interfaces::msg::Target & target,
  const OutpostSolverParams & params) noexcept
{
  Eigen::Vector3d chosen = armor_positions[static_cast<std::size_t>(selected_id)];
  if (params.controller_delay != 0.0) {
    Eigen::Vector3d delayed_pos = predicted_center;
    double          delayed_yaw = predicted_yaw;
    delayed_pos.x() += params.controller_delay * target.velocity.x;
    delayed_pos.y() += params.controller_delay * target.velocity.y;
    delayed_pos.z() += params.controller_delay * target.velocity.z;
    delayed_yaw += params.controller_delay * target.v_yaw;
    chosen = outpostThreePlateSlotWorldPosition(
      delayed_pos, delayed_yaw, static_cast<std::size_t>(selected_id),
      target.radius_1, target.d_zc, target.d_za);
  }
  return chosen;
}

void outpostNormalBuildPitchYawCmd(
  const rm_interfaces::msg::Target & target,
  const Eigen::Vector3d & chosen_armor,
  const OutpostSolverParams & params,
  TrajectoryCompensator * compensator,
  ManualCompensator * manual_compensator,
  double * cmd_yaw_rad,
  double * cmd_pitch_rad) noexcept
{
  // Use chosen_armor (already delayed by outpostNormalChosenArmorWithDelay) as
  // the reference for both yaw and pitch in NORMAL mode. This ensures pitch
  // compensates for controller delay similarly to CENTER mode.
  const Eigen::Vector3d pitch_ref = chosen_armor;
  double pitch = std::atan2(pitch_ref.z(), pitch_ref.head(2).norm());
  if (compensator) {
    double tmp = pitch;
    if (compensator->compensate(pitch_ref, tmp)) pitch = tmp;
  }
  double pitch_off = 0.0, yaw_off = 0.0;
  if (manual_compensator) {
    auto ao = manual_compensator->angleHardCorrect(
      chosen_armor.head(2).norm(), chosen_armor.z());
    pitch_off = ao[0] * M_PI / 180.0;
    yaw_off   = ao[1] * M_PI / 180.0;
  }
  const double yaw_raw = std::atan2(chosen_armor.y(), chosen_armor.x());
  *cmd_pitch_rad       = pitch + pitch_off + params.pitch_offset * M_PI / 180.0;
  *cmd_yaw_rad         = angles::normalize_angle(
    yaw_raw + yaw_off + params.yaw_offset * M_PI / 180.0);
}

/// 与 armor_solver::Solver::isOnTarget 相同思路
bool outpostNormalFireAdviceIsOnTarget(
  const double gimbal_yaw,
  const double cmd_yaw_rad,
  const double armor_delta_angle,
  const double distance,
  const double target_v_yaw,
  const OutpostSolverParams & params,
  OutpostSolverState & state,
  double * tol_rad_dbg,
  double * yaw_err_dbg) noexcept
{
  constexpr double kSmallArmorHalfW = 0.133 / 2.0;  // SMALL_ARMOR_HALF_W，与 armor_solver.hpp 一致
  const double cos_incidence    = std::abs(std::cos(armor_delta_angle));
  const double projected_half_w = kSmallArmorHalfW * cos_incidence;
  double       tolerance_rad    = std::atan2(projected_half_w, distance) * params.fire_margin;
  tolerance_rad = std::clamp(tolerance_rad,
                             params.min_fire_tolerance * M_PI / 180.0,
                             params.max_fire_tolerance * M_PI / 180.0);
  const double yaw_err    = std::abs(angles::shortest_angular_distance(gimbal_yaw, cmd_yaw_rad));
  const bool   on_target  = yaw_err < tolerance_rad;
  const bool   stable     = on_target && state.last_on_target;
  state.last_on_target    = on_target;
  if (tol_rad_dbg) *tol_rad_dbg = tolerance_rad;
  if (yaw_err_dbg) *yaw_err_dbg = yaw_err;
  return stable || (std::abs(target_v_yaw) > params.max_tracking_v_yaw);
}

}  // namespace

// ---------------------------------------------------------------------------
// OutpostSinglePlateRuntime
// ---------------------------------------------------------------------------

OutpostSinglePlateRuntime::OutpostSinglePlateRuntime()
: impl_(std::make_unique<Impl>())
{
}

OutpostSinglePlateRuntime::~OutpostSinglePlateRuntime() = default;

OutpostSinglePlateRuntime::OutpostSinglePlateRuntime(OutpostSinglePlateRuntime && o) noexcept
: impl_(std::move(o.impl_))
{
}

OutpostSinglePlateRuntime &
OutpostSinglePlateRuntime::operator=(OutpostSinglePlateRuntime && o) noexcept
{
  impl_ = std::move(o.impl_);
  return *this;
}

void resetOutpostSinglePlateRuntime(OutpostSinglePlateRuntime & r) noexcept
{
  r.impl_ = std::make_unique<OutpostSinglePlateRuntime::Impl>();
}

// ---------------------------------------------------------------------------
// Parameter management — OutpostParams (id / display)
// ---------------------------------------------------------------------------

void declareOutpostParameters(rclcpp::Node & node)
{
  node.declare_parameter("outpost.id",               std::string("outpost"));
  node.declare_parameter("outpost.plate_pitch_rad",  -0.2617993877991494);
}

// ---------------------------------------------------------------------------
// Parameter management — OutpostSolverParams (all solver params for outpost)
// ---------------------------------------------------------------------------

void declareOutpostSolverParameters(rclcpp::Node & node)
{
  // Ballistic compensator
  node.declare_parameter("outpost.solver.compensator_type",  std::string("ideal"));
  node.declare_parameter("outpost.solver.bullet_speed",      20.0);
  node.declare_parameter("outpost.solver.gravity",           9.8);
  node.declare_parameter("outpost.solver.resistance",        0.001);
  node.declare_parameter("outpost.solver.iteration_times",   20);
  // Delays
  node.declare_parameter("outpost.solver.prediction_delay",  0.0);
  node.declare_parameter("outpost.solver.controller_delay",  0.0);
  // State machine thresholds
  node.declare_parameter("outpost.solver.max_tracking_v_yaw",   6.0);
  node.declare_parameter("outpost.solver.min_switching_v_yaw",  1.0);
  node.declare_parameter("outpost.solver.coming_angle",         55.0);
  node.declare_parameter("outpost.solver.leaving_angle",        20.0);
  node.declare_parameter("outpost.solver.fire_margin",           0.8);
  node.declare_parameter("outpost.solver.min_fire_tolerance",    1.5);
  node.declare_parameter("outpost.solver.max_fire_tolerance",    4.0);
  // Angular offsets
  node.declare_parameter("outpost.solver.yaw_offset",    0.0);
  node.declare_parameter("outpost.solver.pitch_offset",  0.0);
  // Operating mode: 0=normal, 1=center, 2=single_plate
  node.declare_parameter("outpost.solver.mode",       0);
  node.declare_parameter("outpost.solver.center_yaw", 0.2);
  node.declare_parameter("outpost.solver.single_fire_max_yaw", 2.0);
  node.declare_parameter("outpost.solver.single_fire_max_pitch", 2.0);
  node.declare_parameter("outpost.solver.single_ekf.sigma2_q_x",    0.5);
  node.declare_parameter("outpost.solver.single_ekf.sigma2_q_y",    0.5);
  node.declare_parameter("outpost.solver.single_ekf.sigma2_q_z",    0.5);
  node.declare_parameter("outpost.solver.single_ekf.sigma2_q_yaw", 0.05);
  node.declare_parameter("outpost.solver.single_ekf.r_x",          0.02);
  node.declare_parameter("outpost.solver.single_ekf.r_y",          0.02);
  node.declare_parameter("outpost.solver.single_ekf.r_z",          0.02);
  node.declare_parameter("outpost.solver.single_ekf.r_yaw",        0.02);
  node.declare_parameter("outpost.solver.single_ekf.p0_x",       1.0);
  node.declare_parameter("outpost.solver.single_ekf.p0_y",       1.0);
  node.declare_parameter("outpost.solver.single_ekf.p0_z",       1.0);
  node.declare_parameter("outpost.solver.single_ekf.p0_yaw",     0.5);
  // Manual compensator table
  node.declare_parameter("outpost.solver.angle_offset", std::vector<std::string>{});

  // Formation EKF params (individual scalars)
  node.declare_parameter("outpost.solver.formation_ekf.p0_xc", 2.0);
  node.declare_parameter("outpost.solver.formation_ekf.p0_yc", 2.0);
  node.declare_parameter("outpost.solver.formation_ekf.p0_r", 1.0);
  node.declare_parameter("outpost.solver.formation_ekf.p0_omega", 0.2);
  node.declare_parameter("outpost.solver.formation_ekf.p0_theta1", 0.5);
  node.declare_parameter("outpost.solver.formation_ekf.p0_z1", 5.0);
  node.declare_parameter("outpost.solver.formation_ekf.p0_z2", 5.0);
  node.declare_parameter("outpost.solver.formation_ekf.p0_z3", 5.0);

  node.declare_parameter("outpost.solver.formation_ekf.q_xc", 1e-4);
  node.declare_parameter("outpost.solver.formation_ekf.q_yc", 1e-4);
  node.declare_parameter("outpost.solver.formation_ekf.q_r", 1e-6);
  node.declare_parameter("outpost.solver.formation_ekf.q_omega", 1e-6);
  node.declare_parameter("outpost.solver.formation_ekf.q_theta1", 1e-4);
  node.declare_parameter("outpost.solver.formation_ekf.q_z1", 1e-10);
  node.declare_parameter("outpost.solver.formation_ekf.q_z2", 1e-10);
  node.declare_parameter("outpost.solver.formation_ekf.q_z3", 1e-10);

  node.declare_parameter("outpost.solver.formation_ekf.r_x", 0.0064);
  node.declare_parameter("outpost.solver.formation_ekf.r_y", 0.0064);
  node.declare_parameter("outpost.solver.formation_ekf.r_z", 0.0225);
  node.declare_parameter("outpost.solver.formation_ekf.r_yaw", 0.02);

  // Matching thresholds: x,y,z (meters), yaw (degrees)
  node.declare_parameter("outpost.solver.matching_thresh_x", 0.2);
  node.declare_parameter("outpost.solver.matching_thresh_y", 0.2);
  node.declare_parameter("outpost.solver.matching_thresh_z", 0.3);
  node.declare_parameter("outpost.solver.matching_thresh_yaw_deg", 5.0);

  // Formation clamp / deadzone / reset
  node.declare_parameter("outpost.solver.formation_clamp.enable", true);
  node.declare_parameter("outpost.solver.formation_clamp.r_min", 0.18);
  node.declare_parameter("outpost.solver.formation_clamp.r_max", 0.40);
  node.declare_parameter("outpost.solver.formation_clamp.omega_deadzone", 0.01);
  node.declare_parameter("outpost.solver.formation_clamp.omega_max", 4.0);
  node.declare_parameter("outpost.solver.formation_reset_after_temp_lost", 3);
  // SINGLE mode: time (seconds) without measurement to consider lost
  node.declare_parameter("outpost.solver.single_lost_time_thres", 1.5);
}

OutpostSolverParams loadOutpostSolverParams(rclcpp::Node & node)
{
  OutpostSolverParams p;
  p.compensator_type   = node.get_parameter("outpost.solver.compensator_type").as_string();
  p.bullet_speed       = node.get_parameter("outpost.solver.bullet_speed").as_double();
  p.gravity            = node.get_parameter("outpost.solver.gravity").as_double();
  p.resistance         = node.get_parameter("outpost.solver.resistance").as_double();
  p.iteration_times    = static_cast<int>(node.get_parameter("outpost.solver.iteration_times").as_int());
  p.prediction_delay   = node.get_parameter("outpost.solver.prediction_delay").as_double();
  p.controller_delay   = node.get_parameter("outpost.solver.controller_delay").as_double();
  p.max_tracking_v_yaw  = node.get_parameter("outpost.solver.max_tracking_v_yaw").as_double();
  p.min_switching_v_yaw = node.get_parameter("outpost.solver.min_switching_v_yaw").as_double();
  p.coming_angle        = node.get_parameter("outpost.solver.coming_angle").as_double();
  p.leaving_angle       = node.get_parameter("outpost.solver.leaving_angle").as_double();
  p.fire_margin        = node.get_parameter("outpost.solver.fire_margin").as_double();
  p.min_fire_tolerance = node.get_parameter("outpost.solver.min_fire_tolerance").as_double();
  p.max_fire_tolerance = node.get_parameter("outpost.solver.max_fire_tolerance").as_double();
  p.yaw_offset         = node.get_parameter("outpost.solver.yaw_offset").as_double();
  p.pitch_offset       = node.get_parameter("outpost.solver.pitch_offset").as_double();
  p.mode       = static_cast<OutpostMode>(node.get_parameter("outpost.solver.mode").as_int());
  p.center_yaw = node.get_parameter("outpost.solver.center_yaw").as_double();
  p.single_fire_max_yaw =
    node.get_parameter("outpost.solver.single_fire_max_yaw").as_double();
  p.single_fire_max_pitch =
    node.get_parameter("outpost.solver.single_fire_max_pitch").as_double();
  p.single_ekf_sigma2_q_x =
    node.get_parameter("outpost.solver.single_ekf.sigma2_q_x").as_double();
  p.single_ekf_sigma2_q_y =
    node.get_parameter("outpost.solver.single_ekf.sigma2_q_y").as_double();
  p.single_ekf_sigma2_q_z =
    node.get_parameter("outpost.solver.single_ekf.sigma2_q_z").as_double();
  p.single_ekf_sigma2_q_yaw =
    node.get_parameter("outpost.solver.single_ekf.sigma2_q_yaw").as_double();
  p.single_ekf_r_x   = node.get_parameter("outpost.solver.single_ekf.r_x").as_double();
  p.single_ekf_r_y   = node.get_parameter("outpost.solver.single_ekf.r_y").as_double();
  p.single_ekf_r_z   = node.get_parameter("outpost.solver.single_ekf.r_z").as_double();
  p.single_ekf_r_yaw = node.get_parameter("outpost.solver.single_ekf.r_yaw").as_double();
  p.single_ekf_p0_x =
    node.get_parameter("outpost.solver.single_ekf.p0_x").as_double();
  p.single_ekf_p0_y =
    node.get_parameter("outpost.solver.single_ekf.p0_y").as_double();
  p.single_ekf_p0_z =
    node.get_parameter("outpost.solver.single_ekf.p0_z").as_double();
  p.single_ekf_p0_yaw =
    node.get_parameter("outpost.solver.single_ekf.p0_yaw").as_double();
  p.angle_offset       = node.get_parameter("outpost.solver.angle_offset").as_string_array();
  // Read formation EKF individual params
  p.formation_ekf_p0_xc = node.get_parameter("outpost.solver.formation_ekf.p0_xc").as_double();
  p.formation_ekf_p0_yc = node.get_parameter("outpost.solver.formation_ekf.p0_yc").as_double();
  p.formation_ekf_p0_r = node.get_parameter("outpost.solver.formation_ekf.p0_r").as_double();
  p.formation_ekf_p0_omega = node.get_parameter("outpost.solver.formation_ekf.p0_omega").as_double();
  p.formation_ekf_p0_theta1 = node.get_parameter("outpost.solver.formation_ekf.p0_theta1").as_double();
  p.formation_ekf_p0_z1 = node.get_parameter("outpost.solver.formation_ekf.p0_z1").as_double();
  p.formation_ekf_p0_z2 = node.get_parameter("outpost.solver.formation_ekf.p0_z2").as_double();
  p.formation_ekf_p0_z3 = node.get_parameter("outpost.solver.formation_ekf.p0_z3").as_double();

  p.formation_ekf_q_xc = node.get_parameter("outpost.solver.formation_ekf.q_xc").as_double();
  p.formation_ekf_q_yc = node.get_parameter("outpost.solver.formation_ekf.q_yc").as_double();
  p.formation_ekf_q_r = node.get_parameter("outpost.solver.formation_ekf.q_r").as_double();
  p.formation_ekf_q_omega = node.get_parameter("outpost.solver.formation_ekf.q_omega").as_double();
  p.formation_ekf_q_theta1 = node.get_parameter("outpost.solver.formation_ekf.q_theta1").as_double();
  p.formation_ekf_q_z1 = node.get_parameter("outpost.solver.formation_ekf.q_z1").as_double();
  p.formation_ekf_q_z2 = node.get_parameter("outpost.solver.formation_ekf.q_z2").as_double();
  p.formation_ekf_q_z3 = node.get_parameter("outpost.solver.formation_ekf.q_z3").as_double();

  p.formation_ekf_r_x = node.get_parameter("outpost.solver.formation_ekf.r_x").as_double();
  p.formation_ekf_r_y = node.get_parameter("outpost.solver.formation_ekf.r_y").as_double();
  p.formation_ekf_r_z = node.get_parameter("outpost.solver.formation_ekf.r_z").as_double();
  p.formation_ekf_r_yaw = node.get_parameter("outpost.solver.formation_ekf.r_yaw").as_double();

  p.matching_thresh_x = node.get_parameter("outpost.solver.matching_thresh_x").as_double();
  p.matching_thresh_y = node.get_parameter("outpost.solver.matching_thresh_y").as_double();
  p.matching_thresh_z = node.get_parameter("outpost.solver.matching_thresh_z").as_double();
  p.matching_thresh_yaw_deg = node.get_parameter("outpost.solver.matching_thresh_yaw_deg").as_double();

  // Outpost tracker thresholds
  // tracking_thres: consecutive matched frames to enter TRACKING from DETECTING
  // lost_time_thres: seconds of consecutive misses to declare LOST from TEMP_LOST
  p.tracking_thres = node.get_parameter("outpost.tracker.tracking_thres").as_int();
  p.lost_time_thres = node.get_parameter("outpost.tracker.lost_time_thres").as_double();

  // Formation clamp / deadzone / reset
  p.formation_clamp_enable = node.get_parameter("outpost.solver.formation_clamp.enable").as_bool();
  p.formation_r_min = node.get_parameter("outpost.solver.formation_clamp.r_min").as_double();
  p.formation_r_max = node.get_parameter("outpost.solver.formation_clamp.r_max").as_double();
  p.formation_omega_deadzone = node.get_parameter("outpost.solver.formation_clamp.omega_deadzone").as_double();
  p.formation_omega_max = node.get_parameter("outpost.solver.formation_clamp.omega_max").as_double();
  p.formation_reset_after_temp_lost = node.get_parameter("outpost.solver.formation_reset_after_temp_lost").as_int();
  p.single_lost_time_thres = node.get_parameter("outpost.solver.single_lost_time_thres").as_double();
  return p;
}

OutpostParams loadOutpostParams(rclcpp::Node & node)
{
  OutpostParams p;
  p.id               = node.get_parameter("outpost.id").as_string();
  p.plate_pitch_rad  = node.get_parameter("outpost.plate_pitch_rad").as_double();

  if (debug_outpost) {
    PKA_DEBUG("armor_solver",
              "[Outpost::params] id={} plate_pitch={:.4f}rad",
              p.id, p.plate_pitch_rad);
  }
  return p;
}

// ---------------------------------------------------------------------------
// ID check
// ---------------------------------------------------------------------------

bool isOutpostId(const std::string & id, const std::string & outpost_id)
{
  return id == outpost_id;
}

// ---------------------------------------------------------------------------
// Matching helper（yaw-only，直接比较，无 slot 折叠）
// ---------------------------------------------------------------------------

bool outpostIsYawMatched(const double obs_yaw,
                         const double predicted_yaw,
                         const double max_yaw_diff) noexcept
{
  return std::abs(obs_yaw - predicted_yaw) < max_yaw_diff;
}

// ---------------------------------------------------------------------------
// Node helper — raw armor position（pitch 无延时）
// ---------------------------------------------------------------------------

void outpostFillRawArmorPosition(
  const rm_interfaces::msg::Armor & tracked_armor,
  rm_interfaces::msg::Target * target) noexcept
{
  if (!target) return;
  target->tracked_armor_x = tracked_armor.pose.position.x;
  target->tracked_armor_y = tracked_armor.pose.position.y;
  target->tracked_armor_z = tracked_armor.pose.position.z;
}

// ---------------------------------------------------------------------------
// Slot geometry
// ---------------------------------------------------------------------------

Eigen::Vector3d outpostThreePlateSlotWorldPosition(
  const Eigen::Vector3d & center,
  const double target_yaw,
  const std::size_t slot_index,
  const double r,
  const double d_zc,
  const double d_za) noexcept
{
  const double ang = target_yaw + static_cast<double>(slot_index) * (2.0 * M_PI / 3.0);
  const double dz  = d_zc + static_cast<double>(slot_index) * d_za;
  return center + Eigen::Vector3d(-r * std::cos(ang), -r * std::sin(ang), dz);
}

// ---------------------------------------------------------------------------
// Center mode solve
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
  const bool debug_solver,
  rm_interfaces::msg::GimbalCmd * cmd_out) noexcept
{
  if (!cmd_out) return false;
  if (!isOutpostId(target.id, id_params.id)) return false;
  if (params.mode != OutpostMode::CENTER) return false;

  // yaw → predicted center
  const double yaw_raw = std::atan2(predicted_center.y(), predicted_center.x());

  // pitch → aim at the observed plate but apply controller delay (and prediction is in predicted_center)
  const Eigen::Vector3d raw_armor(
    target.tracked_armor_x, target.tracked_armor_y, target.tracked_armor_z);

  // choose slot by nearest predicted plate position (using predicted_center / predicted yaw)
  const double pred_yaw = yaw_raw;
  std::array<Eigen::Vector3d, 3> pred_slots{};
  for (std::size_t i = 0; i < 3; ++i) {
    pred_slots[i] = outpostThreePlateSlotWorldPosition(predicted_center, pred_yaw, i, target.radius_1, target.d_zc, target.d_za);
  }
  int meas_slot = 0;
  if (raw_armor.norm() > 0.05) {
    double best_d = (pred_slots[0] - raw_armor).squaredNorm();
    for (int i = 1; i < 3; ++i) {
      double d = (pred_slots[static_cast<std::size_t>(i)] - raw_armor).squaredNorm();
      if (d < best_d) { best_d = d; meas_slot = i; }
    }
  }

  // apply controller delay to predicted center and yaw, then compute chosen plate position
  Eigen::Vector3d delayed_center = predicted_center;
  double delayed_yaw = pred_yaw;
  if (params.controller_delay != 0.0) {
    delayed_center.x() += params.controller_delay * target.velocity.x;
    delayed_center.y() += params.controller_delay * target.velocity.y;
    delayed_center.z() += params.controller_delay * target.velocity.z;
    delayed_yaw += params.controller_delay * target.v_yaw;
  }
  Eigen::Vector3d chosen_plate = outpostThreePlateSlotWorldPosition(delayed_center, delayed_yaw, static_cast<std::size_t>(meas_slot), target.radius_1, target.d_zc, target.d_za);

  double pitch = std::atan2(chosen_plate.z(), chosen_plate.head(2).norm());

  // Ballistic gravity compensation (based on chosen plate position)
  if (compensator) {
    double tmp = pitch;
    if (compensator->compensate(chosen_plate, tmp)) {
      pitch = tmp;
    }
  }

  // Distance-segmented manual compensator (yaw_offset + pitch_offset table)
  double pitch_offset_val = 0.0;
  double yaw_offset_val   = 0.0;
  if (manual_compensator) {
    auto ao = manual_compensator->angleHardCorrect(
      predicted_center.head(2).norm(), predicted_center.z());
    pitch_offset_val = ao[0] * M_PI / 180.0;
    yaw_offset_val   = ao[1] * M_PI / 180.0;
  }

  const double cmd_pitch = pitch + pitch_offset_val + params.pitch_offset * M_PI / 180.0;
  const double cmd_yaw   = angles::normalize_angle(
    yaw_raw + yaw_offset_val + params.yaw_offset * M_PI / 180.0);

  cmd_out->header     = target.header;
  cmd_out->distance   = predicted_center.norm();
  cmd_out->yaw        = cmd_yaw   * 180.0 / M_PI;
  cmd_out->pitch      = cmd_pitch * 180.0 / M_PI;
  cmd_out->yaw_diff   = (cmd_yaw   - rpy[2]) * 180.0 / M_PI;
  cmd_out->pitch_diff = (cmd_pitch - rpy[1]) * 180.0 / M_PI;

  // Fire: only yaw check, distance NOT checked
  const double yaw_err     = std::abs(angles::shortest_angular_distance(rpy[2], cmd_yaw));
  cmd_out->fire_advice = (yaw_err < params.center_yaw);

  if (debug_solver) {
    PKA_DEBUG("armor_solver",
              "[Outpost::centerMode] center=({:.3f},{:.3f},{:.3f}) "
              "raw_armor=({:.3f},{:.3f},{:.3f}) "
              "yaw={:.3f}deg pitch={:.3f}deg yaw_err={:.4f}rad({:.2f}deg) fire={}",
              predicted_center.x(), predicted_center.y(), predicted_center.z(),
              raw_armor.x(), raw_armor.y(), raw_armor.z(),
              cmd_yaw * 180.0 / M_PI, cmd_pitch * 180.0 / M_PI,
              yaw_err, yaw_err * 180.0 / M_PI,
              cmd_out->fire_advice);
  }

  return true;
}

// ---------------------------------------------------------------------------
// Unified outpost solve（center mode + non-center mode）
// ---------------------------------------------------------------------------

bool outpostSolve(
  const rm_interfaces::msg::Target & target,
  const rm_interfaces::msg::Measurement & measurement,
  const double elapsed_sec,
  const std::array<double, 3> & rpy,
  const OutpostSolverParams & params,
  const OutpostParams & id_params,
  TrajectoryCompensator * compensator,
  ManualCompensator * manual_compensator,
  const bool debug_solver,
  OutpostSolverState & state,
  rm_interfaces::msg::GimbalCmd * cmd_out) noexcept
{
  if (!cmd_out) return false;
  if (!isOutpostId(target.id, id_params.id)) return false;

  Eigen::Vector3d target_position;
  double          target_yaw = 0.0;
  outpostNormalPredictCenter(
    target, elapsed_sec, params, compensator, &target_position, &target_yaw);

  // --- Formation EKF predict / update (8-state) ---
  // Use a persistent formation_rt (file-static) to track [xc,yc,r,omega,theta1,z1,z2,z3]
  formation_rt.syncNoise(params);
  formation_rt.ensureFilter(params);

  // choose dt for EKF predict (use elapsed + prediction delay to be consistent)
  const double ekf_dt = elapsed_sec + params.prediction_delay;
  if (formation_rt.ekf) {
    formation_rt.ekf->setPredictFunc(Outpost8Predict(ekf_dt));
  }

  Eigen::Matrix<double, 8, 1> x_pri = Eigen::Matrix<double, 8, 1>::Zero();
  if (formation_rt.ekf) x_pri = formation_rt.ekf->predict();
  // Publish predicted formation target for debugging
  if (debug_outpost && outpost_debug_target_pub) {
    rm_interfaces::msg::OutpostTarget pred_msg;
    pred_msg.header = target.header;
    pred_msg.id = target.id;
    pred_msg.tracking = formation_rt.initialized;
    pred_msg.armors_num = target.armors_num;
    pred_msg.position.x = x_pri(0);
    pred_msg.position.y = x_pri(1);
    pred_msg.position.z = target.position.z;
    pred_msg.yaw = x_pri(4);
    pred_msg.v_yaw = x_pri(3);
    pred_msg.radius = x_pri(2);
    // predicted first slot position
    Eigen::Vector3d slot0 = outpostThreePlateSlotWorldPosition(
      Eigen::Vector3d(x_pri(0), x_pri(1), target.position.z), x_pri(4), 0, x_pri(2), target.d_zc, target.d_za);
    pred_msg.tracked_armor_x = slot0.x();
    pred_msg.tracked_armor_y = slot0.y();
    pred_msg.tracked_armor_z = slot0.z();
    pred_msg.z1 = x_pri(5);
    pred_msg.z2 = x_pri(6);
    pred_msg.z3 = x_pri(7);
    pred_msg.tracked_armor_yaw = x_pri(4);
    outpost_debug_target_pub->publish(pred_msg);
  }

  Eigen::Matrix<double, 8, 1> x_est = x_pri;

  // If we have a tracked raw armor observation (raw pitch input), use it to update formation EKF
  const Eigen::Vector3d tracked_raw(target.tracked_armor_x, target.tracked_armor_y, target.tracked_armor_z);
  if (target.armors_num == 3 && tracked_raw.norm() >= 0.05) {
    // Ensure initialized: use target-based init when EKF not initialized
    if (!formation_rt.initialized) {
      Eigen::Matrix<double, 8, 1> x0;
      x0.setZero();
      x0(0) = target.position.x;
      x0(1) = target.position.y;
      x0(2) = target.radius_1 > 0.0 ? target.radius_1 : 0.5;
      x0(3) = target.v_yaw;
      x0(4) = target.yaw;
      x0(5) = target.tracked_armor_z > 0.05 ? target.tracked_armor_z : target.position.z;
      x0(6) = x0(5);
      x0(7) = x0(5);
      formation_rt.ekf->setState(x0);
      formation_rt.initialized = true;
      x_est = x0;
    } else {
      // choose slot by nearest predicted plate position
      std::array<Eigen::Vector3d, 3> pred_slots{};
      Eigen::Vector3d center_pri(x_pri(0), x_pri(1), target.position.z);
      const double pred_yaw = x_pri(4);
      for (std::size_t i = 0; i < 3; ++i) {
        pred_slots[i] = outpostThreePlateSlotWorldPosition(center_pri, pred_yaw, i, x_pri(2), target.d_zc, target.d_za);
      }

      int best_idx = 0;
      double best_d = (pred_slots[0] - tracked_raw).squaredNorm();
      for (int i = 1; i < 3; ++i) {
        double d = (pred_slots[static_cast<std::size_t>(i)] - tracked_raw).squaredNorm();
        if (d < best_d) { best_d = d; best_idx = i; }
      }

      // simple previous-frame comparison to detect jump / next-slot rule
      bool same_as_prev = false;
      if (formation_prev_has_meas) {
        if (std::abs(tracked_raw.x() - formation_prev_x) <= params.matching_thresh_x &&
            std::abs(tracked_raw.y() - formation_prev_y) <= params.matching_thresh_y &&
            std::abs(tracked_raw.z() - formation_prev_z) <= params.matching_thresh_z) {
          same_as_prev = true;
        }
      }
      int meas_slot = best_idx;
      if (!same_as_prev && formation_prev_has_meas && formation_prev_idx >= 0) {
        meas_slot = (formation_prev_idx + 1) % 3;
      }

      // update EKF with the observed plate position (x,y,z,yaw)
      Eigen::Matrix<double, 4, 1> z;
      z << tracked_raw.x(), tracked_raw.y(), tracked_raw.z(), measurement.yaw;
      // Publish raw measurement for debug
      if (debug_outpost && outpost_debug_measure_pub) {
        rm_interfaces::msg::OutpostMeasurement mm;
        mm.header = target.header;
        mm.header.frame_id = target.header.frame_id;
        mm.x = tracked_raw.x();
        mm.y = tracked_raw.y();
        mm.z = tracked_raw.z();
        mm.yaw = measurement.yaw; // per-plate yaw from tracker measurement
        outpost_debug_measure_pub->publish(mm);
      }
      // Check mismatch between predicted slot and measurement => TEMP_LOST handling
      const Eigen::Vector3d &pred_slot = pred_slots[static_cast<std::size_t>(meas_slot)];
      const double err_x = std::abs(tracked_raw.x() - pred_slot.x());
      const double err_y = std::abs(tracked_raw.y() - pred_slot.y());
      const double err_z = std::abs(tracked_raw.z() - pred_slot.z());
      // yaw mismatch: compare per-slot predicted yaw vs measured yaw (degrees)
      const double pred_slot_yaw = pred_yaw + static_cast<double>(meas_slot) * 2.0 * M_PI / 3.0;
      const double err_yaw = std::abs(angles::shortest_angular_distance(pred_slot_yaw, measurement.yaw));
      const double err_yaw_deg = err_yaw * 180.0 / M_PI;
      const bool is_mismatch = (err_x > params.matching_thresh_x) ||
               (err_y > params.matching_thresh_y) ||
               (err_z > params.matching_thresh_z) ||
               (err_yaw_deg > params.matching_thresh_yaw_deg);

      // Outpost tracker state machine (DETECTING -> TRACKING -> TEMP_LOST -> LOST)
      const bool matched = !is_mismatch;
      {
        using TS = OutpostSolverState::TrackerState;
        if (state.tracker_state == TS::DETECTING) {
          if (matched) {
            state.detect_count++;
            if (state.detect_count > params.tracking_thres) {
              state.detect_count = 0;
              state.tracker_state = TS::TRACKING;
              PKA_DEBUG("armor_solver", "[Outpost::Tracker] state: TRACKING id={} slot={}", target.id, meas_slot);
            } else {
              if (debug_solver) {
                PKA_DEBUG("armor_solver", "[Outpost::Tracker] DETECTING id={} detect_count={}/{}",
                          target.id, state.detect_count, params.tracking_thres);
              }
            }
          } else {
            state.detect_count = 0;
            state.tracker_state = TS::LOST;
            PKA_DEBUG("armor_solver", "[Outpost::Tracker] state: LOST id={} slot={}", target.id, meas_slot);
          }
        } else if (state.tracker_state == TS::TRACKING) {
          if (!matched) {
            state.tracker_state = TS::TEMP_LOST;
            state.lost_time_accum = elapsed_sec;
            PKA_DEBUG("armor_solver", "[Outpost::Tracker] state: TEMP_LOST id={} slot={}", target.id, meas_slot);
          } else {
            if (debug_solver) {
              PKA_DEBUG("armor_solver", "[Outpost::Tracker] TRACKING id={} matched slot={}", target.id, meas_slot);
            }
          }
        } else if (state.tracker_state == TS::TEMP_LOST) {
          if (!matched) {
            state.lost_time_accum += elapsed_sec;
            if (state.lost_time_accum > params.lost_time_thres) {
              state.lost_time_accum = 0.0;
              state.tracker_state = TS::LOST;
              PKA_DEBUG("armor_solver", "[Outpost::Tracker] state: LOST id={} slot={} after timeout",
                        target.id, meas_slot);
            } else {
              if (debug_solver) {
                PKA_DEBUG("armor_solver", "[Outpost::Tracker] TEMP_LOST id={} lost_time={:.3f}/{:.3f}s",
                          target.id, state.lost_time_accum, params.lost_time_thres);
              }
            }
          } else {
            state.tracker_state = TS::TRACKING;
            state.lost_time_accum = 0.0;
            PKA_DEBUG("armor_solver", "[Outpost::Tracker] state: TRACKING id={} slot={} (recovered)",
                      target.id, meas_slot);
          }
        } else { // LOST
          if (matched) {
            state.tracker_state = TS::DETECTING;
            state.detect_count = 1;
            PKA_DEBUG("armor_solver", "[Outpost::Tracker] state: DETECTING id={} slot={} (new detect)",
                      target.id, meas_slot);
          }
        }
      }
      if (is_mismatch) {
        // treat as TEMP_LOST: skip update and keep prior
        formation_temp_lost_count++;
        x_est = x_pri;
        if (debug_solver) {
          PKA_DEBUG("armor_solver",
                    "[Outpost::TEMP_LOST] meas_slot={} err=({:.3f},{:.3f},{:.3f}) thresh=({:.3f},{:.3f},{:.3f}) cnt={}",
                    meas_slot, err_x, err_y, err_z,
                    params.matching_thresh_x, params.matching_thresh_y, params.matching_thresh_z,
                    formation_temp_lost_count);
        }
        if (formation_temp_lost_count >= params.formation_reset_after_temp_lost) {
          // Reset EKF covariance/state to avoid divergence
          formation_rt.ekf.reset();
          formation_rt.ensureFilter(params);
          if (formation_rt.ekf) formation_rt.ekf->setState(x_pri);
          formation_rt.initialized = true;
          formation_prev_has_meas = false;
          formation_prev_idx = -1;
          formation_temp_lost_count = 0;
          if (debug_solver) PKA_DEBUG("armor_solver", "[Outpost::RESET] formation EKF reset after consecutive TEMP_LOST");
        }
      } else {
        // measurement accepted: perform EKF update
        formation_temp_lost_count = 0;
        if (formation_rt.ekf) {
          formation_rt.ekf->setMeasureFunc(Outpost8Measure(meas_slot));
          x_est = formation_rt.ekf->update(z);
        }

        // store last measurement only when not mismatch
        formation_prev_has_meas = true;
        formation_prev_x = tracked_raw.x();
        formation_prev_y = tracked_raw.y();
        formation_prev_z = tracked_raw.z();
        formation_prev_idx = meas_slot;

        // Clamp/Deadzone for radius and omega
        if (params.formation_clamp_enable) {
          double omega = x_est(3);
          if (std::abs(omega) < params.formation_omega_deadzone) omega = 0.0;
          omega = std::clamp(omega, -params.formation_omega_max, params.formation_omega_max);
          x_est(3) = omega;
          double r = x_est(2);
          r = std::clamp(r, params.formation_r_min, params.formation_r_max);
          x_est(2) = r;
        }
      }
    }
  }

  // Use EKF center if available
  Eigen::Vector3d ekf_center = target_position;
  if (formation_rt.initialized) {
    ekf_center.x() = x_est(0);
    ekf_center.y() = x_est(1);
    ekf_center.z() = target.position.z; // center z left as reported by tracker
  }

  if (params.mode == OutpostMode::CENTER) {
    return outpostCenterModeSolve(target, measurement, target_position, rpy, params, id_params,
                                     compensator, manual_compensator, debug_solver, cmd_out);
  }

  std::array<Eigen::Vector3d, 3> armor_positions{};
  outpostNormalFillArmorPositions(ekf_center, target_yaw, target, &armor_positions);

  std::array<double, 3> delta_angles{};
  outpostNormalComputeDeltaAngles(ekf_center, target_yaw, &delta_angles);

  const int       selected_id     = outpostNormalSelectArmorSlot(
    delta_angles, target.v_yaw, params, state);
  const double    selected_delta  = delta_angles[static_cast<std::size_t>(selected_id)];
  const Eigen::Vector3d chosen_armor = outpostNormalChosenArmorWithDelay(
    ekf_center, target_yaw, armor_positions, selected_id, target, params);

  if (chosen_armor.norm() < 0.1) return false;

  const double distance = chosen_armor.norm();
  double       cmd_yaw = 0.0, cmd_pitch = 0.0;
  outpostNormalBuildPitchYawCmd(
    target, chosen_armor, params, compensator, manual_compensator, &cmd_yaw, &cmd_pitch);

  double tol_rad = 0.0, yaw_err = 0.0;
  const bool fire_advice = outpostNormalFireAdviceIsOnTarget(
    rpy[2], cmd_yaw, selected_delta, distance, target.v_yaw, params, state,
    debug_solver ? &tol_rad : nullptr, debug_solver ? &yaw_err : nullptr);

  const Eigen::Vector3d raw_armor(
    target.tracked_armor_x, target.tracked_armor_y, target.tracked_armor_z);

  cmd_out->header      = target.header;
  cmd_out->distance    = distance;
  cmd_out->yaw         = cmd_yaw * 180.0 / M_PI;
  cmd_out->pitch       = cmd_pitch * 180.0 / M_PI;
  cmd_out->yaw_diff    = (cmd_yaw - rpy[2]) * 180.0 / M_PI;
  cmd_out->pitch_diff  = (cmd_pitch - rpy[1]) * 180.0 / M_PI;
  cmd_out->fire_advice = fire_advice;

  if (debug_solver) {
    PKA_DEBUG("armor_solver",
              "[Outpost::nonCenter] slot={} delta={:.2f}deg "
              "armor=({:.3f},{:.3f},{:.3f}) raw_armor=({:.3f},{:.3f},{:.3f}) "
              "yaw={:.3f}deg pitch={:.3f}deg tol={:.3f}deg yaw_err={:.3f}deg fire={}",
              selected_id, selected_delta * 180.0 / M_PI,
              chosen_armor.x(), chosen_armor.y(), chosen_armor.z(),
              raw_armor.x(), raw_armor.y(), raw_armor.z(),
              cmd_yaw * 180.0 / M_PI, cmd_pitch * 180.0 / M_PI,
              tol_rad * 180.0 / M_PI, yaw_err * 180.0 / M_PI,
              fire_advice);
  }

  return true;
}

// ---------------------------------------------------------------------------
// Single-plate mode：4D [x,y,z,yaw] EKF，掉识别时 predict 保持 cmd 与 Target 连续
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
  rm_interfaces::msg::Target * target_out) noexcept
{
  (void)dt_sec;
  if (!cmd_out || !runtime) return false;
  if (params.mode != OutpostMode::SINGLE) return false;

  auto & im = *runtime->impl_;

  const rm_interfaces::msg::Armor * armor_ptr = nullptr;
  double best_dist_sq = 0.0;
  for (const auto & a : armors) {
    if (!isOutpostId(a.number, id_params.id)) continue;
    const auto & pp = a.pose.position;
    const double d2 = pp.x * pp.x + pp.y * pp.y + pp.z * pp.z;
    if (!armor_ptr || d2 < best_dist_sq) {
      armor_ptr    = &a;
      best_dist_sq = d2;
    }
  }

  std::array<double, 3> rpy{0.0, 0.0, 0.0};
  try {
    const auto tf = tf2_buffer->lookupTransform(
      target_frame, "gimbal_link", tf2::TimePointZero);
    tf2::Quaternion q;
    tf2::fromMsg(tf.transform.rotation, q);
    double roll{}, pitch{}, yaw{};
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    rpy[0] = roll;
    rpy[1] = -pitch;
    rpy[2] = yaw;
  } catch (const tf2::TransformException & ex) {
    PKA_ERROR("armor_solver", "[Outpost::singlePlate/EKF] TF2 error: {}", ex.what());
    return false;
  }

  Eigen::Matrix<double, 4, 1> x_est;

  if (!im.initialized) {
    if (!armor_ptr) return false;
    const auto & pp = armor_ptr->pose.position;
    if (std::hypot(std::hypot(pp.x, pp.y), pp.z) < 0.05) return false;
    im.syncNoise(params);
    im.ensureFilter(params);
    Eigen::Matrix<double, 4, 1> z0;
    z0 << pp.x, pp.y, pp.z, quatYaw(armor_ptr->pose.orientation);
    im.ekf->setState(z0);
    im.initialized = true;
    // init tracker on first good measurement
    im.tracker.tracker_state = OutpostSolverState::TrackerState::DETECTING;
    im.tracker.detect_count = 1;
    im.tracker.lost_time_accum = 0.0;
    x_est = z0;
  } else {
    im.syncNoise(params);
    if (im.ekf) {
      im.ekf->setPredictFunc(Single4Predict{});
    }
    Eigen::Matrix<double, 4, 1> x_pri = im.ekf->predict();

    // prepare measurement if available
    bool has_meas = false;
    Eigen::Matrix<double, 4, 1> z;
    Eigen::Vector3d meas_pos(0.0, 0.0, 0.0);
    double raw_meas_yaw = 0.0;
    if (armor_ptr) {
      const auto & pp = armor_ptr->pose.position;
      if (std::hypot(std::hypot(pp.x, pp.y), pp.z) >= 0.05) {
        has_meas = true;
        meas_pos = Eigen::Vector3d(pp.x, pp.y, pp.z);
        raw_meas_yaw = quatYaw(armor_ptr->pose.orientation);
        z << pp.x, pp.y, pp.z, x_pri(3) + angles::shortest_angular_distance(x_pri(3), raw_meas_yaw);
      }
    }

    // matching check against prior
    bool matched = false;
    if (has_meas) {
      const double err_x = std::abs(meas_pos.x() - x_pri(0));
      const double err_y = std::abs(meas_pos.y() - x_pri(1));
      const double err_z = std::abs(meas_pos.z() - x_pri(2));
      const double err_yaw = std::abs(angles::shortest_angular_distance(x_pri(3), raw_meas_yaw));
      const double err_yaw_deg = err_yaw * 180.0 / M_PI;
      const bool is_mismatch = (err_x > params.matching_thresh_x) ||
                               (err_y > params.matching_thresh_y) ||
                               (err_z > params.matching_thresh_z) ||
                               (err_yaw_deg > params.matching_thresh_yaw_deg);
      matched = !is_mismatch;
    }

    // Tracker state machine
    {
      using TS = OutpostSolverState::TrackerState;
      if (im.tracker.tracker_state == TS::DETECTING) {
        if (matched) {
          im.tracker.detect_count++;
          if (im.tracker.detect_count > params.tracking_thres) {
            im.tracker.detect_count = 0;
            im.tracker.tracker_state = TS::TRACKING;
            PKA_DEBUG("armor_solver", "[Outpost::Tracker] state: TRACKING id={}", id_params.id);
          } else {
            if (debug_solver) {
              PKA_DEBUG("armor_solver", "[Outpost::Tracker] DETECTING id={} detect_count={}/{})",
                        id_params.id, im.tracker.detect_count, params.tracking_thres);
            }
          }
        } else {
          im.tracker.detect_count = 0;
          im.tracker.tracker_state = TS::LOST;
          PKA_DEBUG("armor_solver", "[Outpost::Tracker] state: LOST id={}", id_params.id);
          // On LOST: reset EKF, clear cmd and mark target not tracking
          im.ekf.reset();
          im.initialized = false;
          if (cmd_out) {
            cmd_out->header.frame_id = target_frame;
            cmd_out->distance = -1;
            cmd_out->yaw = 0;
            cmd_out->pitch = 0;
            cmd_out->yaw_diff = 0;
            cmd_out->pitch_diff = 0;
            cmd_out->fire_advice = false;
          }
          if (target_out) {
            target_out->tracking = false;
            target_out->armors_num = 0;
            target_out->header.frame_id = target_frame;
          }
          return true;
        }
      } else if (im.tracker.tracker_state == TS::TRACKING) {
        if (!matched) {
          im.tracker.tracker_state = TS::TEMP_LOST;
          im.tracker.lost_time_accum = dt_sec;
          PKA_DEBUG("armor_solver", "[Outpost::Tracker] state: TEMP_LOST id={}", id_params.id);
        } else {
          if (debug_solver) {
            PKA_DEBUG("armor_solver", "[Outpost::Tracker] TRACKING id={} matched", id_params.id);
          }
        }
      } else if (im.tracker.tracker_state == TS::TEMP_LOST) {
        if (!matched) {
          im.tracker.lost_time_accum += dt_sec;
          if (im.tracker.lost_time_accum > params.single_lost_time_thres) {
            im.tracker.lost_time_accum = 0.0;
            im.tracker.tracker_state = TS::LOST;
            PKA_DEBUG("armor_solver", "[Outpost::Tracker] state: LOST id={} after timeout", id_params.id);
            im.ekf.reset();
            im.initialized = false;
            if (cmd_out) {
              cmd_out->header.frame_id = target_frame;
              cmd_out->distance = -1;
              cmd_out->yaw = 0;
              cmd_out->pitch = 0;
              cmd_out->yaw_diff = 0;
              cmd_out->pitch_diff = 0;
              cmd_out->fire_advice = false;
            }
            if (target_out) {
              target_out->tracking = false;
              target_out->armors_num = 0;
              target_out->header.frame_id = target_frame;
            }
            return true;
          } else {
            if (debug_solver) {
              PKA_DEBUG("armor_solver", "[Outpost::Tracker] TEMP_LOST id={} lost_time={:.3f}/{:.3f}s",
                        id_params.id, im.tracker.lost_time_accum, params.single_lost_time_thres);
            }
          }
        } else {
          im.tracker.tracker_state = TS::TRACKING;
          im.tracker.lost_time_accum = 0.0;
          PKA_DEBUG("armor_solver", "[Outpost::Tracker] state: TRACKING id={} (recovered)", id_params.id);
        }
      } else { // LOST
        if (matched) {
          im.tracker.tracker_state = TS::DETECTING;
          im.tracker.detect_count = 1;
          PKA_DEBUG("armor_solver", "[Outpost::Tracker] state: DETECTING id={} (new detect)", id_params.id);
        }
      }
    }

    // measurement accepted: update EKF only when matched and we have a measurement
    if (has_meas && matched) {
      if (im.ekf) {
        x_est = im.ekf->update(z);
      } else {
        x_est = x_pri;
      }
    } else {
      x_est = x_pri;
    }
  }

  const Eigen::Vector3d armor_pos(x_est(0), x_est(1), x_est(2));
  if (armor_pos.norm() < 0.05) return false;

  const double yaw_raw  = std::atan2(armor_pos.y(), armor_pos.x());
  double       pitch    = std::atan2(armor_pos.z(), armor_pos.head(2).norm());
  const double distance = armor_pos.norm();

  if (compensator) {
    double tmp = pitch;
    if (compensator->compensate(armor_pos, tmp)) {
      pitch = tmp;
    }
  }

  double pitch_off = 0.0, yaw_off = 0.0;
  if (manual_compensator) {
    const auto ao = manual_compensator->angleHardCorrect(
      armor_pos.head(2).norm(), armor_pos.z());
    pitch_off = ao[0] * M_PI / 180.0;
    yaw_off   = ao[1] * M_PI / 180.0;
  }

  const double cmd_pitch = pitch + pitch_off + params.pitch_offset * M_PI / 180.0;
  const double cmd_yaw   = angles::normalize_angle(
    yaw_raw + yaw_off + params.yaw_offset * M_PI / 180.0);

  cmd_out->header.frame_id = target_frame;
  cmd_out->distance        = distance;
  cmd_out->yaw             = cmd_yaw * 180.0 / M_PI;
  cmd_out->pitch           = cmd_pitch * 180.0 / M_PI;
  cmd_out->yaw_diff        = (cmd_yaw - rpy[2]) * 180.0 / M_PI;
  cmd_out->pitch_diff      = (cmd_pitch - rpy[1]) * 180.0 / M_PI;
  cmd_out->fire_advice = 0;
  if(x_est(3) < -0.7){
    cmd_out->fire_advice = std::abs(x_est(3) + 0.7) <= params.single_fire_max_yaw;
    cmd_out->fire_advice = std::abs(cmd_out->pitch_diff) <= params.single_fire_max_pitch;
  }

  if (target_out) {
    target_out->id           = id_params.id;
    target_out->tracking     = true;
    target_out->armors_num   = 1;
    target_out->position.x   = x_est(0);
    target_out->position.y   = x_est(1);
    target_out->position.z   = x_est(2);
    target_out->velocity.x   = 0.0;
    target_out->velocity.y   = 0.0;
    target_out->velocity.z   = 0.0;
    target_out->yaw          = x_est(3);
    target_out->v_yaw        = 0.0;
    target_out->radius_1     = 0.26;
    target_out->radius_2     = 0.26;
    target_out->d_zc         = 0.0;
    target_out->d_za         = 0.0;
    target_out->tracked_armor_x = x_est(0);
    target_out->tracked_armor_y = x_est(1);
    target_out->tracked_armor_z = x_est(2);
  }

  if (debug_solver) {
    PKA_DEBUG(
      "armor_solver",
      "[Outpost::singlePlate/EKF] pos=({:.3f},{:.3f},{:.3f}) yaw_f={:.4f}rad "
      "cmd_yaw={:.1f}deg cmd_pitch={:.1f}deg has_meas={} fire={}",
      armor_pos.x(), armor_pos.y(), armor_pos.z(), x_est(3),
      cmd_out->yaw, cmd_out->pitch, static_cast<bool>(armor_ptr), cmd_out->fire_advice);
  }
  return true;
}

// ---------------------------------------------------------------------------
// RViz visualization
// ---------------------------------------------------------------------------

void appendOutpostFilteredArmorsStripMarkers(
  const rm_interfaces::msg::Target & target,
  const OutpostParams & p,
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  visualization_msgs::msg::MarkerArray * markers,
  visualization_msgs::msg::Marker * last_marker_out)
{
  if (!markers || target.armors_num != 3 || !isOutpostId(target.id, p.id)) return;

  const double yaw  = target.yaw;
  const double r    = target.radius_1;
  const double xc   = target.position.x;
  const double yc   = target.position.y;
  const double zc   = target.position.z;
  const double d_zc = target.d_zc;
  const double d_za = target.d_za;

  for (std::size_t i = 0; i < 3; ++i) {
    const double ang = yaw + static_cast<double>(i) * (2.0 * M_PI / 3.0);
    const double px  = xc - r * std::cos(ang);
    const double py  = yc - r * std::sin(ang);
    const double pz  = zc + d_zc + static_cast<double>(i) * d_za;

    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp    = stamp;
    m.ns              = "filtered_armors";
    m.id              = static_cast<int>(i);
    m.type            = visualization_msgs::msg::Marker::CUBE;
    m.action          = visualization_msgs::msg::Marker::ADD;
    m.scale.x         = 0.03;
    m.scale.y         = 0.135;
    m.scale.z         = 0.125;
    m.color.a         = 1.0;
    m.color.b         = 1.0;
    m.pose.position.x = px;
    m.pose.position.y = py;
    m.pose.position.z = pz;
    tf2::Quaternion q;
    q.setRPY(0.0, p.plate_pitch_rad, ang);
    m.pose.orientation = tf2::toMsg(q);
    markers->markers.push_back(m);
    if (last_marker_out && i == 2) *last_marker_out = m;
  }
}

}  // namespace pka::auto_aim
