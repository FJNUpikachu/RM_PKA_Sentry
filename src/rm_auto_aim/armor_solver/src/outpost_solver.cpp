// outpost_solver.cpp — RMUC 2026 前哨站：纯旋转模型（6-D EKF）
//
// Physical model
// ──────────────
//   三块装甲板，绕固定竖轴匀速旋转，间隔 120°，旋转半径 R。
//   高度：各板 z 坐标不同，但建模时取三板均值 zc 作为旋转轴高度。
//
// EKF state (6-D)
// ───────────────
//   x = [xc, yc, zc, yaw, v_yaw, r]
//
//   predict (CONSTANT_ROTATION):
//     xc, yc, zc — 保持不变（前哨站固定原地旋转）
//     yaw         += v_yaw * dt
//     v_yaw, r   — 保持不变
//
//   measure (slot-0 等效，输入由 outpostPrepareEKFMeasurement 预处理):
//     z[0] = xc - r·cos(yaw)
//     z[1] = yc - r·sin(yaw)
//     z[2] = zc
//     z[3] = yaw
//
// Tracking strategy
// ─────────────────
//   Phase 1 (DETECTING / accumulation):
//     每次看到新的不同板（yaw 差 > 0.7 rad）就加入缓冲区，最多收集三块。
//     三块到齐后，等第四次出现（即第一块板循环回来），此时：
//       xc = mean(xa_i + r·cos(yaw_i))，yc = mean(...)，zc = mean(za_i)
//     初始化 EKF，进入 TRACKING。
//
//   Phase 2 (TRACKING / TEMP_LOST):
//     EKF predict → 三 slot 匹配 → slot-0 重映射 → EKF update。
//
// Licensed under the Apache License, Version 2.0

#include "armor_solver/outpost_solver.hpp"

#include <cmath>
#include <algorithm>

#include <angles/angles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "rm_utils/pkaLoggerCenter.hpp"

namespace pka::auto_aim {

bool debug_outpost = false;

namespace {

// Convert quaternion to yaw (consistent with Tracker convention)
double quatYaw(const geometry_msgs::msg::Quaternion & q)
{
  tf2::Quaternion tf_q;
  tf2::fromMsg(q, tf_q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  return yaw;
}

}  // namespace

// ---------------------------------------------------------------------------
// Parameter management
// ---------------------------------------------------------------------------

void declareOutpostParameters(rclcpp::Node & node)
{
  node.declare_parameter("outpost.id",              std::string("outpost"));
  node.declare_parameter("outpost.radius_m",         0.275);
  node.declare_parameter("outpost.plate_pitch_rad",  -0.2617993877991494);
  node.declare_parameter("outpost.mode",             1);
  node.declare_parameter("outpost.fire_yaw_max_deg", 8.0);
  node.declare_parameter("outpost.detect.same_plate_yaw_thresh_rad", 0.7);
  node.declare_parameter("outpost.detect.require_gap_for_new_plate", true);
  node.declare_parameter("outpost.detect.min_gap_frames", 1);
  node.declare_parameter("outpost.detect.new_plate_min_yaw_jump_rad", 1.2);
  node.declare_parameter("outpost.detect.force_next_on_gap", true);
  node.declare_parameter("outpost.detect.force_next_min_yaw_jump_rad", 0.15);
  node.declare_parameter("outpost.detect.force_next_min_z_diff_m", 0.03);
  node.declare_parameter("outpost.detect.require_return_to_first_plate", true);
  node.declare_parameter("outpost.detect.return_to_first_yaw_thresh_rad", 0.7);
  node.declare_parameter("outpost.detect.rebuild_on_invalid_height", true);
  node.declare_parameter("outpost.detect.min_valid_dza_m", 0.02);
}

OutpostParams loadOutpostParams(rclcpp::Node & node)
{
  OutpostParams p;
  p.id               = node.get_parameter("outpost.id").as_string();
  p.radius_m         = node.get_parameter("outpost.radius_m").as_double();
  p.plate_pitch_rad  = node.get_parameter("outpost.plate_pitch_rad").as_double();
  p.fire_yaw_max_deg = node.get_parameter("outpost.fire_yaw_max_deg").as_double();
  p.detect_same_plate_yaw_thresh_rad =
    node.get_parameter("outpost.detect.same_plate_yaw_thresh_rad").as_double();
  p.detect_require_gap_for_new_plate =
    node.get_parameter("outpost.detect.require_gap_for_new_plate").as_bool();
  p.detect_min_gap_frames =
    node.get_parameter("outpost.detect.min_gap_frames").as_int();
  p.detect_new_plate_min_yaw_jump_rad =
    node.get_parameter("outpost.detect.new_plate_min_yaw_jump_rad").as_double();
  p.detect_force_next_on_gap =
    node.get_parameter("outpost.detect.force_next_on_gap").as_bool();
  p.detect_force_next_min_yaw_jump_rad =
    node.get_parameter("outpost.detect.force_next_min_yaw_jump_rad").as_double();
  p.detect_force_next_min_z_diff_m =
    node.get_parameter("outpost.detect.force_next_min_z_diff_m").as_double();
  p.detect_require_return_to_first_plate =
    node.get_parameter("outpost.detect.require_return_to_first_plate").as_bool();
  p.detect_return_to_first_yaw_thresh_rad =
    node.get_parameter("outpost.detect.return_to_first_yaw_thresh_rad").as_double();
  p.detect_rebuild_on_invalid_height =
    node.get_parameter("outpost.detect.rebuild_on_invalid_height").as_bool();
  p.detect_min_valid_dza_m =
    node.get_parameter("outpost.detect.min_valid_dza_m").as_double();
  if (debug_outpost) {
    PKA_DEBUG("armor_solver",
              "[Outpost::params] id={} R={:.3f}m pitch={:.4f}rad fire_yaw_max={:.1f}deg "
              "same_th={:.3f} gap_req={} gap_min={} jump_min={:.3f} "
              "force_gap={} force_jump_min={:.3f} force_z_min={:.3f} "
              "ret_first={} ret_first_th={:.3f} rebuild_bad_h={} min_dza={:.3f}",
              p.id, p.radius_m, p.plate_pitch_rad, p.fire_yaw_max_deg,
              p.detect_same_plate_yaw_thresh_rad,
              p.detect_require_gap_for_new_plate,
              p.detect_min_gap_frames,
              p.detect_new_plate_min_yaw_jump_rad,
              p.detect_force_next_on_gap,
              p.detect_force_next_min_yaw_jump_rad,
              p.detect_force_next_min_z_diff_m,
              p.detect_require_return_to_first_plate,
              p.detect_return_to_first_yaw_thresh_rad,
              p.detect_rebuild_on_invalid_height,
              p.detect_min_valid_dza_m);
  }
  return p;
}

OutpostMode loadOutpostMode(rclcpp::Node & node)
{
  const int m = node.get_parameter("outpost.mode").as_int();
  return (m == 0) ? OutpostMode::SINGLE_PLATE : OutpostMode::FUSION;
}

// ---------------------------------------------------------------------------
// Filtering helpers
// ---------------------------------------------------------------------------

std::vector<rm_interfaces::msg::Armor> filterOutpostArmors(
  const rm_interfaces::msg::Armors::SharedPtr & msg,
  const std::string & outpost_id)
{
  std::vector<rm_interfaces::msg::Armor> out;
  if (!msg) return out;
  for (const auto & a : msg->armors) {
    if (a.number == outpost_id) out.push_back(a);
  }
  return out;
}

bool isOutpostId(const std::string & id, const std::string & outpost_id)
{
  return id == outpost_id;
}

// ---------------------------------------------------------------------------
// OutpostPlateBuffer
// ---------------------------------------------------------------------------

void OutpostPlateBuffer::reset()
{
  plates.clear();
  lost_count = 0;
  gap_count = 0;
  had_armor_last_frame = false;
  has_last_yaw = false;
  last_yaw = 0.0;
}

bool OutpostPlateBuffer::tryAdd(
  const double xa, const double ya, const double za, const double yaw,
  const double same_yaw_thresh_rad)
{
  // Check if this plate is already known
  for (const auto & p : plates) {
    if (std::abs(angles::shortest_angular_distance(p.yaw, yaw)) < same_yaw_thresh_rad) {
      return false;
    }
  }
  if (static_cast<int>(plates.size()) < NUM_PLATES) {
    plates.push_back({xa, ya, za, yaw});
    return true;
  }
  // Buffer is already full — should not happen in normal flow
  return false;
}

bool OutpostPlateBuffer::forceAdd(
  const double xa, const double ya, const double za, const double yaw)
{
  if (static_cast<int>(plates.size()) < NUM_PLATES) {
    plates.push_back({xa, ya, za, yaw});
    return true;
  }
  return false;
}

bool OutpostPlateBuffer::hasEnoughZSeparation(const double za, const double min_z_diff_m) const
{
  for (const auto & p : plates) {
    if (std::abs(za - p.za) < min_z_diff_m) {
      return false;
    }
  }
  return true;
}

bool OutpostPlateBuffer::matchesKnown(const double yaw, const double same_yaw_thresh_rad) const
{
  for (const auto & p : plates) {
    if (std::abs(angles::shortest_angular_distance(p.yaw, yaw)) < same_yaw_thresh_rad) {
      return true;
    }
  }
  return false;
}

// ---------------------------------------------------------------------------
// EKF initialization from 3-plate buffer
// ---------------------------------------------------------------------------

void outpostInitStateFromBuffer(
  const OutpostParams & p,
  const OutpostPlateBuffer & buf,
  const double current_obs_yaw,
  Eigen::VectorXd * state_out)
{
  if (!state_out || buf.plates.empty()) return;

  const double r = p.radius_m;
  double xc_sum = 0.0, yc_sum = 0.0, zc_sum = 0.0;
  for (const auto & pl : buf.plates) {
    xc_sum += pl.xa + r * std::cos(pl.yaw);
    yc_sum += pl.ya + r * std::sin(pl.yaw);
    zc_sum += pl.za;
  }
  const double n  = static_cast<double>(buf.plates.size());
  const double xc = xc_sum / n;
  const double yc = yc_sum / n;
  const double zc = zc_sum / n;

  state_out->resize(X_N_OUTPOST);
  *state_out << xc, yc, zc, current_obs_yaw, 0.0, r;

  if (debug_outpost) {
    PKA_DEBUG("armor_solver",
              "[Outpost::initFromBuf] {} plates averaged:"
              " center=({:.3f},{:.3f},{:.3f}) yaw={:.4f} r={:.3f}",
              buf.plates.size(), xc, yc, zc, current_obs_yaw, r);
  }
}

void outpostComputeHeightOffsetsFromBuffer(
  const OutpostPlateBuffer & buf,
  double * d_zc_out,
  double * d_za_out)
{
  if (!d_zc_out || !d_za_out) return;
  *d_zc_out = 0.0;
  *d_za_out = 0.0;
  if (buf.plates.size() < 3) return;

  // Use the first three accepted-plate observations directly (in observed order),
  // so model height follows exactly the initialized 3-plate cache.
  const double z0 = buf.plates[0].za;
  const double z1 = buf.plates[1].za;
  const double z2 = buf.plates[2].za;
  const double zc = (z0 + z1 + z2) / 3.0;

  *d_zc_out = z0 - zc;
  *d_za_out = ((z1 - z0) + (z2 - z1)) * 0.5;

  if (debug_outpost) {
    PKA_DEBUG("armor_solver",
              "[Outpost::heightInit] z_slots=({:.3f},{:.3f},{:.3f}) -> zc={:.3f} d_zc={:.3f} d_za={:.3f}",
              z0, z1, z2, zc, *d_zc_out, *d_za_out);
  }
}

// ---------------------------------------------------------------------------
// Detecting phase handler
// ---------------------------------------------------------------------------

OutpostDetectingResult outpostHandleDetecting(
  OutpostPlateBuffer & buf,
  const int lost_thres,
  OutpostStateEKF & ekf,
  const OutpostParams & p,
  const bool has_armor,
  const rm_interfaces::msg::Armor & armor,
  Eigen::VectorXd * state_out,
  const bool debug) noexcept
{
  const bool trace = debug || debug_outpost;

  // ── No armor this frame ─────────────────────────────────────────────────
  if (!has_armor) {
    if (buf.had_armor_last_frame && trace) {
      PKA_INFO("armor_solver",
               "[Outpost::build] edge 1->0 detected, arm next-plate-by-gap rule");
    }
    buf.lost_count++;
    buf.gap_count++;
    buf.had_armor_last_frame = false;
    if (trace) {
      PKA_DEBUG("armor_solver",
                "[Outpost::detecting] no armor, lost_count={}/{} gap_count={} plates={}/{}",
                buf.lost_count, lost_thres,
                buf.gap_count,
                buf.plates.size(), OutpostPlateBuffer::NUM_PLATES);
      // Keep a readable progress line for operators while waiting the next plate.
      if (buf.gap_count == 1 || (buf.gap_count % 5) == 0) {
        PKA_INFO("armor_solver",
                 "[Outpost::build] waiting next plate: plates={}/{} gap_count={} lost_count={}/{}",
                 buf.plates.size(), OutpostPlateBuffer::NUM_PLATES,
                 buf.gap_count, buf.lost_count, lost_thres);
      }
    }
    if (buf.lost_count > lost_thres) {
      if (trace) {
        PKA_WARN("armor_solver",
                 "[Outpost::build] timeout -> GO_LOST, buffered plates cleared (plates={})",
                 buf.plates.size());
      }
      buf.reset();
      PKA_DEBUG("armor_solver",
                "[Outpost::detecting] timeout -> GO_LOST, reset plate buffer");
      return OutpostDetectingResult::GO_LOST;
    }
    return OutpostDetectingResult::KEEP_DETECTING;
  }

  // ── Armor observed ───────────────────────────────────────────────────────
  buf.lost_count = 0;
  const double xa  = armor.pose.position.x;
  const double ya  = armor.pose.position.y;
  const double za  = armor.pose.position.z;
  const double yaw = quatYaw(armor.pose.orientation);
  const bool enough_gap =
    (!p.detect_require_gap_for_new_plate) || (buf.gap_count >= p.detect_min_gap_frames);
  const double yaw_jump = buf.has_last_yaw ?
    std::abs(angles::shortest_angular_distance(buf.last_yaw, yaw)) : M_PI;
  const bool enough_yaw_jump = yaw_jump >= p.detect_new_plate_min_yaw_jump_rad;
  const bool rising_after_gap = (!buf.had_armor_last_frame) && (buf.gap_count >= p.detect_min_gap_frames);
  const bool force_gap_jump_ok = yaw_jump >= p.detect_force_next_min_yaw_jump_rad;
  const bool force_gap_z_ok = buf.hasEnoughZSeparation(za, p.detect_force_next_min_z_diff_m);
  const bool force_next_by_gap =
    p.detect_force_next_on_gap && rising_after_gap && force_gap_jump_ok && force_gap_z_ok;
  const bool matches_known = buf.matchesKnown(yaw, p.detect_same_plate_yaw_thresh_rad);

  if (trace) {
    PKA_INFO("armor_solver",
             "[Outpost::build] obs yaw={:.4f} pos=({:.3f},{:.3f},{:.3f}) "
             "plates={}/{} gate(gap={} jump={} known={} force_gap={}) "
             "th(gap>={} jump>={:.3f} same<{:.3f} force_jump>={:.3f} force_z>={:.3f})",
             yaw, xa, ya, za,
             buf.plates.size(), OutpostPlateBuffer::NUM_PLATES,
             enough_gap, enough_yaw_jump, matches_known, force_next_by_gap,
             p.detect_min_gap_frames,
             p.detect_new_plate_min_yaw_jump_rad,
             p.detect_same_plate_yaw_thresh_rad,
             p.detect_force_next_min_yaw_jump_rad,
             p.detect_force_next_min_z_diff_m);
    PKA_DEBUG("armor_solver",
              "[Outpost::detecting] obs yaw={:.4f} gap_ok={} yaw_jump={:.4f} jump_ok={} known={}",
              yaw, enough_gap, yaw_jump, enough_yaw_jump, matches_known);
  }

  // Buffer already full:
  // by default, require "4th observation = first cached plate" before START_TRACKING.
  if (buf.isFull()) {
    const double first_yaw = buf.plates.empty() ? yaw : buf.plates.front().yaw;
    const double return_first_yaw_th = p.detect_return_to_first_yaw_thresh_rad;
    const bool match_first =
      std::abs(angles::shortest_angular_distance(first_yaw, yaw)) < return_first_yaw_th;
    const bool can_start_tracking =
      (!p.detect_require_return_to_first_plate) || match_first;

    if (!can_start_tracking) {
      if (trace) {
        PKA_INFO("armor_solver",
                 "[Outpost::build] 3 plates cached, waiting 4th==1st: "
                 "first_yaw={:.4f} current_yaw={:.4f} diff={:.4f} th={:.4f}",
                 first_yaw, yaw,
                 std::abs(angles::shortest_angular_distance(first_yaw, yaw)),
                 return_first_yaw_th);
      }
      // Keep target preview stable while waiting: use cached 3-plate average center/z.
      outpostInitStateFromBuffer(p, buf, first_yaw, state_out);
      ekf.setState(*state_out);
      buf.had_armor_last_frame = true;
      buf.has_last_yaw = true;
      buf.last_yaw = yaw;
      return OutpostDetectingResult::KEEP_DETECTING;
    }

    outpostInitStateFromBuffer(p, buf, yaw, state_out);
    ekf.setState(*state_out);
    buf.gap_count = 0;
    buf.has_last_yaw = true;
    buf.last_yaw = yaw;
    if (trace) {
      if (buf.plates.size() == 3) {
        PKA_INFO("armor_solver",
                 "[Outpost::build] 4th appearance -> START_TRACKING. "
                 "buffer yaws=[{:.3f},{:.3f},{:.3f}] current={:.3f}",
                 buf.plates[0].yaw, buf.plates[1].yaw, buf.plates[2].yaw, yaw);
      } else {
        PKA_INFO("armor_solver",
                 "[Outpost::build] 4th appearance -> START_TRACKING. current={:.3f}",
                 yaw);
      }
      PKA_DEBUG("armor_solver",
                "[Outpost::detecting] 4th appearance yaw={:.4f} -> START_TRACKING "
                "state=({:.3f},{:.3f},{:.3f},{:.4f},{:.4f},{:.3f})",
                yaw, (*state_out)(0), (*state_out)(1), (*state_out)(2),
                (*state_out)(3), (*state_out)(4), (*state_out)(5));
    }
    return OutpostDetectingResult::START_TRACKING;
  }

  // Still accumulating — try to add as a new plate
  const bool can_add_new = enough_gap && enough_yaw_jump && !matches_known;
  bool added = false;
  if (force_next_by_gap) {
    added = buf.forceAdd(xa, ya, za, yaw);
  } else if (can_add_new) {
    added = buf.tryAdd(xa, ya, za, yaw, p.detect_same_plate_yaw_thresh_rad);
  }
  if (trace) {
    if (!added) {
      PKA_INFO("armor_solver",
               "[Outpost::build] reject plate: gap_ok={} jump_ok={} known={} force_gap={} "
               "force_z_ok={} (gap_count={} yaw_jump={:.4f})",
               enough_gap, enough_yaw_jump, matches_known, force_next_by_gap,
               force_gap_z_ok, buf.gap_count, yaw_jump);
    } else {
      PKA_INFO("armor_solver",
               "[Outpost::build] accept {} -> plates={}/{}",
               force_next_by_gap ? "NEW_PLATE_BY_GAP" : "NEW_PLATE",
               buf.plates.size(), OutpostPlateBuffer::NUM_PLATES);
    }
    PKA_DEBUG("armor_solver",
              "[Outpost::detecting] armor yaw={:.4f} pos=({:.3f},{:.3f},{:.3f}) "
              "{} plates={}/{}",
              yaw, xa, ya, za,
              added ? "NEW_PLATE" : "known_plate",
              buf.plates.size(), OutpostPlateBuffer::NUM_PLATES);
  }
  if (added) {
    buf.gap_count = 0;
  }
  buf.had_armor_last_frame = true;
  buf.has_last_yaw = true;
  buf.last_yaw = yaw;

  // Update rough EKF state estimate (single-plate center approximation)
  // Used only for debug display; overwritten by outpostInitStateFromBuffer on START_TRACKING.
  const double r = p.radius_m;
  state_out->resize(X_N_OUTPOST);
  *state_out << (xa + r * std::cos(yaw)), (ya + r * std::sin(yaw)), za,
                yaw, 0.0, r;
  ekf.setState(*state_out);

  return OutpostDetectingResult::KEEP_DETECTING;
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
  // 新模型：d_zc=0, d_za=0（所有板高度 = center.z），接口保持向后兼容。
  const double ang = target_yaw + static_cast<double>(slot_index) * (2.0 * M_PI / 3.0);
  const double dz  = d_zc + static_cast<double>(slot_index) * d_za;
  return center + Eigen::Vector3d(-r * std::cos(ang), -r * std::sin(ang), dz);
}

Eigen::Vector3d outpostSlotWorldPosition(
  const Eigen::VectorXd & state_6d,
  const std::size_t slot_k) noexcept
{
  if (state_6d.size() < X_N_OUTPOST) return Eigen::Vector3d::Zero();
  const double xc  = state_6d(0);
  const double yc  = state_6d(1);
  const double zc  = state_6d(2);
  const double yaw = state_6d(3);
  const double r   = state_6d(5);
  const double ang = yaw + static_cast<double>(slot_k) * (2.0 * M_PI / 3.0);
  return Eigen::Vector3d(xc - r * std::cos(ang),
                         yc - r * std::sin(ang),
                         zc);
}

// ---------------------------------------------------------------------------
// Tracker helpers
// ---------------------------------------------------------------------------

void outpostFindBestSlotMatch(
  const Eigen::VectorXd & ekf_pred,
  const rm_interfaces::msg::Armor & armor,
  double * pos_diff_out,
  double * yaw_diff_out) noexcept
{
  if (!pos_diff_out || !yaw_diff_out || ekf_pred.size() < X_N_OUTPOST) return;

  const double base_yaw = ekf_pred(3);
  const Eigen::Vector3d meas_pos(armor.pose.position.x,
                                  armor.pose.position.y,
                                  armor.pose.position.z);
  const double armor_yaw = quatYaw(armor.pose.orientation);

  double best_pd = 1e18, best_yd = 1e18;
  for (std::size_t slot = 0; slot < 3; ++slot) {
    const Eigen::Vector3d slot_pos = outpostSlotWorldPosition(ekf_pred, slot);
    const double slot_yaw = base_yaw + static_cast<double>(slot) * (2.0 * M_PI / 3.0);
    const double pd = (slot_pos - meas_pos).norm();
    const double yd = std::abs(angles::shortest_angular_distance(slot_yaw, armor_yaw));
    if (pd < best_pd) {
      best_pd = pd;
      best_yd = yd;
    }
  }
  *pos_diff_out = best_pd;
  *yaw_diff_out = best_yd;
}

Eigen::Vector4d outpostPrepareEKFMeasurement(
  const Eigen::VectorXd & ekf_pred,
  const rm_interfaces::msg::Armor & armor,
  const double measured_yaw) noexcept
{
  if (ekf_pred.size() < X_N_OUTPOST) {
    return Eigen::Vector4d(armor.pose.position.x, armor.pose.position.y,
                           armor.pose.position.z, measured_yaw);
  }

  const double xc       = ekf_pred(0);
  const double yc       = ekf_pred(1);
  const Eigen::Vector3d meas_pos(armor.pose.position.x,
                                  armor.pose.position.y,
                                  armor.pose.position.z);

  // Find the slot whose predicted position is closest to the observation
  std::size_t best_slot = 0;
  double best_pd = 1e18;
  for (std::size_t slot = 0; slot < 3; ++slot) {
    const Eigen::Vector3d slot_pos = outpostSlotWorldPosition(ekf_pred, slot);
    const double pd = (slot_pos - meas_pos).norm();
    if (pd < best_pd) {
      best_pd   = pd;
      best_slot = slot;
    }
  }

  // Remap XY to slot-0 equivalent (rotate by -best_slot * 2π/3)
  const double ang_off = static_cast<double>(best_slot) * (2.0 * M_PI / 3.0);
  const double c  = std::cos(-ang_off);
  const double s  = std::sin(-ang_off);
  const double dx = meas_pos.x() - xc;
  const double dy = meas_pos.y() - yc;
  const double xa_adj  = xc + c * dx - s * dy;
  const double ya_adj  = yc + s * dx + c * dy;
  // Pass raw za — EKF measurement noise absorbs plate-height variation;
  // zc (average height) will remain stable with large r_z noise parameter.
  const double za_raw  = meas_pos.z();
  const double yaw_adj = measured_yaw - ang_off;

  if (debug_outpost) {
    PKA_DEBUG("armor_solver",
              "[Outpost::prepareEKF] slot={} ang_off={:.4f}rad "
              "adj=({:.3f},{:.3f},{:.3f}) yaw_adj={:.4f}",
              best_slot, ang_off, xa_adj, ya_adj, za_raw, yaw_adj);
  }

  return Eigen::Vector4d(xa_adj, ya_adj, za_raw, yaw_adj);
}

// ---------------------------------------------------------------------------
// State clamp
// ---------------------------------------------------------------------------

void outpostClampTargetState(
  OutpostStateEKF & ekf,
  Eigen::VectorXd & state,
  const double v_yaw_min, const double v_yaw_max,
  const double radius_min, const double radius_max,
  const bool debug_ekf) noexcept
{
  if (state.size() < X_N_OUTPOST) return;

  const Eigen::VectorXd before = state;
  state(4) = std::clamp(state(4), v_yaw_min, v_yaw_max);  // v_yaw
  state(5) = std::clamp(state(5), radius_min, radius_max); // r
  ekf.setState(state);

  if (debug_ekf) {
    PKA_DEBUG("armor_solver",
              "[EKF::state_clamp outpost] "
              "v_yaw {:.3f}->{:.3f}  r {:.3f}->{:.3f}",
              before(4), state(4),
              before(5), state(5));
  }
}

// ---------------------------------------------------------------------------
// Node helpers
// ---------------------------------------------------------------------------

void outpostApplyTargetFields(
  const OutpostParams & p,
  const OutpostMode mode,
  const Eigen::VectorXd & state,
  const double d_zc,
  const double d_za,
  const rm_interfaces::msg::Armor & /* tracked_armor */,
  rm_interfaces::msg::Target * target)
{
  if (!target || !isOutpostId(target->id, p.id)) return;
  if (state.size() < X_N_OUTPOST) return;

  // state = [xc, yc, zc, yaw, v_yaw, r]
  if (mode == OutpostMode::SINGLE_PLATE) {
    target->position.x = state(0);
    target->position.y = state(1);
    target->position.z = state(2);
    target->velocity.x = 0.0;
    target->velocity.y = 0.0;
    target->velocity.z = 0.0;
    target->yaw        = state(3);
    target->v_yaw      = 0.0;
    target->radius_1   = p.radius_m;
    target->radius_2   = p.radius_m;
    target->d_zc       = 0.0;
    target->d_za       = 0.0;
    target->armors_num = 1;
  } else {
    // FUSION: full 3-plate EKF state
    target->position.x = state(0);
    target->position.y = state(1);
    target->position.z = state(2);
    target->velocity.x = 0.0;
    target->velocity.y = 0.0;
    target->velocity.z = 0.0;
    target->yaw        = state(3);
    target->v_yaw      = state(4);
    target->radius_1   = state(5);
    target->radius_2   = state(5);
    target->d_zc       = d_zc;
    target->d_za       = d_za;
    target->armors_num = 3;
  }

  if (debug_outpost) {
    PKA_DEBUG("armor_solver",
              "[Outpost::target] mode={} armors_num={} "
              "center=({:.3f},{:.3f},{:.3f}) yaw={:.4f} v_yaw={:.4f} r={:.3f} d_zc={:.3f} d_za={:.3f}",
              (mode == OutpostMode::SINGLE_PLATE ? "SINGLE" : "FUSION"),
              target->armors_num,
              state(0), state(1), state(2),
              state(3), state(4), state(5),
              target->d_zc, target->d_za);
  }
}

void outpostFillDetectingTarget(
  const OutpostParams & p,
  const Eigen::VectorXd & ekf_state,
  rm_interfaces::msg::Target * target)
{
  // DETECTING 阶段发布非 tracking 的单板回退目标：
  // 便于 RViz 持续看到模型，且不触发 timerCallback 中的 solve/fire。
  if (!target || ekf_state.size() < X_N_OUTPOST) return;
  target->tracking = false;
  target->position.x = ekf_state(0);
  target->position.y = ekf_state(1);
  target->position.z = ekf_state(2);
  target->velocity.x = 0.0;
  target->velocity.y = 0.0;
  target->velocity.z = 0.0;
  target->yaw        = ekf_state(3);
  target->v_yaw      = 0.0;
  target->radius_1   = ekf_state(5);
  target->radius_2   = ekf_state(5);
  target->d_zc       = 0.0;
  target->d_za       = 0.0;
  target->armors_num = 1;
  target->id         = p.id;
}

void outpostApplySinglePlateFireConstraint(
  const OutpostParams & p,
  rm_interfaces::msg::GimbalCmd * gimbal_cmd) noexcept
{
  if (!gimbal_cmd) return;
  if (std::abs(gimbal_cmd->yaw_diff) > p.fire_yaw_max_deg) {
    gimbal_cmd->fire_advice = false;
    if (debug_outpost) {
      PKA_DEBUG("armor_solver",
                "[Outpost::fireCons] single-plate fire blocked: "
                "|yaw_diff|={:.2f}deg > max={:.2f}deg",
                std::abs(gimbal_cmd->yaw_diff), p.fire_yaw_max_deg);
    }
  }
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

  const double yaw = target.yaw;
  const double r   = target.radius_1;
  const double xc  = target.position.x;
  const double yc  = target.position.y;
  const double zc  = target.position.z;
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
