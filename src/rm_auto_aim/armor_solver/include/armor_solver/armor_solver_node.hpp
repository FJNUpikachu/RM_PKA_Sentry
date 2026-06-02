#ifndef ARMOR_SOLVER_SOLVER_NODE_HPP_
#define ARMOR_SOLVER_SOLVER_NODE_HPP_

// ros2
#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
// std
#include <memory>
#include <string>
#include <vector>
// project
#include "armor_solver/armor_solver.hpp"
#include "armor_solver/armor_tracker.hpp"
#include "armor_solver/outpost_solver.hpp"
#include "rm_interfaces/msg/armors.hpp"
#include "rm_interfaces/msg/measurement.hpp"
#include "rm_interfaces/msg/target.hpp"
// Outpost debug messages
#include "rm_interfaces/msg/outpost_target.hpp"
#include "rm_interfaces/msg/outpost_measurement.hpp"
#include "rm_interfaces/srv/set_mode.hpp"
#include "rm_utils/heartbeat.hpp"
#include "rm_utils/pkaLoggerCenter.hpp"

namespace pka::auto_aim {
using tf2_filter = tf2_ros::MessageFilter<rm_interfaces::msg::Armors>;
class ArmorSolverNode : public rclcpp::Node {
public:
  explicit ArmorSolverNode(const rclcpp::NodeOptions &options);

private:
  void armorsCallback(const rm_interfaces::msg::Armors::SharedPtr armors_ptr);

  void initMarkers() noexcept;

  void publishMarkers(const rm_interfaces::msg::Target &target_msg,
                      const rm_interfaces::msg::GimbalCmd &gimbal_cmd) noexcept;

  void setModeCallback(const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
                       std::shared_ptr<rm_interfaces::srv::SetMode::Response> response);

  bool debug_mode_;

  // 分组调试开关
  bool debug_tracker_;
  bool debug_ekf_;
  bool debug_solver_;
  bool debug_filter_;
  bool debug_outpost_;

  // 最大跟踪距离（按目标类型分流；工程兵在回调中直接剔除不参与瞄准）
  double max_armor_distance_;
  double max_outpost_armor_distance_;
  double max_base_armor_distance_;

  // 前哨站参数（id、plate_pitch）
  OutpostParams outpost_params_;

  // Heartbeat
  HeartBeatPublisher::SharedPtr heartbeat_;

  rclcpp::Time last_time_;
  double dt_;

  // -------------------------------------------------------
  // 地面兵种 EKF 过程噪声参数
  // -------------------------------------------------------
  double s2qx_, s2qy_, s2qz_, s2qyaw_, s2qr_, s2qd_zc_;
  double r_x_, r_y_, r_z_, r_yaw_;

  // -------------------------------------------------------
  // 前哨站 EKF 过程噪声参数（与地面兵种分开调节）
  // 前哨站使用与地面兵种相同的 10-D EKF，但通过 u_q/u_r lambda 内的
  // is_outpost 判断自动切换到前哨站专用噪声参数。
  // -------------------------------------------------------
  double outpost_s2qx_, outpost_s2qy_, outpost_s2qz_, outpost_s2qyaw_, outpost_s2qr_, outpost_s2qd_zc_;
  double outpost_r_x_, outpost_r_y_, outpost_r_z_, outpost_r_yaw_;

  double lost_time_thres_;
  int    outpost_tracking_thres_{0};
  double outpost_lost_time_thres_{0.0};

  std::unique_ptr<Tracker> tracker_;
  std::unique_ptr<Solver>  solver_;

  // ── 前哨站单板四元 EKF（mode=SINGLE，实现于 outpost_solver）────────────────
  OutpostSinglePlateRuntime outpost_single_rt_{};
  rm_interfaces::msg::GimbalCmd outpost_single_cmd_;
  bool outpost_single_cmd_valid_{false};
  // Last measurement (saved from armorsCallback) — passed into Solver::solve
  rm_interfaces::msg::Measurement last_measure_msg_;
  // Time of last single-mode valid measurement (for SINGLE timeout)
  rclcpp::Time last_outpost_single_time_;

  // Subscriber with tf2 message_filter
  std::string target_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  message_filters::Subscriber<rm_interfaces::msg::Armors> armors_sub_;
  rm_interfaces::msg::Target armor_target_;
  std::shared_ptr<tf2_filter> tf2_filter_;

  // Publishers
  rclcpp::Publisher<rm_interfaces::msg::Measurement>::SharedPtr measure_pub_;
  rclcpp::Publisher<rm_interfaces::msg::Target>::SharedPtr      target_pub_;
  rclcpp::Publisher<rm_interfaces::msg::GimbalCmd>::SharedPtr   gimbal_pub_;
  // Debug publishers for outpost-specific topics
  rclcpp::Publisher<rm_interfaces::msg::OutpostMeasurement>::SharedPtr outpost_measure_pub_;
  rclcpp::Publisher<rm_interfaces::msg::OutpostTarget>::SharedPtr outpost_target_pub_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  void timerCallback();

  bool enable_;
  rclcpp::Service<rm_interfaces::srv::SetMode>::SharedPtr set_mode_srv_;

  // Visualization markers
  visualization_msgs::msg::Marker position_marker_;
  visualization_msgs::msg::Marker linear_v_marker_;
  visualization_msgs::msg::Marker angular_v_marker_;
  visualization_msgs::msg::Marker trajectory_marker_;
  visualization_msgs::msg::Marker armors_marker_;
  visualization_msgs::msg::Marker selection_marker_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

}  // namespace pka::auto_aim

#endif  // ARMOR_SOLVER_SOLVER_NODE_HPP_
