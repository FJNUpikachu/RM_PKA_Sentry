// Copyright Chen Jun 2023. Licensed under the MIT License.
//
// Additional modifications and features by Chengfu Zou, Labor. Licensed under
// Apache License 2.0.
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

#ifndef ARMOR_DETECTOR_DETECTOR_NODE_HPP_
#define ARMOR_DETECTOR_DETECTOR_NODE_HPP_

// ros2
#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
// std
#include <memory>
#include <string>
#include <vector>
// project
#include "armor_detector/armor_detector.hpp"
#include "armor_detector/armor_pose_estimator.hpp"
#include "armor_detector/number_classifier.hpp"
#include "rm_interfaces/msg/armors.hpp"
#include "rm_interfaces/srv/set_mode.hpp"
#include "rm_utils/heartbeat.hpp"
#include "rm_utils/pkaLoggerCenter.hpp"

namespace pka::auto_aim {

// Armor Detector Node
// Subscribe to the image topic, run the armor detection algorithm and publish
// the detected armors
class ArmorDetectorNode : public rclcpp::Node {
public:
  ArmorDetectorNode(const rclcpp::NodeOptions &options);

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);

  std::unique_ptr<Detector> initDetector();

  std::vector<Armor>
  detectArmors(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg);

  void createDebugPublishers() noexcept;
  void destroyDebugPublishers() noexcept;

  void publishMarkers() noexcept;

  void setModeCallback(
      const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
      std::shared_ptr<rm_interfaces::srv::SetMode::Response> response);

  // Dynamic Parameter
  rcl_interfaces::msg::SetParametersResult
  onSetParameters(std::vector<rclcpp::Parameter> parameters);
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr
      on_set_parameters_callback_handle_;

  // Heartbeat
  HeartBeatPublisher::SharedPtr heartbeat_;

  // Armor Detector
  // 装甲板识别
  std::unique_ptr<Detector> detector_;

  // Pose Solver
  // 姿态解算
  bool use_ba_;
  std::unique_ptr<ArmorPoseEstimator> armor_pose_estimator_;

  // Detected armors publisher
  rm_interfaces::msg::Armors armors_msg_;
  // armors的发布者
  rclcpp::Publisher<rm_interfaces::msg::Armors>::SharedPtr armors_pub_;

  // Visualization marker publisher
  visualization_msgs::msg::Marker armor_marker_;
  visualization_msgs::msg::Marker text_marker_;
  visualization_msgs::msg::MarkerArray marker_array_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_pub_;

  // Camera info part
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  cv::Point2f cam_center_;
  std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;

  // Image subscription
  // 图像信息的订阅者
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

  std::deque<Armor> tracked_armors_;

  // TF lookup（只做查询，不做广播；广播由 uart_node 负责）
  std::string odom_frame_;
  Eigen::Matrix3d imu_to_camera_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

  // TF 初始化标志：首次成功 lookupTransform 后置为 true。
  // 未初始化时 TF 查询失败直接跳过本帧（等待 uart_node 广播）；
  // 已初始化后查询失败则沿用上一帧旋转矩阵，避免偶发抖动丢帧。
  bool tf_initialized_{false};

  // Enable/Disable Armor Detector
  rclcpp::Service<rm_interfaces::srv::SetMode>::SharedPtr set_mode_srv_;

  // Debug information
  bool debug_;
  std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;
  rclcpp::Publisher<rm_interfaces::msg::DebugLights>::SharedPtr
      lights_data_pub_;
  rclcpp::Publisher<rm_interfaces::msg::DebugArmors>::SharedPtr
      armors_data_pub_;
  image_transport::Publisher binary_img_pub_;
  image_transport::Publisher number_img_pub_;
  image_transport::Publisher result_img_pub_;
};

} // namespace pka::auto_aim

#endif // ARMOR_DETECTOR_DETECTOR_NODE_HPP_