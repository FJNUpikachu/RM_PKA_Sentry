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

// std
#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <numeric>
#include <string>
#include <vector>
// ros2
#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>

#include <image_transport/image_transport.hpp>
#include <rclcpp/qos.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// third party
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
// project
#include "armor_detector/armor_detector_node.hpp"
#include "armor_detector/ba_solver.hpp"
#include "armor_detector/types.hpp"
#include "rm_utils/assert.hpp"
#include "rm_utils/common.hpp"
#include "rm_utils/pkaLoggerCenter.hpp"
#include "rm_utils/math/pnp_solver.hpp"
#include "rm_utils/math/utils.hpp"
#include "rm_utils/url_resolver.hpp"

namespace pka::auto_aim {
ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions &options)
    : Node("armor_detector", options) {
  PKA_INFO("armor_detector", "Starting ArmorDetectorNode!");
  // Detector
  detector_ = initDetector();

  // Tricks to make pose more accurate
  // 使姿势更准确的技巧
  use_ba_ = this->declare_parameter("use_ba", true);

  // Armors Publisher
  // Armors发布者
  armors_pub_ = this->create_publisher<rm_interfaces::msg::Armors>(
      "armor_detector/armors", rclcpp::SensorDataQoS());

  // Transform initialize
  // 转换初始化
  odom_frame_ = this->declare_parameter("target_frame", "odom");
  //imu到camera的转移矩阵
  imu_to_camera_ = Eigen::Matrix3d::Identity();

  // Visualization Marker Publisher
  // Detector识别的可视化发布
  // See http://wiki.ros.org/rviz/DisplayTypes/Marker
  armor_marker_.ns = "armors";
  armor_marker_.action = visualization_msgs::msg::Marker::ADD;
  armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
  armor_marker_.scale.x = 0.03;
  armor_marker_.scale.y = 0.15;
  armor_marker_.scale.z = 0.12;
  armor_marker_.color.a = 1.0;
  armor_marker_.color.r = 1.0;
  armor_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

  text_marker_.ns = "classification";
  text_marker_.action = visualization_msgs::msg::Marker::ADD;
  // 显示文本
  text_marker_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text_marker_.scale.z = 0.1;
  text_marker_.color.a = 1.0;
  text_marker_.color.r = 1.0;
  text_marker_.color.g = 1.0;
  text_marker_.color.b = 1.0;
  text_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

  // 可视化发布者
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "armor_detector/marker", 10);

  // Debug Publishers
  // 调试发布者
  debug_ = this->declare_parameter("debug", true);
  if (debug_) 
  {
    createDebugPublishers();
  }
  // Debug param change moniter
  // 调试参数更改监视器
  // 用来监听debug_参数是否发生变化
  debug_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  debug_cb_handle_ = debug_param_sub_->add_parameter_callback(
      "debug", [this](const rclcpp::Parameter &p) {
        debug_ = p.as_bool();
        debug_ ? createDebugPublishers() : destroyDebugPublishers();
      });

  // 相机消息订阅者
  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "camera_info", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::SharedPtr camera_info) 
      {
        // 相机光心
        cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
        // 将camera_info的内容复制到一个共享指针cam_info_中，保存相机信息，供后续使用。
        cam_info_ =
            std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
        // Setup armor pose solver
        // 设置armor姿态解算器
        armor_pose_estimator_ = std::make_unique<ArmorPoseEstimator>(cam_info_);
        // 设置是否开启BA解算器
        armor_pose_estimator_->enableBA(use_ba_);

        // 重置该智能指针
        cam_info_sub_.reset();
      });

  // 图像信息订阅者（订阅相机原始图像帧信息）
  // 订阅相机发布的图像，并进行识别即回调imageCallback()函数

  // 创建图像信息的订阅
  img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image_raw", rclcpp::SensorDataQoS(),
      std::bind(&ArmorDetectorNode::imageCallback, this,
                std::placeholders::_1));

  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  // 传递当前节点的基础接口和定时器接口来初始化定时器创建和管理
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
  
  // 由timer_interface来提供定时器来定期更新数据tf关系
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  // 设置识别模式服务 
  set_mode_srv_ = this->create_service<rm_interfaces::srv::SetMode>(
      "armor_detector/set_mode",
      std::bind(&ArmorDetectorNode::setModeCallback, this,
                std::placeholders::_1, std::placeholders::_2));

  heartbeat_ = HeartBeatPublisher::create(this);
}

void ArmorDetectorNode::imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr img_msg) 
{
  // Get the transform from odom to gimbal
  // 获取odom到gimbal的转换关系
  try {
    rclcpp::Time target_time = img_msg->header.stamp;

    // odom_to_gimbal
    // 获得img到odom的转换关系
    // 相机到odom
    auto odom_to_gimbal = tf2_buffer_->lookupTransform(
        odom_frame_, img_msg->header.frame_id, target_time,
        rclcpp::Duration::from_seconds(0.01));
    
    // 提取转换关系中的旋转关系
    auto msg_q = odom_to_gimbal.transform.rotation;
    tf2::Quaternion tf_q;
    //  将 ROS 消息类型的四元数
    tf2::fromMsg(msg_q, tf_q);
    // 获得旋转矩阵
    tf2::Matrix3x3 tf2_matrix = tf2::Matrix3x3(tf_q);

    
    imu_to_camera_ << tf2_matrix.getRow(0)[0], tf2_matrix.getRow(0)[1],
        tf2_matrix.getRow(0)[2], tf2_matrix.getRow(1)[0],
        tf2_matrix.getRow(1)[1], tf2_matrix.getRow(1)[2],
        tf2_matrix.getRow(2)[0], tf2_matrix.getRow(2)[1],
        tf2_matrix.getRow(2)[2];
  } catch (...) {
    PKA_ERROR("armor_detector", "Something Wrong when lookUpTransform");
    return;
  }

  // Detect armors
  // 识别armors
  auto armors = detectArmors(img_msg);

  // Init message
  armors_msg_.header = img_msg->header;
  armors_msg_.armors.clear();

  // Extract armor poses
  if (armor_pose_estimator_ != nullptr) {
    armors_msg_.armors =
        armor_pose_estimator_->extractArmorPoses(armors, imu_to_camera_);
  } else {
    PKA_WARN("armor_detector", "PnP Failed!");
  }

  // Publishing marker
  if (debug_) {
    marker_array_.markers.clear();
    armor_marker_.id = 0;
    text_marker_.id = 0;
    armor_marker_.header = text_marker_.header = armors_msg_.header;
    // Fill the markers
    for (const auto &armor : armors_msg_.armors) {
      armor_marker_.pose = armor.pose;
      armor_marker_.id++;
      text_marker_.pose.position = armor.pose.position;
      text_marker_.id++;
      text_marker_.pose.position.y -= 0.1;
      text_marker_.text = armor.number;
      marker_array_.markers.emplace_back(armor_marker_);
      marker_array_.markers.emplace_back(text_marker_);
    }
    publishMarkers();
  }

  // Publishing detected armors
  armors_pub_->publish(armors_msg_);
}

//初始化Detector
std::unique_ptr<Detector> ArmorDetectorNode::initDetector() 
{
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.integer_range.resize(1);
  param_desc.integer_range[0].step = 1;
  param_desc.integer_range[0].from_value = 0;
  param_desc.integer_range[0].to_value = 255;
  int binary_thres = declare_parameter("binary_thres", 160, param_desc);

  //灯条参数
  Detector::LightParams l_params = {
      .min_ratio = declare_parameter("light.min_ratio", 0.08),
      .max_ratio = declare_parameter("light.max_ratio", 0.4),
      .max_angle = declare_parameter("light.max_angle", 40.0),
      .color_diff_thresh =
          static_cast<int>(declare_parameter("light.color_diff_thresh", 25))};

  //装甲板参数
  Detector::ArmorParams a_params = {
      .min_light_ratio = declare_parameter("armor.min_light_ratio", 0.6),
      .min_small_center_distance =
          declare_parameter("armor.min_small_center_distance", 0.8),
      .max_small_center_distance =
          declare_parameter("armor.max_small_center_distance", 3.2),
      .min_large_center_distance =
          declare_parameter("armor.min_large_center_distance", 3.2),
      .max_large_center_distance =
          declare_parameter("armor.max_large_center_distance", 5.0),
      .max_angle = declare_parameter("armor.max_angle", 35.0)};

  // ── 初始识别颜色从参数读取 ────────────────────────────────────────────────
  // detect_color: 0=RED(default)  1=BLUE
  // 与 VisionMode / SetMode.srv 中 0=自瞄红 1=自瞄蓝 保持一致
  int detect_color_param = declare_parameter("detect_color", 0);
  EnemyColor initial_color = (detect_color_param == 1)
      ? EnemyColor::BLUE
      : EnemyColor::RED;
  PKA_INFO("armor_detector",
    "Initial detect_color={} ({})",
    detect_color_param, initial_color == EnemyColor::RED ? "RED" : "BLUE");

  // 创建Detector类（使用从参数读取的初始颜色，不再硬编码 RED）
  auto detector = std::make_unique<Detector>(binary_thres, initial_color, l_params, a_params);

  // 初始化分类器
  // Init classifier
  namespace fs = std::filesystem;
  fs::path model_path = utils::URLResolver::getResolvedPath(
      "package://armor_detector/model/lenet.onnx");
  fs::path label_path = utils::URLResolver::getResolvedPath(
      "package://armor_detector/model/label.txt");
  PKA_ASSERT_MSG(fs::exists(model_path) && fs::exists(label_path),
                 model_path.string() + " Not Found!");

  // 设置分类器置信度
  double threshold = this->declare_parameter("classifier_threshold", 0.7);

  std::vector<std::string> ignore_classes = this->declare_parameter(
      "ignore_classes", std::vector<std::string>{"negative"});

  // 创建NumberClassifier类
  detector->classifier = std::make_unique<NumberClassifier>(
      model_path, label_path, threshold, ignore_classes);

  // 初始化修正器
  // Init Corrector
  bool use_pca = this->declare_parameter("use_pca", true);
  if (use_pca) 
  {
    // 创建LightCornerCorrector类
    detector->corner_corrector = std::make_unique<LightCornerCorrector>();
  }

  // Set dynamic parameter callback
  // 设置动态参数回调
  on_set_parameters_callback_handle_ =
      this->add_on_set_parameters_callback(std::bind(
          &ArmorDetectorNode::onSetParameters, this, std::placeholders::_1));

  // 返回一个detector类
  return detector;
}

std::vector<Armor> ArmorDetectorNode::detectArmors(
    const sensor_msgs::msg::Image::ConstSharedPtr &img_msg) 
{
  // Convert ROS img to cv::Mat
  // 将ROS的img转换为cv::Mat，采用rgb8的形式转换
  auto img = cv_bridge::toCvShare(img_msg, "rgb8")->image;

  // 识别装甲板函数
  auto armors = detector_->detect(img);

  auto final_time = this->now();
  // 计算延迟
  auto latency = (final_time - img_msg->header.stamp).seconds() * 1000;

  // Publish debug info
  // 发布调试信息
  if (debug_) 
  {
    binary_img_pub_.publish(
        cv_bridge::CvImage(img_msg->header, "mono8", detector_->binary_img)
            .toImageMsg());

    // Sort lights and armors data by x coordinate
    // 按x坐标对灯条和装甲板进行排序
    std::sort(detector_->debug_lights.data.begin(),
              detector_->debug_lights.data.end(),
              [](const auto &l1, const auto &l2) {
                return l1.center_x < l2.center_x;
              });
    std::sort(detector_->debug_armors.data.begin(),
              detector_->debug_armors.data.end(),
              [](const auto &a1, const auto &a2) {
                return a1.center_x < a2.center_x;
              });

    // 调试信息发布
    lights_data_pub_->publish(detector_->debug_lights);
    armors_data_pub_->publish(detector_->debug_armors);

    if (!armors.empty()) 
    {
      // 获得所有标签图像的纵向合并图像
      auto all_num_img = detector_->getAllNumbersImage();
      number_img_pub_.publish(
          *cv_bridge::CvImage(img_msg->header, "mono8", all_num_img)
               .toImageMsg());
    }
    // 框出识别到的装甲板
    detector_->drawResults(img);

    // Draw camera center
    // 绘制相机中心
    cv::circle(img, cam_center_, 5, cv::Scalar(255, 0, 0), 2);
    // Draw latency
    // 绘制延迟
    std::stringstream latency_ss;
    latency_ss << "Latency: " << std::fixed << std::setprecision(2) << latency << "ms";
    auto latency_s = latency_ss.str();
    cv::putText(img, latency_s, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
    result_img_pub_.publish(cv_bridge::CvImage(img_msg->header, "rgb8", img).toImageMsg());
  }

  // 返回的是被成功识别的装甲板
  return armors;
}

rcl_interfaces::msg::SetParametersResult
// 设置参数
ArmorDetectorNode::onSetParameters(std::vector<rclcpp::Parameter> parameters) 
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto &param : parameters) 
  {
    if (param.get_name() == "binary_thres") 
    {
      detector_->binary_thres = param.as_int();
    }
    else if (param.get_name() == "classifier_threshold") 
    {
      detector_->classifier->threshold = param.as_double();
    } 
    else if (param.get_name() == "light.min_ratio") 
    {
      detector_->light_params.min_ratio = param.as_double();
    } 
    else if (param.get_name() == "light.max_ratio") 
    {
      detector_->light_params.max_ratio = param.as_double();
    } 
    else if (param.get_name() == "light.max_angle") 
    {
      detector_->light_params.max_angle = param.as_double();
    } 
    else if (param.get_name() == "light.color_diff_thresh") 
    {
      detector_->light_params.color_diff_thresh = param.as_int();
    } 
    else if (param.get_name() == "armor.min_light_ratio") 
    {
      detector_->armor_params.min_light_ratio = param.as_double();
    } 
    else if (param.get_name() == "armor.min_small_center_distance") 
    {
      detector_->armor_params.min_small_center_distance = param.as_double();
    } 
    else if (param.get_name() == "armor.max_small_center_distance") 
    {
      detector_->armor_params.max_small_center_distance = param.as_double();
    } 
    else if (param.get_name() == "armor.min_large_center_distance") 
    {
      detector_->armor_params.min_large_center_distance = param.as_double();
    } 
    else if (param.get_name() == "armor.max_large_center_distance") 
    {
      detector_->armor_params.max_large_center_distance = param.as_double();
    } 
    else if (param.get_name() == "armor.max_angle") 
    {
      detector_->armor_params.max_angle = param.as_double();
    }
  }
  return result;
}

// 创建调试发布者
void ArmorDetectorNode::createDebugPublishers() noexcept 
{
  // 灯条数据发布
  lights_data_pub_ = this->create_publisher<rm_interfaces::msg::DebugLights>(
      "armor_detector/debug_lights", 10);
  // 装甲板数据发布
  armors_data_pub_ = this->create_publisher<rm_interfaces::msg::DebugArmors>(
      "armor_detector/debug_armors", 10);
  
  this->declare_parameter("armor_detector.result_img.jpeg_quality", 50);
  this->declare_parameter("armor_detector.binary_img.jpeg_quality", 50);
  binary_img_pub_ =
      image_transport::create_publisher(this, "armor_detector/binary_img");
  number_img_pub_ =
      image_transport::create_publisher(this, "armor_detector/number_img");
  result_img_pub_ =
      image_transport::create_publisher(this, "armor_detector/result_img");
}

// 释放调试发布者
void ArmorDetectorNode::destroyDebugPublishers() noexcept 
{
  lights_data_pub_.reset();
  armors_data_pub_.reset();

  binary_img_pub_.shutdown();
  number_img_pub_.shutdown();
  result_img_pub_.shutdown();
}

void ArmorDetectorNode::publishMarkers() noexcept 
{
  using Marker = visualization_msgs::msg::Marker;
  armor_marker_.action =
      armors_msg_.armors.empty() ? Marker::DELETEALL : Marker::ADD;
  marker_array_.markers.emplace_back(armor_marker_);
  marker_pub_->publish(marker_array_);
}

void ArmorDetectorNode::setModeCallback(
    const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
    std::shared_ptr<rm_interfaces::srv::SetMode::Response> response) {
  response->success = true;
  response->message = "0";

  VisionMode mode = static_cast<VisionMode>(request->mode);
  // 将自瞄模式名称转换成字符串
  std::string mode_name = visionModeToString(mode);
  //如果为"UNKNOWN"
  if (mode_name == "UNKNOWN") 
  {
    PKA_ERROR("armor_detector", "Invalid mode: {}", request->mode);
    response->success = false;
    response->message = "Invalid mode";
    return;
  }

  // 创建图片信息的订阅者函数
  auto createImageSub = [this]() 
  {
    // 如果图像订阅为空就重新创建图像订阅
    if (img_sub_ == nullptr) 
    {
      img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
          "image_raw", rclcpp::SensorDataQoS(),
          std::bind(&ArmorDetectorNode::imageCallback, this,
                    std::placeholders::_1));
    }
  };

  switch (mode) 
  {
  case VisionMode::AUTO_AIM_RED: 
  {
    detector_->detect_color = EnemyColor::RED;
    createImageSub();
    break;
  }
  case VisionMode::AUTO_AIM_BLUE: 
  {
    detector_->detect_color = EnemyColor::BLUE;
    createImageSub();
    break;
  }
  default: 
  {
    img_sub_.reset();
  }
  }

  // 设置识别模式为mode_name
  PKA_WARN("armor_detector", "Set mode to {}", mode_name);
}

} // namespace pka::auto_aim

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pka::auto_aim::ArmorDetectorNode)