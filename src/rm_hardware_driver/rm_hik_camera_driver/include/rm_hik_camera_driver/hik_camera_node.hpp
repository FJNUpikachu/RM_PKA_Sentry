#ifndef RM_HIK_CAMERA_DRIVER_HIK_CAMERA_NODE_HPP_
#define RM_HIK_CAMERA_DRIVER_HIK_CAMERA_NODE_HPP_

#include "MvCameraControl.h"
// Project
#include "rm_utils/heartbeat.hpp"
#include "rm_utils/pkaLoggerCenter.hpp"
// ROS
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
// STD
#include <thread>
#include <atomic>
#include <memory>
#include <string>
#include <chrono>

namespace pka::hik_camera
{

class HikCameraNode : public rclcpp::Node
{
public:
    explicit HikCameraNode(const rclcpp::NodeOptions & options);
    ~HikCameraNode() override;

private:
    void declareParameters();

    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> & parameters);

    void captureThreadFunc();

    // ---- Image / camera ----
    sensor_msgs::msg::Image image_msg_;
    image_transport::CameraPublisher camera_pub_;

    int    nRet           = MV_OK;
    void * camera_handle_ = nullptr;
    MV_IMAGE_BASIC_INFO       img_info_;
    MV_CC_PIXEL_CONVERT_PARAM convert_param_;

    std::string camera_name_;
    std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
    sensor_msgs::msg::CameraInfo camera_info_msg_;

    // ---- Capture thread ----
    int               fail_conut_ = 0;
    std::thread       capture_thread_;
    std::atomic<bool> stop_capture_thread_{false};

    OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
    std::shared_ptr<HeartBeatPublisher> heartbeat_;
};

}  // namespace pka::hik_camera

#endif  // RM_HIK_CAMERA_DRIVER_HIK_CAMERA_NODE_HPP_