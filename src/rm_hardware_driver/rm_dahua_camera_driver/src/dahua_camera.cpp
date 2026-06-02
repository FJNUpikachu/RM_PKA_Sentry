#include "rm_dahua_camera_driver/dahua_camera.hpp"
// std
#include <bits/getopt_core.h>
#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <thread>

// Opencv
#include <opencv2/opencv.hpp>

// project
#include "rm_dahua_camera_driver/Camera_Control.h"
#include "rm_utils/pkaLoggerCenter.hpp"

// ros2
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <vector>
using namespace cv;
using namespace std;

namespace pka::dahua_camera {
// 相机节点构造函数
DahuaCameraNode::DahuaCameraNode(const rclcpp::NodeOptions &options)
    : Node("camera_driver", options) {
    PKA_INFO("rm_dahua_camera_driver", "Starting DahuaCameraNode!");

    // ROS2 参数限制
    rcl_interfaces::msg::ParameterDescriptor exposure_time_desc;
    exposure_time_desc.description = "Exposure time of the camera";
    exposure_time_desc.floating_point_range.resize(1);
    exposure_time_desc.floating_point_range[0].from_value = 1.0;
    exposure_time_desc.floating_point_range[0].to_value = 60000.0;
    exposure_time_ = this->declare_parameter("exposure_time", 3000.0);

    rcl_interfaces::msg::ParameterDescriptor brightness_desc;
    brightness_desc.description = "Brightness level";
    brightness_desc.integer_range.resize(1);
    brightness_desc.integer_range[0].from_value = 0;
    brightness_desc.integer_range[0].to_value = 100;
    brightness = this->declare_parameter("Brightness", 50);

    // 参数部分
    camera_name_ = this->declare_parameter("camera_name", "narrow_stereo");
    camera_info_url_ = this->declare_parameter(
        "camera_info_url", "package://rm_bringup/config/dahua_camera_info.yaml");
    frame_id_ = this->declare_parameter("camera_frame_id", "camera_optical_frame");
    // 是否开启自动白平衡
    auto_white_balance_ = this->declare_parameter<int>("auto_white_balance", 1);
    frame_rate_ = this->declare_parameter("frame_rate", 100.0);

    gain_ = this->declare_parameter("gain", 2.0);

    resolution_width_ = this->declare_parameter("resolution_width", 800);
    resolution_height_ = this->declare_parameter("resolution_height", 600);
    offset_x_ = this->declare_parameter("offsetX", 0);
    offset_y_ = this->declare_parameter("offsetY", 0);

    // 为存储原始图像数据申请空间
    image_msg_.header.frame_id = frame_id_;
    image_msg_.encoding = "rgb8";
    image_msg_.height = resolution_height_;
    image_msg_.width = resolution_width_;
    image_msg_.step = resolution_width_ * 3;
    image_msg_.data.resize(image_msg_.height * image_msg_.step);

    // 创建相机信息管理器
    camera_info_manager_ =
        std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
    // 加载相机信息
    if (camera_info_manager_->validateURL(camera_info_url_)) {
        camera_info_manager_->loadCameraInfo(camera_info_url_);
        camera_info_msg_ = camera_info_manager_->getCameraInfo();
    } else {
        // 如果读取URL失败则打印无效信息
        PKA_WARN("rm_dahua_camera_driver", "Invalid camera info URL: {}", camera_info_url_.c_str());
    }
    camera_info_msg_.header.frame_id = frame_id_;
    camera_info_msg_.header.stamp = this->now();

    //==================
    // qos设置
    bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", true);
    auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
    // 创建相机图片发布
    camera_pub_ = image_transport::create_camera_publisher(this, "image_raw", qos);
    //=================

    // Heartbeat 心跳节点创建
    heartbeat_ = HeartBeatPublisher::create(this);

    // Check if camera is alive every seconds
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),
                                     std::bind(&DahuaCameraNode::timerCallback, this));

    // Param set callback
    on_set_parameters_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&DahuaCameraNode::onSetParameters, this, std::placeholders::_1));

    PKA_INFO("rm_dahua_camera_driver", "DahuaCameraNode has been Start!");
}

// 相机节点析构函数
DahuaCameraNode::~DahuaCameraNode() {
    PKA_INFO("rm_dahua_camera_driver", "DahuaCameraNode destructor called!");

    // Stop threads
    if (work_thread_ && work_thread_->joinable()) {
        PKA_INFO("rm_dahua_camera_driver", "Waiting for capture thread to finish...");
        work_thread_->join();
        PKA_INFO("rm_dahua_camera_driver", "Capture thread finished");
    }

    close();
    PKA_INFO("rm_dahua_camera_driver", "Shutting down node!");
}

// 监测回调函数
void DahuaCameraNode::timerCallback() {
    while (!is_open_ && rclcpp::ok()) {
        bool is_open_success = open();
        if (!is_open_success) {
            PKA_ERROR("rm_dahua_camera_driver", "open camera failed!");
            close();
            openCamera_error++;
            if (openCamera_error > 5) {
                rclcpp::shutdown();
            }
            return;
        }
    }
    // Watch Dog
    double dt = (this->now() - rclcpp::Time(camera_info_msg_.header.stamp)).seconds();
    if (dt > 5.0) {
        PKA_WARN("rm_dahua_camera_driver", "Camera is not alive! lost frame for {:.2f} seconds", dt);
        close();
    }
}

// 打开相机
bool DahuaCameraNode::open() {
    PKA_INFO("rm_dahua_camera_driver", "Opening Dahua Camera Device!");
    if (is_open_) {
        PKA_WARN("rm_dahua_camera_driver", "Dahua Camera Device is already opened!");
        close();
    }
    if (!is_open_) {
        if (Check_connect()) {
            is_open_ = true;
        } else {
            is_open_ = false;
            return false;
        }
        
        camera.CameraChangeTrig(Camera_Control::ETrigType::trigContinous);

        if (camera.SetExposeTime(exposure_time_) &&
            camera.SetAdjustPlus(gain_) &&
            camera.setBrightness(brightness) &&
            camera.setROI(offset_x_, offset_y_, resolution_width_, resolution_height_) &&
            camera.autoBalance(auto_white_balance_) &&
            camera.setFrameRate(frame_rate_)
        ) {
            PKA_INFO("rm_dahua_camera_driver", "Camera parameters set successfully!");
        } else {
            PKA_WARN("rm_dahua_camera_driver", "Camera parameters set failed!");
            return false;
        }

        if (!camera.videoStart()) {
            PKA_WARN("rm_dahua_camera_driver", "videoStart failed!!");
            return false;
        }
        if (!camera.stopGrabbing()) {
            PKA_WARN("rm_dahua_camera_driver", "stopGrabbing failed!!");
            return false;
        }
        if (!camera.startGrabbing()) {
            PKA_WARN("rm_dahua_camera_driver", "startGrabbing failed!!");
            return false;
        }
        PKA_INFO("rm_dahua_camera_driver", "Dahua Camera Device Open Success!");
        work_thread_ = std::make_shared<std::thread>(std::bind(&DahuaCameraNode::CaptureImage, this));
    }
    return true;
}

bool DahuaCameraNode::close() {
    PKA_INFO("rm_dahua_camera_driver", "Closing Dahua Camera!");
    if (is_open_) {
        camera.stopGrabbing();
        camera.Disconnect_Camera();
        is_open_ = false;
        return true;
    }
    PKA_WARN("rm_dahua_camera_driver", "Dahua Camera Device is already closed or No Camera!");
    return false;
}

bool DahuaCameraNode::Check_connect() {
    int VideoCheck_LOSTUM = 1;
    int VideoOpen_LOSTUM = 1;
    while (1) {
        switch (connect_state) {
            case 0:
                if (camera.videoCheck()) {
                    connect_state = 1;
                    continue;
                }
                VideoCheck_LOSTUM++;
                if (VideoCheck_LOSTUM >= 3) {
                    PKA_WARN("rm_dahua_camera_driver", "No camera connection found!");
                    return false;
                }
                break;
            case 1:
                if (camera.Connect_Camera()) {
                    connect_state = 0;
                    return true;
                }
                VideoOpen_LOSTUM++;
                if (VideoOpen_LOSTUM >= 3) {
                    connect_state = 0;
                    PKA_WARN("rm_dahua_camera_driver", "The camera connection failed!");
                    return false;
                }
                break;
            default:
                PKA_ERROR("rm_dahua_camera_driver", "The initial value of the connect_state is incorrect!");
                return false;
        }
    }
}

void DahuaCameraNode::CaptureImage()
{
    while (rclcpp::ok()) {
        if (camera.getFrame(src)) {
            if (!src.empty()) {
                image_msg_.header.stamp = camera_info_msg_.header.stamp = this->now();
                image_msg_.is_bigendian = false;
                memcpy(&image_msg_.data[0], src.data, image_msg_.data.size());
                camera_pub_.publish(image_msg_, camera_info_msg_);

                empty_frame = 0;
            } else {
                PKA_ERROR("rm_dahua_camera_driver", "Read Frame is empty!");
                empty_frame++;
            }
        } else {
            PKA_WARN("rm_dahua_camera_driver", "GetFrame Failed!");
            empty_frame++;
        }
        if (empty_frame > 5) {
            PKA_ERROR("rm_dahua_camera_driver", "Camera failed!");
            rclcpp::shutdown();
        }
    }
}

// 参数设置回调
rcl_interfaces::msg::SetParametersResult DahuaCameraNode::onSetParameters(std::vector<rclcpp::Parameter> parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto &param : parameters) {
        if (param.get_name() == "exposure_time") {
            exposure_time_ = param.as_double();
            camera.SetExposeTime(exposure_time_);
            PKA_INFO("rm_dahua_camera_driver", "Set exposure_time: {:.3f}", exposure_time_);
        } else if (param.get_name() == "gain") {
            gain_ = param.as_double();
            camera.SetAdjustPlus(gain_);
            PKA_INFO("rm_dahua_camera_driver", "Set gain: {:.3f}", gain_);
        } else {
            result.successful = false;
            result.reason = "Unknown parameter: " + param.get_name();
        }
    }
    return result;
}

}  // namespace pka::dahua_camera

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pka::dahua_camera::DahuaCameraNode)