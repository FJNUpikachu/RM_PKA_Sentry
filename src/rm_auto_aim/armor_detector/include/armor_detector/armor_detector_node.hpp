#ifndef ARMOR_DETECTOR_DETECTOR_NODE_HPP_
#define ARMOR_DETECTOR_DETECTOR_NODE_HPP_

// std
#include <memory>
#include <string>
#include <vector>
// ros2
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
// tf2
#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/create_timer_ros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// msg
#include <rm_interfaces/msg/armors.hpp>
#include <rm_interfaces/msg/debug_armors.hpp>
#include <rm_interfaces/msg/debug_lights.hpp>
// srv
#include <rm_interfaces/srv/set_mode.hpp>
// files
#include <armor_detector/armor_detector.hpp>
#include <armor_detector/armor_pose_estimator.hpp>
#include <rm_utils/pkaLoggerCenter.hpp>
#include <rm_utils/url_resolver.hpp>
#include <rm_utils/heartbeat.hpp>

namespace pka::auto_aim {

class ArmorDetectorNode : public rclcpp::Node {
private:
    // detector
    std::shared_ptr<Detector> detector_;
    std::shared_ptr<Detector> initDetector();

    // estimator
    bool optimize_yaw;
    double search_range;
    double yaw_offset_inclined;
    Eigen::Matrix3d R_camera2gimbal;
    Eigen::Matrix3d R_gimbal2odom;
    Eigen::Vector3d t_camera2gimbal;
    std::shared_ptr<ArmorPoseEstimator> estimator_;

    // image sub
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);
    std::vector<Armor> detectArmors(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg);

    // armors publisher
    rm_interfaces::msg::Armors armors_msg_;
    rclcpp::Publisher<rm_interfaces::msg::Armors>::SharedPtr armors_pub_;

    // transform
    std::string odom_frame_;
    std::string gimbal_frame_;
    std::string cam_frame_;

    // visualization markers
    visualization_msgs::msg::Marker armor_marker_;
    visualization_msgs::msg::Marker text_marker_;
    visualization_msgs::msg::MarkerArray marker_array_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    void publishMarkers() noexcept;

    // camera
    cv::Point2f cam_center_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;

    // tf2 and look up timeout
    int lookUp_thres_;
    int lookUp_error_count_ = 0;
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    // debug
    bool debug_;
    std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;
    rclcpp::Publisher<rm_interfaces::msg::DebugLights>::SharedPtr lights_data_pub_;
    rclcpp::Publisher<rm_interfaces::msg::DebugArmors>::SharedPtr armors_data_pub_;
    image_transport::Publisher binary_img_pub_;
    image_transport::Publisher number_img_pub_;
    image_transport::Publisher result_img_pub_;
    void createDebugPublishers() noexcept;
    void destroyDebugPublishers() noexcept;

    // mode setting
    rclcpp::Service<rm_interfaces::srv::SetMode>::SharedPtr set_mode_srv_;
    void setModeCallback(
        const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
        std::shared_ptr<rm_interfaces::srv::SetMode::Response> response);

    // heartbeat
    HeartBeatPublisher::SharedPtr heartbeat_;

public:
    ArmorDetectorNode(const rclcpp::NodeOptions& options);
};

}

#endif