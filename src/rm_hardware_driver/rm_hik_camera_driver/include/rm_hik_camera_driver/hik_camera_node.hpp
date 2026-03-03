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
// OpenCV
#include <opencv2/opencv.hpp>
// STD
#include <thread>
#include <mutex>
#include <atomic>
#include <memory>
#include <string>
#include <chrono>
#include <deque>

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

    // ---- Recording helpers ----
    /// 通过 MV_CC_GetFrameRate 查询相机 SDK 报告的帧率
    double queryCameraFps();
    /// 启动一段新的录制，打开新的 MP4 文件
    bool startRecording(int width, int height, double fps);
    /// 停止并释放当前 VideoWriter，将文件落盘
    void stopRecording();
    /// 生成带时间戳的输出文件完整路径
    std::string generateOutputPath() const;

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

    // ---- Recording parameters (runtime-adjustable) ----
    std::atomic<bool> recording_enabled_{false};
    std::string       record_output_dir_;
    std::string       record_filename_prefix_;
    /// 每段录制最长时间（秒），0 表示不自动分段
    std::atomic<int>  record_segment_duration_s_{0};

    // ---- VideoWriter ----
    cv::VideoWriter video_writer_;
    std::mutex      video_writer_mutex_;

    /// 当前分段的起始时间点（用于自动分段判断）
    std::chrono::steady_clock::time_point segment_start_tp_;

    int    record_width_{0};
    int    record_height_{0};
    double record_fps_{30.0};

    // ---- 帧率估算（基于 SDK 帧信息的 nHostTimeStamp，单位 µs）----
    // 通过相邻帧时间差滑动平均，可自适应相机实际帧率和延迟
    static constexpr int kFpsWindow = 60;    ///< 滑动窗口帧数
    std::deque<int64_t>  host_timestamps_us_; ///< 最近 N 帧的 nHostTimeStamp
};

}  // namespace pka::hik_camera

#endif  // RM_HIK_CAMERA_DRIVER_HIK_CAMERA_NODE_HPP_