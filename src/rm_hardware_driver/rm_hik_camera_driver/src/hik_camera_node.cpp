#include "rm_hik_camera_driver/hik_camera_node.hpp"

#include <chrono>
#include <thread>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <filesystem>

namespace pka::hik_camera
{

// ============================================================================
// Constructor
// ============================================================================
HikCameraNode::HikCameraNode(const rclcpp::NodeOptions & options)
  : Node("camera_driver", options)
{
  PKA_INFO("rm_hik_camera_node", "Starting HikCameraNode!");

  // ---- 枚举并打开设备 ----
  MV_CC_DEVICE_INFO_LIST device_list;
  nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  PKA_INFO("rm_hik_camera_node", "Found camera count = {}", device_list.nDeviceNum);

  while (device_list.nDeviceNum == 0 && rclcpp::ok())
  {
    PKA_ERROR("rm_hik_camera_node", "No camera found!");
    PKA_INFO("rm_hik_camera_node", "Enum state: [{}]", nRet);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  }

  MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[0]);
  MV_CC_OpenDevice(camera_handle_);

  // ---- 获取图像基本信息 ----
  MV_CC_GetImageInfo(camera_handle_, &img_info_);
  image_msg_.data.reserve(img_info_.nHeightMax * img_info_.nWidthMax * 3);

  // ---- 初始化像素格式转换参数 ----
  convert_param_.nWidth         = img_info_.nWidthValue;
  convert_param_.nHeight        = img_info_.nHeightValue;
  convert_param_.enDstPixelType = PixelType_Gvsp_RGB8_Packed;

  // ---- 发布者 ----
  bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", true);
  auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
  camera_pub_ = image_transport::create_camera_publisher(this, "image_raw", qos);

  // ---- Heartbeat ----
  heartbeat_ = HeartBeatPublisher::create(this);

  // ---- 声明参数（含录制参数）----
  declareParameters();

  // ---- 开始采集 ----
  MV_CC_StartGrabbing(camera_handle_);

  // ---- 加载相机标定信息 ----
  camera_name_ = this->declare_parameter("camera_name", "narrow_stereo");
  camera_info_manager_ =
    std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
  auto camera_info_url = this->declare_parameter(
    "camera_info_url", "package://rm_bringup/config/hik_camera_info.yaml");
  if (camera_info_manager_->validateURL(camera_info_url))
  {
    camera_info_manager_->loadCameraInfo(camera_info_url);
    camera_info_msg_ = camera_info_manager_->getCameraInfo();
  }
  else
  {
    PKA_WARN("rm_hik_camera_node", "Invalid camera info URL: {}", camera_info_url.c_str());
  }

  // ---- 参数动态回调 ----
  params_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&HikCameraNode::parametersCallback, this, std::placeholders::_1));

  // ---- 启动采集线程 ----
  stop_capture_thread_ = false;
  capture_thread_ = std::thread(&HikCameraNode::captureThreadFunc, this);
}

// ============================================================================
// Destructor
// ============================================================================
HikCameraNode::~HikCameraNode()
{
  PKA_INFO("rm_hik_camera_node", "HikCameraNode destructor called!");

  // 通知采集线程退出
  stop_capture_thread_ = true;

  if (capture_thread_.joinable())
  {
    PKA_INFO("rm_hik_camera_node", "Waiting for capture thread to finish...");
    capture_thread_.join();
    PKA_INFO("rm_hik_camera_node", "Capture thread finished");
  }

  // 确保录像文件已落盘
  stopRecording();

  // 释放相机资源
  if (camera_handle_)
  {
    PKA_INFO("rm_hik_camera_node", "Releasing camera resources...");
    MV_CC_StopGrabbing(camera_handle_);
    MV_CC_CloseDevice(camera_handle_);
    MV_CC_DestroyHandle(&camera_handle_);
    camera_handle_ = nullptr;
  }

  PKA_INFO("rm_hik_camera_node", "HikCameraNode destroyed!");
}

// ============================================================================
// declareParameters
// ============================================================================
void HikCameraNode::declareParameters()
{
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  MVCC_FLOATVALUE f_value;
  param_desc.integer_range.resize(1);
  param_desc.integer_range[0].step = 1;

  // ---- 曝光时间 ----
  param_desc.description = "Exposure time in microseconds";
  MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &f_value);
  param_desc.integer_range[0].from_value = f_value.fMin;
  param_desc.integer_range[0].to_value   = f_value.fMax;
  double exposure_time = this->declare_parameter("exposure_time", 5000, param_desc);
  MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time);
  PKA_INFO("rm_hik_camera_node", "Exposure time: {}", exposure_time);

  // ---- 增益 ----
  param_desc.description = "Gain";
  MV_CC_GetFloatValue(camera_handle_, "Gain", &f_value);
  param_desc.integer_range[0].from_value = f_value.fMin;
  param_desc.integer_range[0].to_value   = f_value.fMax;
  double gain = this->declare_parameter("gain", f_value.fCurValue, param_desc);
  MV_CC_SetFloatValue(camera_handle_, "Gain", gain);
  PKA_INFO("rm_hik_camera_node", "Gain: {}", gain);

  // ---- 录制开关 ----
  bool record_enable = this->declare_parameter("record_enable", false);
  recording_enabled_ = record_enable;

  // ---- 保存目录：默认存放到用户主目录下的 recordings/ ----
  // std::filesystem::path::home_directory() 在 C++17 不可用，
  // 使用 HOME 环境变量（Ubuntu 下始终有效）
  std::string home_dir = "/home";
  const char * env_home = std::getenv("HOME");
  if (env_home != nullptr)
  {
    home_dir = std::string(env_home);
  }
  std::string default_output_dir = home_dir + "/recordings";

  record_output_dir_ = this->declare_parameter("record_output_dir", default_output_dir);

  // ---- 文件名前缀 ----
  record_filename_prefix_ = this->declare_parameter("record_filename_prefix",
                                                     std::string("Vision_raw"));

  // ---- 自动分段时长（秒），0 = 不分段 ----
  int segment_s = this->declare_parameter("record_segment_duration_s", 300); // 默认 5 分钟
  record_segment_duration_s_ = segment_s;

  PKA_INFO("rm_hik_camera_node", "Recording enabled      : {}", record_enable);
  PKA_INFO("rm_hik_camera_node", "Recording output dir   : {}", record_output_dir_);
  PKA_INFO("rm_hik_camera_node", "Recording filename pfx : {}", record_filename_prefix_);
  PKA_INFO("rm_hik_camera_node", "Recording segment (s)  : {}", segment_s);
}

// ============================================================================
// parametersCallback
// ============================================================================
rcl_interfaces::msg::SetParametersResult HikCameraNode::parametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & param : parameters)
  {
    if (param.get_name() == "exposure_time")
    {
      int status = MV_CC_SetFloatValue(camera_handle_, "ExposureTime",
                                       static_cast<float>(param.as_int()));
      if (MV_OK != status)
      {
        result.successful = false;
        result.reason = "Failed to set exposure time, status = " + std::to_string(status);
      }
    }
    else if (param.get_name() == "gain")
    {
      int status = MV_CC_SetFloatValue(camera_handle_, "Gain",
                                       static_cast<float>(param.as_double()));
      if (MV_OK != status)
      {
        result.successful = false;
        result.reason = "Failed to set gain, status = " + std::to_string(status);
      }
    }
    else if (param.get_name() == "record_enable")
    {
      bool enable = param.as_bool();
      if (enable && !recording_enabled_)
      {
        PKA_INFO("rm_hik_camera_node", "Recording enabled via parameter.");
        {
          // 清空时间戳缓存，重新估算帧率后再开启 VideoWriter
          std::lock_guard<std::mutex> lock(video_writer_mutex_);
          host_timestamps_us_.clear();
          record_width_  = 0;
          record_height_ = 0;
        }
        recording_enabled_ = true;
      }
      else if (!enable && recording_enabled_)
      {
        PKA_INFO("rm_hik_camera_node", "Recording disabled via parameter.");
        recording_enabled_ = false;
        stopRecording();
      }
    }
    else if (param.get_name() == "record_output_dir")
    {
      record_output_dir_ = param.as_string();
    }
    else if (param.get_name() == "record_filename_prefix")
    {
      record_filename_prefix_ = param.as_string();
    }
    else if (param.get_name() == "record_segment_duration_s")
    {
      record_segment_duration_s_ = static_cast<int>(param.as_int());
      PKA_INFO("rm_hik_camera_node", "Segment duration updated to {} s",
               record_segment_duration_s_.load());
    }
    else
    {
      result.successful = false;
      result.reason = "Unknown parameter: " + param.get_name();
    }
  }

  return result;
}

// ============================================================================
// Recording helpers
// ============================================================================

double HikCameraNode::queryCameraFps()
{
  // 通过 MV_CC_GetFrameRate 读取相机 SDK 报告的当前帧率
  MVCC_FLOATVALUE fps_value{};
  int ret = MV_CC_GetFrameRate(camera_handle_, &fps_value);
  if (MV_OK == ret && fps_value.fCurValue > 0.0f)
  {
    PKA_INFO("rm_hik_camera_node",
             "SDK reported frame rate: {:.2f} fps (min={:.2f}, max={:.2f})",
             fps_value.fCurValue, fps_value.fMin, fps_value.fMax);
    return static_cast<double>(fps_value.fCurValue);
  }
  PKA_WARN("rm_hik_camera_node",
           "MV_CC_GetFrameRate failed (ret={}), falling back to 30 fps", ret);
  return 30.0;
}

std::string HikCameraNode::generateOutputPath() const
{
  // 确保输出目录存在
  std::filesystem::create_directories(record_output_dir_);

  // 文件名：<prefix>_YYYYMMDD_HHMMSS.mp4
  auto now   = std::chrono::system_clock::now();
  std::time_t t = std::chrono::system_clock::to_time_t(now);
  std::tm tm_info{};
  localtime_r(&t, &tm_info);

  std::ostringstream oss;
  oss << record_output_dir_ << "/"
      << record_filename_prefix_ << "_"
      << std::put_time(&tm_info, "%Y%m%d_%H%M%S")
      << ".mp4";
  return oss.str();
}

bool HikCameraNode::startRecording(int width, int height, double fps)
{
  std::lock_guard<std::mutex> lock(video_writer_mutex_);

  if (video_writer_.isOpened())
  {
    return true;  // 已在录制中，不重复打开
  }

  std::string path = generateOutputPath();

  // mp4v 编码 + MP4 容器（需 OpenCV 带 FFmpeg 支持）
  // 如果系统 OpenCV 支持 avc1/H.264，可将 fourcc 改为 ('a','v','c','1')
  int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
  video_writer_.open(path, fourcc, fps, cv::Size(width, height), true);

  if (!video_writer_.isOpened())
  {
    PKA_ERROR("rm_hik_camera_node",
              "Failed to open VideoWriter: {} ({}x{} @ {:.1f}fps)", path, width, height, fps);
    return false;
  }

  record_width_  = width;
  record_height_ = height;
  record_fps_    = fps;
  segment_start_tp_ = std::chrono::steady_clock::now();

  PKA_INFO("rm_hik_camera_node",
           "Recording started -> {} ({}x{} @ {:.2f} fps)", path, width, height, fps);
  return true;
}

void HikCameraNode::stopRecording()
{
  std::lock_guard<std::mutex> lock(video_writer_mutex_);
  if (video_writer_.isOpened())
  {
    video_writer_.release();
    PKA_INFO("rm_hik_camera_node", "Recording stopped and file saved.");
  }
}

// ============================================================================
// Capture thread
// ============================================================================
void HikCameraNode::captureThreadFunc()
{
  MV_FRAME_OUT out_frame;
  memset(&out_frame, 0, sizeof(MV_FRAME_OUT));

  PKA_INFO("rm_hik_camera_node", "Capture thread started.");

  image_msg_.header.frame_id = "camera_optical_frame";
  image_msg_.encoding        = "rgb8";

  while (rclcpp::ok() && !stop_capture_thread_)
  {
    nRet = MV_CC_GetImageBuffer(camera_handle_, &out_frame, 2000);

    if (MV_OK != nRet)
    {
      PKA_WARN("rm_hik_camera_node", "Get buffer failed! nRet: [{}]", nRet);
      MV_CC_StopGrabbing(camera_handle_);
      MV_CC_StartGrabbing(camera_handle_);
      fail_conut_++;

      if (fail_conut_ > 1000000)
      {
        stopRecording();  // 先保存已录内容
        PKA_FATAL("rm_hik_camera_node", "Camera failed!");
        rclcpp::shutdown();
      }
      continue;
    }

    // -----------------------------------------------------------------------
    // 1. 像素格式转换（Bayer/YUV → RGB8）
    // -----------------------------------------------------------------------
    const int frame_w = static_cast<int>(out_frame.stFrameInfo.nWidth);
    const int frame_h = static_cast<int>(out_frame.stFrameInfo.nHeight);

    // 确保目标缓冲区足够大（resize 不频繁，仅在尺寸变化时触发）
    image_msg_.data.resize(frame_w * frame_h * 3);

    convert_param_.pDstBuffer      = image_msg_.data.data();
    convert_param_.nDstBufferSize  = static_cast<unsigned int>(image_msg_.data.size());
    convert_param_.pSrcData        = out_frame.pBufAddr;
    convert_param_.nSrcDataLen     = out_frame.stFrameInfo.nFrameLen;
    convert_param_.enSrcPixelType  = out_frame.stFrameInfo.enPixelType;
    convert_param_.nWidth          = static_cast<unsigned int>(frame_w);
    convert_param_.nHeight         = static_cast<unsigned int>(frame_h);

    MV_CC_ConvertPixelType(camera_handle_, &convert_param_);

    // -----------------------------------------------------------------------
    // 2. 填充并发布 ROS 图像消息
    // -----------------------------------------------------------------------
    image_msg_.header.stamp = this->now();
    image_msg_.height = static_cast<uint32_t>(frame_h);
    image_msg_.width  = static_cast<uint32_t>(frame_w);
    image_msg_.step   = static_cast<uint32_t>(frame_w * 3);

    camera_info_msg_.header = image_msg_.header;
    camera_pub_.publish(image_msg_, camera_info_msg_);

    // -----------------------------------------------------------------------
    // 3. 内录逻辑
    // -----------------------------------------------------------------------
    if (recording_enabled_)
    {
      // --- 3a. 用 nHostTimeStamp（µs，主机侧打戳，与实际到达时刻对应）
      //         维护滑动窗口，估算当前真实帧率 ---
      const int64_t host_ts_us = out_frame.stFrameInfo.nHostTimeStamp;
      host_timestamps_us_.push_back(host_ts_us);
      if (static_cast<int>(host_timestamps_us_.size()) > kFpsWindow)
      {
        host_timestamps_us_.pop_front();
      }

      // --- 3b. 帧率稳定后，按需初始化 VideoWriter ---
      if (!video_writer_.isOpened())
      {
        // 优先用 SDK API 读取帧率（更权威）；
        // 再用时间窗口做兜底验证
        double sdk_fps = queryCameraFps();

        // 用 nHostTimeStamp 窗口做简单验证（窗口内数据够用时）
        if (static_cast<int>(host_timestamps_us_.size()) >= kFpsWindow)
        {
          double ts_span_s = static_cast<double>(
            host_timestamps_us_.back() - host_timestamps_us_.front()) * 1e-6;
          if (ts_span_s > 0.0)
          {
            double measured_fps =
              static_cast<double>(host_timestamps_us_.size() - 1) / ts_span_s;
            PKA_INFO("rm_hik_camera_node",
                     "Measured fps from host timestamps: {:.2f}", measured_fps);

            // 如果 SDK 值与实测相差超过 20%，以实测为准
            double diff_ratio = std::abs(measured_fps - sdk_fps) / sdk_fps;
            if (diff_ratio > 0.20)
            {
              PKA_WARN("rm_hik_camera_node",
                       "SDK fps ({:.2f}) deviates from measured ({:.2f}), using measured.",
                       sdk_fps, measured_fps);
              sdk_fps = measured_fps;
            }
          }
        }

        // 开启一个新的分段文件
        startRecording(frame_w, frame_h, sdk_fps);
      }

      // --- 3c. 写帧 ---
      if (video_writer_.isOpened())
      {
        // 分辨率变更时关闭旧文件、重新开始新分段
        if (frame_w != record_width_ || frame_h != record_height_)
        {
          PKA_WARN("rm_hik_camera_node",
                   "Resolution changed ({}x{} -> {}x{}), restarting recorder.",
                   record_width_, record_height_, frame_w, frame_h);
          stopRecording();
          host_timestamps_us_.clear();
          // 下一帧循环会重新触发 startRecording
        }
        else
        {
          // RGB → BGR（VideoWriter 内部是 BGR）
          cv::Mat rgb_mat(frame_h, frame_w, CV_8UC3, image_msg_.data.data());
          cv::Mat bgr_mat;
          cv::cvtColor(rgb_mat, bgr_mat, cv::COLOR_RGB2BGR);

          {
            std::lock_guard<std::mutex> lock(video_writer_mutex_);
            if (video_writer_.isOpened())
            {
              video_writer_.write(bgr_mat);
            }
          }

          // --- 3d. 自动分段 ---
          int seg_s = record_segment_duration_s_.load();
          if (seg_s > 0)
          {
            auto elapsed = std::chrono::steady_clock::now() - segment_start_tp_;
            if (elapsed >= std::chrono::seconds(seg_s))
            {
              PKA_INFO("rm_hik_camera_node",
                       "Segment duration ({} s) reached, starting new segment.", seg_s);
              // 关闭当前文件
              stopRecording();
              host_timestamps_us_.clear();
              // 立即以当前已知帧率开启下一段，无需重新等待估算窗口
              startRecording(frame_w, frame_h, record_fps_);
            }
          }
        }
      }
    }
    else
    {
      // 录制关闭：清空时间戳缓存，方便下次重新估算
      if (!host_timestamps_us_.empty())
      {
        host_timestamps_us_.clear();
      }
    }

    MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
    fail_conut_ = 0;
  }

  PKA_INFO("rm_hik_camera_node", "Capture thread exiting...");
}

}  // namespace pka::hik_camera

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pka::hik_camera::HikCameraNode)