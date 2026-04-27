#include "rm_hik_camera_driver/hik_camera_node.hpp"

#include <chrono>
#include <thread>

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
    else
    {
      result.successful = false;
      result.reason = "Unknown parameter: " + param.get_name();
    }
  }

  return result;
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
    nRet = MV_CC_GetImageBuffer(camera_handle_, &out_frame, 1000);

    if (MV_OK != nRet)
    {
      PKA_WARN("rm_hik_camera_node", "Get buffer failed! nRet: [{}]", nRet);
      MV_CC_StopGrabbing(camera_handle_);
      MV_CC_StartGrabbing(camera_handle_);
      fail_conut_++;

      if (fail_conut_ > 5)
      {
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

    MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
    fail_conut_ = 0;
  }

  PKA_INFO("rm_hik_camera_node", "Capture thread exiting...");
}

}  // namespace pka::hik_camera

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pka::hik_camera::HikCameraNode)