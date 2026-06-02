#include <armor_detector/armor_detector_node.hpp>

namespace pka::auto_aim {

ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions& options) : 
Node("armor_detector", options) {
    // register logger node
    PKA_INFO("armor_detector", "Starting ArmorDetectorNode!");

    // init detector
    this->detector_ = this->initDetector();

    // debug mode & optimize yaw
    this->debug_ = this->declare_parameter("debug", false);
    this->optimize_yaw = this->declare_parameter("optimize_yaw", true);
    this->search_range = this->declare_parameter("search_range", 80.0);
    this->yaw_offset_inclined = this->declare_parameter("yaw_offset_inclined", 5.0);

    // create armors publisher
    this->armors_pub_ = this->create_publisher<rm_interfaces::msg::Armors>(
        "armor_detector/armors", rclcpp::SensorDataQoS()
    );

    // TF 坐标系（与 serial_driver target_frame / URDF 对齐；yaml 中 odom_frame 常为 gimbal_odom）
    this->odom_frame_   = this->declare_parameter("odom_frame",   std::string("gimbal_odom"));
    this->gimbal_frame_ = this->declare_parameter("gimbal_frame", std::string("gimbal_link"));
    this->cam_frame_    = this->declare_parameter("cam_frame",    std::string("camera_optical_frame"));
    PKA_INFO("armor_detector",
      "TF frames: odom_frame='{}' gimbal_frame='{}' cam_frame='{}'",
      this->odom_frame_, this->gimbal_frame_, this->cam_frame_);

    // Visualization Marker Publisher
    // Detector识别的可视化发布
    // See http://wiki.ros.org/rviz/DisplayTypes/Marker
    this->armor_marker_.ns = "armors";
    this->armor_marker_.action = visualization_msgs::msg::Marker::ADD;
    this->armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
    this->armor_marker_.scale.x = 0.03;
    this->armor_marker_.scale.y = 0.15;
    this->armor_marker_.scale.z = 0.12;
    this->armor_marker_.color.a = 1.0;
    this->armor_marker_.color.r = 1.0;
    this->armor_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

    this->text_marker_.ns = "classification";
    this->text_marker_.action = visualization_msgs::msg::Marker::ADD;
    // 显示文本
    this->text_marker_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    this->text_marker_.scale.z = 0.1;
    this->text_marker_.color.a = 1.0;
    this->text_marker_.color.r = 1.0;
    this->text_marker_.color.g = 1.0;
    this->text_marker_.color.b = 1.0;
    this->text_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

    // 可视化发布者
    this->marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "armor_detector/marker", 10);

    // debug settings
    if (this->debug_) createDebugPublishers();
    this->debug_param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    this->debug_cb_handle_ = this->debug_param_sub_->add_parameter_callback(
        "debug", [this](const rclcpp::Parameter &p) {
            this->debug_ = p.as_bool();
            this->debug_ ? this->createDebugPublishers() : this->destroyDebugPublishers();
        });
    
    // camera information sub
    this->cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::CameraInfo::SharedPtr camera_info) {
            this->cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
            this->cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
            this->estimator_ = std::make_shared<ArmorPoseEstimator>(camera_info);
            this->estimator_->option.enable_optimize_yaw = this->optimize_yaw;
            this->estimator_->option.search_range = this->search_range;
            this->estimator_->option.inclined = this->yaw_offset_inclined;

            // get info for once
            this->cam_info_sub_.reset();
        }
    );

    // image subs and callback
    this->img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw", rclcpp::SensorDataQoS(),
        std::bind(&ArmorDetectorNode::imageCallback, this, std::placeholders::_1)
    );

    // tf2 management
    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    // 传递当前节点的基础接口和定时器接口来初始化定时器创建和管理
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(), this->get_node_timers_interface());
    // 由timer_interface来提供定时器来定期更新数据tf关系
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    // mode switch callback
    this->set_mode_srv_ = this->create_service<rm_interfaces::srv::SetMode>(
        "armor_detector/set_mode",
        std::bind(&ArmorDetectorNode::setModeCallback, this,
            std::placeholders::_1, std::placeholders::_2));

    this->heartbeat_ = HeartBeatPublisher::create(this);
}

void ArmorDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg) {
    // look up transform
    if (this->optimize_yaw) {
        // try to get the relations between camera and gimbal
        try {
            rclcpp::Time target_time = img_msg->header.stamp;
            auto R_camera2gimbal_tf = this->tf2_buffer_->lookupTransform(
                this->gimbal_frame_, this->cam_frame_, target_time, rclcpp::Duration::from_seconds(0.01)
            );
            auto msg_c2g_q = R_camera2gimbal_tf.transform.rotation;
            tf2::Quaternion tf_c2g_q;
            tf2::fromMsg(msg_c2g_q, tf_c2g_q);
            tf2::Matrix3x3 tf_c2g_mat(tf_c2g_q);

            this->R_camera2gimbal << tf_c2g_mat.getRow(0)[0], tf_c2g_mat.getRow(0)[1],
                tf_c2g_mat.getRow(0)[2], tf_c2g_mat.getRow(1)[0],
                tf_c2g_mat.getRow(1)[1], tf_c2g_mat.getRow(1)[2],
                tf_c2g_mat.getRow(2)[0], tf_c2g_mat.getRow(2)[1],
                tf_c2g_mat.getRow(2)[2];
            
            // reset look up timeout
            if (this->lookUp_error_count_ != 0) this->lookUp_error_count_ = 0;
        } catch (...) {
            PKA_ERROR("armor_detector",
              "lookUpTransform failed: {} -> {} (camera to gimbal)",
              this->gimbal_frame_, this->cam_frame_);
            if (this->lookUp_error_count_ > this->lookUp_thres_) rclcpp::shutdown();
            this->lookUp_error_count_++;
            return;
        }

        // try to get the relations between camera and gimbal
        try {
            rclcpp::Time target_time = img_msg->header.stamp;
            auto R_gimbal2odom_tf = this->tf2_buffer_->lookupTransform(
                this->odom_frame_, this->gimbal_frame_, target_time, rclcpp::Duration::from_seconds(0.05)
            );
            auto msg_g2o_q = R_gimbal2odom_tf.transform.rotation;
            
            tf2::Quaternion tf_g2o_q;
            tf2::fromMsg(msg_g2o_q, tf_g2o_q);
            tf2::Matrix3x3 tf_g2o_mat(tf_g2o_q);

            this->R_gimbal2odom << tf_g2o_mat.getRow(0)[0], tf_g2o_mat.getRow(0)[1],
                tf_g2o_mat.getRow(0)[2], tf_g2o_mat.getRow(1)[0],
                tf_g2o_mat.getRow(1)[1], tf_g2o_mat.getRow(1)[2],
                tf_g2o_mat.getRow(2)[0], tf_g2o_mat.getRow(2)[1],
                tf_g2o_mat.getRow(2)[2];
            
            // reset look up timeout
            if (this->lookUp_error_count_ != 0) this->lookUp_error_count_ = 0;
        } catch (...) {
            PKA_ERROR("armor_detector",
              "lookUpTransform failed: {} -> {} (gimbal to world/odom frame)",
              this->odom_frame_, this->gimbal_frame_);
            if (this->lookUp_error_count_ > this->lookUp_thres_) rclcpp::shutdown();
            this->lookUp_error_count_++;
            return;
        }

        // update
        this->estimator_->updateTransforms(this->R_camera2gimbal, this->R_gimbal2odom);
    }

    // get armors
    auto armors = detectArmors(img_msg);

    // init msg
    this->armors_msg_.header = img_msg->header;
    this->armors_msg_.armors.clear();

    // pnp
    if (this->estimator_ != nullptr) {
        this->armors_msg_.armors = this->estimator_->doPoseExtract(armors);
    } else PKA_ERROR("armor_detector", "Estimator hasn't initialized! PnP Failed!");

    // pub markers
    if (debug_) {
        this->marker_array_.markers.clear();
        this->armor_marker_.id = 0;
        this->text_marker_.id = 0;
        this->armor_marker_.header = this->text_marker_.header = this->armors_msg_.header;
        // Fill the markers
        for (const auto &armor : armors_msg_.armors) {
            this->armor_marker_.pose = armor.pose;
            this->armor_marker_.id++;
            this->text_marker_.pose.position = armor.pose.position;
            this->text_marker_.id++;
            this->text_marker_.pose.position.y -= 0.1;
            this->text_marker_.text = armor.number;
            this->marker_array_.markers.emplace_back(this->armor_marker_);
            this->marker_array_.markers.emplace_back(this->text_marker_);
        }
        this->publishMarkers();
    }

    // pub armors
    this->armors_pub_->publish(armors_msg_);
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
        //"mono8"：指定图像的编码格式。"mono8" 表示 8 位单通道图像，通常用于灰度图像。
        // CvImage函数用于将cv::Mat转换成ROS图像消息  
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
        // std::stringstream用于字符串的输入和输出
        std::stringstream latency_ss;
        latency_ss << "Latency: " << std::fixed << std::setprecision(2) << latency << "ms";
        auto latency_s = latency_ss.str();
        // 绘制延迟
        cv::putText(img, latency_s, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
        result_img_pub_.publish(cv_bridge::CvImage(img_msg->header, "rgb8", img).toImageMsg());
    }

    // 返回的是被成功识别的装甲板
    return armors;
}

std::shared_ptr<Detector> ArmorDetectorNode::initDetector() {
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    param_desc.integer_range.resize(1);
    param_desc.integer_range[0].step = 1;
    param_desc.integer_range[0].from_value = 0;
    param_desc.integer_range[0].to_value = 255;
    int binary_thres = declare_parameter("binary_thres", 160, param_desc);

    this->lookUp_thres_ = declare_parameter("lookUp_thres", 10);

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

    // 创建Detector类
    auto detector = std::make_shared<Detector>(binary_thres, EnemyColor::RED, l_params, a_params);

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
    if (use_pca) {
        // 创建LightCornerCorrector类
        detector->pca_ = std::make_unique<PCA>();
    }

    return detector;
}

void ArmorDetectorNode::createDebugPublishers() noexcept {
    // 灯条数据发布
    this->lights_data_pub_ = this->create_publisher<rm_interfaces::msg::DebugLights>(
        "armor_detector/debug_lights", 10);
    // 装甲板数据发布
    this->armors_data_pub_ = this->create_publisher<rm_interfaces::msg::DebugArmors>(
        "armor_detector/debug_armors", 10);

    this->declare_parameter("armor_detector.result_img.jpeg_quality", 50);
    this->declare_parameter("armor_detector.binary_img.jpeg_quality", 50);
    this->binary_img_pub_ =
        image_transport::create_publisher(this, "armor_detector/binary_img");
    this->number_img_pub_ =
        image_transport::create_publisher(this, "armor_detector/number_img");
    this->result_img_pub_ =
        image_transport::create_publisher(this, "armor_detector/result_img");
}

void ArmorDetectorNode::destroyDebugPublishers() noexcept {
    this->lights_data_pub_.reset();
    this->armors_data_pub_.reset();

    this->binary_img_pub_.shutdown();
    this->number_img_pub_.shutdown();
    this->result_img_pub_.shutdown();
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
    //如果为“UNKNOWN”
    if (mode_name == "UNKNOWN") 
    {
        PKA_ERROR("armor_detector", "Invalid mode: {}", request->mode);
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

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(pka::auto_aim::ArmorDetectorNode)