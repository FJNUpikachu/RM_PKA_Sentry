# armor_detector

订阅相机参数及图像流进行装甲板的识别并解算三维位置，输出识别到的装甲板在输入frame下的三维位置 (一般是以相机光心为原点的相机坐标系)

## pka::ArmorDetectorNode

装甲板识别节点

### 发布话题 

*  `armor_detector/armors` (`rm_interfaces/msg/Armors`) - 识别到的装甲板信息
*  `armor_detector/debug_lights` (`rm_interfaces/msg/DebugLights`) - Debug灯条信息
*  `armor_detector/debug_armors` (`rm_interfaces/msg/DebugArmors`) - Debug装甲板信息
*  `armor_detector/result_img` (`sensor_msgs/msg/Image`) - 识别结果可视化图像
*  `armor_detector/binary_img` (`sensor_msgs/msg/Image`) - 二值化图像
*  `armor_detector/number_img` (`sensor_msgs/msg/Image`) - 数字识别roi

### 订阅话题

*  `image_raw` (`sensor_msgs/msg/Image`) - 相机图像
*  `camera_info` (`sensor_msgs/msg/CameraInfo`) - 相机参数

### 服务

*  `armor_detector/set_mode` (`rm_interfaces/srv/SetMode`) - 设置模式

### 参数 

* `debug` (`bool`, default: false) - 是否开启调试模式
* `classify_threshold` (`double`, default: 0.8) - 数字分类阈值
* `ignore_class` (`vector<string>`, default: ["negativie"]) - 跳过的类别
* `binary_thres` (`int`, default: 100) - 二值化阈值
* `light.min_ratio` (`double`, default: 0.08) - 灯条最小长宽比
* `light.max_ratio` (`double`, default: 0.4) - 灯条最大长宽比
* `light.max_angle` (`double`, default: 40) - 灯条最大倾斜角度
* `light.color_diff_thresh` (`int`, default: 25) - 灯条颜色差异阈值`
* `armor.min_light_ratio` (`double`, default: 0.6) - 装甲板最小长宽比
* `armor.min_small_center_distance` (`double`, default: 0.8) - 小装甲板最小中心距离长宽比
* `armor.max_small_center_distance` (`double`, default: 3.2) - 小装甲板最大中心距离长宽比
* `armor.min_large_center_distance` (`double`, default: 1.8) - 大装甲板最小中心距离长宽比
* `armor.max_large_center_distance` (`double`, default: 6.4) - 大装甲板最大中心距离长宽比
* `armor.max_angle` (`double`, default: 35.0) - 装甲板最大倾斜角度

### 位姿解算思路与调参说明

#### 参数

* `optimize_yaw` (`bool`, default: true) - 是否开启Yaw角优化
* `search_range` (`double`, default: 80.0) - 迭代的角度范围
* `yaw_offset_inclined` (`double`, default: 5.0) - 与原位姿误差惩罚代价（越大惩罚越重）

#### 思路

基于上海交通大学2023年于青工会上的分享，与实际在代码测试过程中的体验，发现由于图像像素的误差，在实际位姿为±10多度的时候会发生yaw的正负跳变，对于
后续通过建立运动方程解决小陀螺问题产生了巨大影响。
由于在RM中描述装甲板姿态的3个自由度(DoFs)有两个是固定的，分别为roll与pitch，故为解决这个问题，较好的方法是通过降低自由度，
在一定范围内改变在相机系下相同tvec的装甲板的yaw，并进行重投影，通过各种方式比对与实际检测结果的误差，选择误差最小的yaw值并更新。

重投影代价函数：

```cpp
// 基于欧氏距离的代价函数，采用平方和综合求误差，精度更高
double ArmorPoseEstimator::calculateReprojectionError(const Armor& armor, double yaw) {
    // reprojected points
    std::vector<cv::Point2f> reprojected_points(4);

    // set obj points
    const auto& obj_points = armor.type == ArmorType::SMALL ? 
        this->obj_points_map_[ArmorType::SMALL] : this->obj_points_map_[ArmorType::LARGE];

    // tvec
    const auto& t_armor2camera = armor.xyz_in_camera;
    cv::Vec3d tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

    // rvec
    double pitch = armor.number == "outpost" ? 
        -15.0 / 180.0 * CV_PI : 15.0 / 180.0 * CV_PI;
    auto R_armor2odom = this->calculateAsumMatrix(yaw, pitch);
    Eigen::Matrix3d R_armor2camera =
        this->R_camera2gimbal_.transpose() * this->R_gimbal2odom_.transpose() * R_armor2odom;
    cv::Mat rvec, rmat;
    cv::eigen2cv(R_armor2camera, rmat);
    cv::Rodrigues(rmat, rvec);

    // reproject
    cv::projectPoints(obj_points, rvec, tvec, this->camera_matrix_, this->dist_coeffs_, 
        reprojected_points);

    // calculate error
    double error = 0.0f;
    for (int i = 0; i < 4; i++) {
        auto d = armor.landmarks()[i] - reprojected_points[i];
        error += d.x * d.x + d.y * d.y;
    }

    // error = SJTU_cost(reprojected_points, armor.landmarks(), 35.0f / 180.0f * CV_PI);

    return error;
}
```

旧的代码中亦有类似尝试解决该问题的做法，其函数尝试通过cv::solvePnPGenerics()方法来获取在某种情况下PnP的多解，然后进行重投影与检测值比对，选择
重投影误差最小的值作为pnp的位姿。该做法解出的yaw大致方向是正确的，也就是说不会再出现正负yaw跳变的情况。

虽然解决了yaw角的正负跳变，但是考虑到基于6DoFs下的PnP解算，当装甲板偏离检测角度时，即当装甲板本身的yaw旋转到检测器能识别出该装甲板的极限yaw
时，此时解算出的yaw角的绝对值通常比实际值的绝对值要小，也就是说越远对角度的变化越不敏感。该点通过这一方法是无法解决的。

首先最常见的做法为，直接通过cv::solvePnP()函数解出当前最优的PnP，然后直接按照上述方式从云台yaw开始进行多次迭代，直到找出最优yaw，
但是该方法通常迭代次数过多，且同样会产生未知跳变，可能是迭代角度不合理导致。于是打算回归原来的思路，对旧的多解PnP函数进行重写。打算获取到
大致的位姿后再通过降自由度重投影精修获得较为准确的位姿。

重写后的solvePnP函数：

```cpp
bool ArmorPoseEstimator::solvePnP(
    Armor& armor,
    const std::vector<cv::Point3f>& object_points,
    cv::Mat& rvec,
    cv::Mat& tvec,
    cv::SolvePnPMethod solution
) {
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;

    int res = cv::solvePnPGeneric(
        object_points,
        armor.landmarks(),
        this->camera_matrix_,
        this->dist_coeffs_,
        rvecs,
        tvecs,
        false,
        solution
    );

    if (res <= 0 || rvecs.empty() || tvecs.empty()) {
        return false;
    }

    if (res == 1 || rvecs.size() == 1 || tvecs.size() == 1) {
        rvec = rvecs[0];
        tvec = tvecs[0];
        return true;
    }

    constexpr double PROJECT_ERR_THRES = 3.0;
    constexpr double ROLL_JUDGE = 10.0 * M_PI / 180.0;

    cv::Mat& rvec1 = rvecs[0];
    cv::Mat& tvec1 = tvecs[0];
    cv::Mat& rvec2 = rvecs[1];
    cv::Mat& tvec2 = tvecs[1];

    cv::Mat R1_cv, R2_cv;
    cv::Rodrigues(rvec1, R1_cv);
    cv::Rodrigues(rvec2, R2_cv);

    Eigen::Matrix3d R1, R2;
    cv::cv2eigen(R1_cv, R1);
    cv::cv2eigen(R2_cv, R2);

    auto rpy1 = this->rotationMatrixToRPY(this->R_camera2gimbal_ * R1);
    auto rpy2 = this->rotationMatrixToRPY(this->R_camera2gimbal_ * R2);

    auto armor_type = armor.type == ArmorType::SMALL ? ArmorType::SMALL : ArmorType::LARGE;
    double error1 = this->calculateReprojectionError(armor.landmarks(), rvec1, tvec1, armor_type);
    double error2 = this->calculateReprojectionError(armor.landmarks(), rvec2, tvec2, armor_type);

    bool roll_pass1 = std::abs(rpy1[0]) < ROLL_JUDGE;
    bool roll_pass2 = std::abs(rpy2[0]) < ROLL_JUDGE;

    if (!roll_pass1 && !roll_pass2) {
        return false;
    }
    if (roll_pass1 && !roll_pass2) {
        rvec = rvec1;
        tvec = tvec1;
        return true;
    }
    if (!roll_pass1 && roll_pass2) {
        rvec = rvec2;
        tvec = tvec2;
        return true;
    }

    if (error2 / error1 > PROJECT_ERR_THRES) {
        rvec = rvec1;
        tvec = tvec1;
        return true;
    }
    if (error1 / error2 > PROJECT_ERR_THRES) {
        rvec = rvec2;
        tvec = tvec2;
        return true;
    }

    double l_angle = std::atan2(armor.left_light.axis.y, armor.left_light.axis.x) * 180.0 / M_PI;
    double r_angle = std::atan2(armor.right_light.axis.y, armor.right_light.axis.x) * 180.0 / M_PI;
    double angle = (l_angle + r_angle) / 2.0 + 90.0;

    if ((angle > 0 && rpy1[2] > 0 && rpy2[2] < 0) ||
        (angle < 0 && rpy1[2] < 0 && rpy2[2] > 0)) {
        rvec = rvec2;
        tvec = tvec2;
    } else {
        rvec = rvec1;
        tvec = tvec1;
    }

    return true;
}
```

同样获取其最优值，以最优值的yaw为迭代范围的中心，准备进行迭代
（能以原装甲板的yaw进行迭代的前提是能够确定yaw的正负号）。因此，该函数主要是对yaw的正负值
进行规范，确保其正负值合理。

于是，基于正负值合理的前提下，通过原始yaw的位姿，设置迭代初始值进行小范围的迭代精修。

同时。为防止因范围设置不合理而导致的其他问题，新增代价函数

```cpp
double total_error = error + yaw_opt_inclined * yaw_diff * yaw_diff;
```

将更新后的yaw与原来的yaw的差值的平方乘以一个权重值，用以限制不合理的yaw更新。其中权重越高，yaw限制得越死

最终yaw优化函数：

```cpp
// 降低自由度以pnp结果为基准优化yaw
void ArmorPoseEstimator::optimizeYaw(Armor& armor, double yaw_opt_inclined) {
    // 完全相信cv的多解pnp函数
    double yaw_init = armor.rpy_in_odom[2];

    double search_half = (this->option.search_range / 2.0f) * CV_PI / 180.0f;
    double step = 0.5f * CV_PI / 180.0;

    double min_error = std::numeric_limits<double>::max();
    double best_yaw = yaw_init;

    // 只做微调处理
    for (double yaw = yaw_init - search_half; yaw <= yaw_init + search_half; yaw += step) {
        double yaw_fixed = this->shortest_angular_distance(yaw);

        double error = this->calculateReprojectionError(armor, yaw_fixed);

        double yaw_diff = this->shortest_angular_distance(yaw_fixed - yaw_init);

        // 权重稳定误差
        double total_error = error + yaw_opt_inclined * yaw_diff * yaw_diff;

        if (total_error < min_error) {
            min_error = total_error;
            best_yaw = yaw_fixed;
        }
    }

    // PKA_INFO("armor_detector", "yaw_init={}, best_yaw={}, error={}", yaw_init, best_yaw, min_error);

    armor.rpy_in_odom[2] = best_yaw;
    Eigen::Matrix3d R_armor2camera_updated =
        this->R_camera2gimbal_.transpose() *
        this->R_gimbal2odom_.transpose() *
        this->rpy2RotationMatrix(armor.rpy_in_odom);
    armor.rpy_in_camera = this->rotationMatrixToRPY(R_armor2camera_updated);
}
```

总结：先获取大概位姿，限制yaw正负号，再小范围降自由度重投影，结合位姿差权重综合计算误差，对PnP出来的位姿精修，最后获得较为准确的位姿。