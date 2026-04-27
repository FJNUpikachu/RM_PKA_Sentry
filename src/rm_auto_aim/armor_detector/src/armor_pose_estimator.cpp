#include <armor_detector/armor_pose_estimator.hpp>

namespace pka::auto_aim {

double SJTU_get_abs_angle(const Eigen::Vector2d & vec1, const Eigen::Vector2d & vec2)
{
    if (vec1.norm() == 0. || vec2.norm() == 0.) {
        return 0.;
    }
    return std::acos(vec1.dot(vec2) / (vec1.norm() * vec2.norm()));
}

template <typename T>
T SJTU_square(T const & a)
{
    return a * a;
};

ArmorPoseEstimator::ArmorPoseEstimator(const sensor_msgs::msg::CameraInfo::SharedPtr camera_info) {
    this->camera_matrix_ = cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_info->k.data())).clone();
    this->dist_coeffs_ = cv::Mat(1, 5, CV_64F, const_cast<double *>(camera_info->d.data())).clone();

    // fill the points
    this->obj_points_map_[ArmorType::SMALL] = Armor::buildObjectPoints<cv::Point3f>
        (SMALL_ARMOR_WIDTH, SMALL_ARMOR_HEIGHT);
    this->obj_points_map_[ArmorType::LARGE] = Armor::buildObjectPoints<cv::Point3f>
        (LARGE_ARMOR_WIDTH, LARGE_ARMOR_HEIGHT);

    // camera mat
    cv::cv2eigen(this->camera_matrix_, this->camera_matrix_eigen_);
}

void ArmorPoseEstimator::updateTransforms
(const Eigen::Matrix3d& R_camera2gimbal, const Eigen::Matrix3d& R_gimbal2odom) {
    this->R_camera2gimbal_ = R_camera2gimbal;
    this->R_gimbal2odom_ = R_gimbal2odom;
}

Eigen::Vector3d ArmorPoseEstimator::rotationMatrixToRPY(const Eigen::Matrix3d &R) {
    // Transform to camera frame
    Eigen::Quaterniond q(R);
    // Get armor yaw
    tf2::Quaternion tf_q(q.x(), q.y(), q.z(), q.w());
    Eigen::Vector3d rpy;
    tf2::Matrix3x3(tf_q).getRPY(rpy[0], rpy[1], rpy[2]);
    return rpy;
}

Eigen::Quaterniond ArmorPoseEstimator::rpy2Quaternion(const Eigen::Vector3d& rpy)
{
    return Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX());
}

Eigen::Matrix3d ArmorPoseEstimator::rpy2RotationMatrix(const Eigen::Vector3d& rpy) {
    return this->rpy2Quaternion(rpy).toRotationMatrix();
}

double ArmorPoseEstimator::shortest_angular_distance(double angle) {
    while (angle > CV_PI) angle -= 2 * CV_PI;
    while (angle <= -CV_PI) angle += 2 * CV_PI;
    return angle;
}

// 三分搜索
double ArmorPoseEstimator::trisectionSearch
(const Armor& armor, double l, double r, double eps) {
    while (r - l > eps) {
        double m1 = l + (r - l) / 3.0f;
        double m2 = r - (r - l) / 3.0f;

        double f1 = this->calculateReprojectionError(armor, m1);
        double f2 = this->calculateReprojectionError(armor, m2);

        if (f1 < f2) r = m2;
        else l = m1;
    }

    return (l + r) / 2.0f;
}

float ArmorPoseEstimator::cal2CenterDist(const cv::Point2f& image_point) {
    auto cam_center_x = this->camera_matrix_.at<double>(0, 2);
    auto cam_center_y = this->camera_matrix_.at<double>(1, 2);
    return cv::norm(image_point - cv::Point2f(cam_center_x, cam_center_y));
}

Eigen::Matrix3d ArmorPoseEstimator::calculateAsumMatrix(double yaw, double pitch) {
    auto sin_yaw = std::sin(yaw);
    auto cos_yaw = std::cos(yaw);
    auto sin_pitch = std::sin(pitch);
    auto cos_pitch = std::cos(pitch);

    const Eigen::Matrix3d result {
        {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
        {sin_yaw * cos_pitch, cos_yaw, sin_yaw * sin_pitch},
        {-sin_pitch, 0, cos_pitch}
    };

    return result;
}

// 上海交通大学代价函数
// 比对四点欧氏距离与角度，四边形边之间的关系，综合形成代价函数
double ArmorPoseEstimator::SJTU_cost(
  const std::vector<cv::Point2f> & cv_refs, const std::vector<cv::Point2f> & cv_pts,
  const double & inclined) const
{
    std::size_t size = cv_refs.size();
    std::vector<Eigen::Vector2d> refs;
    std::vector<Eigen::Vector2d> pts;
    for (std::size_t i = 0u; i < size; ++i) {
        refs.emplace_back(cv_refs[i].x, cv_refs[i].y);
        pts.emplace_back(cv_pts[i].x, cv_pts[i].y);
    }
    double cost = 0.;
    for (std::size_t i = 0u; i < size; ++i) {
        std::size_t p = (i + 1u) % size;
        // i - p 构成线段。过程：先移动起点，再补长度，再旋转
        Eigen::Vector2d ref_d = refs[p] - refs[i];  // 标准
        Eigen::Vector2d pt_d = pts[p] - pts[i];
        // 长度差代价 + 起点差代价(1 / 2)（0 度左右应该抛弃)
        double pixel_dis =  // dis 是指方差平面内到原点的距离
        (0.5 * ((refs[i] - pts[i]).norm() + (refs[p] - pts[p]).norm()) +
        std::fabs(ref_d.norm() - pt_d.norm())) /
        ref_d.norm();
        double angular_dis = ref_d.norm() * SJTU_get_abs_angle(ref_d, pt_d) / ref_d.norm();
        // 平方可能是为了配合 sin 和 cos
        // 弧度差代价（0 度左右占比应该大）
        double cost_i =
            SJTU_square(pixel_dis * std::sin(inclined)) +
            SJTU_square(angular_dis * std::cos(inclined)) * 2.0;  // DETECTOR_ERROR_PIXEL_BY_SLOPE
        // 重投影像素误差越大，越相信斜率
        cost += std::sqrt(cost_i);
    }
    return cost;
}

double ArmorPoseEstimator::calculateReprojectionError(
    const std::vector<cv::Point2f> &image_points,
    const cv::Mat &rvec,
    const cv::Mat &tvec,
    const ArmorType &coord_frame_name
) const noexcept {
    if (this->obj_points_map_.find(coord_frame_name) != this->obj_points_map_.end()) {
        const auto &object_points = this->obj_points_map_.at(coord_frame_name);
        std::vector<cv::Point2f> reprojected_points;
        cv::projectPoints(
            object_points, rvec, tvec, this->camera_matrix_, this->dist_coeffs_, reprojected_points);
        double error = 0;
        for (size_t i = 0; i < image_points.size(); ++i) {
            error += cv::norm(image_points[i] - reprojected_points[i]);
        }
        return error;
    } else {
        return 0;
    }
}

// 修改自原函数，从多解中选出基于6DoFs的最优位姿
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


std::vector<rm_interfaces::msg::Armor> 
ArmorPoseEstimator::doPoseExtract(std::vector<Armor>& armors) {
    std::vector<rm_interfaces::msg::Armor> armor_msgs;

    for (auto& armor : armors) {
        // init msg
        rm_interfaces::msg::Armor armor_msg;

        // obj points
        const auto& object_points = armor.type == ArmorType::SMALL ? 
            this->obj_points_map_[ArmorType::SMALL] : this->obj_points_map_[ArmorType::LARGE];
        
        // transform info
        cv::Mat rvec, tvec;

        // solve pnp
        auto ok = this->solvePnP(armor, object_points, rvec, tvec, cv::SOLVEPNP_IPPE);
        if (!ok) continue;

        // process t relationship
        Eigen::Vector3d t_armor2camera;
        cv::cv2eigen(tvec, t_armor2camera);
        armor.xyz_in_camera = t_armor2camera;

        // process r relationship
        cv::Mat rmat;
        Eigen::Matrix3d R_armor2camera;
        cv::Rodrigues(rvec, rmat);
        cv::cv2eigen(rmat, R_armor2camera);
        armor.rpy_in_camera = this->rotationMatrixToRPY(R_armor2camera);
        Eigen::Matrix3d R_armor2odom = this->R_gimbal2odom_ * this->R_camera2gimbal_ * R_armor2camera;
        armor.rpy_in_odom = this->rotationMatrixToRPY(R_armor2odom);

        // optimize yaw
        if (this->option.enable_optimize_yaw) {
            this->optimizeYaw(armor, this->option.inclined);
        }

        // rotation's q transformation
        auto armor_q = this->rpy2Quaternion(armor.rpy_in_camera);

        // msg build
        armor_msg.number = armor.number;
        armor_msg.type = armorTypeToString(armor.type);
        armor_msg.distance_to_image_center = this->cal2CenterDist(armor.center);
        armor_msg.pose.position.x = armor.xyz_in_camera[0];
        armor_msg.pose.position.y = armor.xyz_in_camera[1];
        armor_msg.pose.position.z = armor.xyz_in_camera[2];
        armor_msg.pose.orientation.w = armor_q.w();
        armor_msg.pose.orientation.x = armor_q.x();
        armor_msg.pose.orientation.y = armor_q.y();
        armor_msg.pose.orientation.z = armor_q.z();

        // push into vector
        armor_msgs.emplace_back(armor_msg);
    }

    return armor_msgs;
}

} // pka::auto_aim