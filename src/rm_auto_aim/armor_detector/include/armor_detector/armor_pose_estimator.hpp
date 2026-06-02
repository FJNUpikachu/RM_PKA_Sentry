#ifndef ARMOR_DETECTOR_POSE_ESTIMATOR_HPP_
#define ARMOR_DETECTOR_POSE_ESTIMATOR_HPP_

// std
#include <unordered_map>
#include <numeric>
// eigen
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
// ros
#include <sensor_msgs/msg/camera_info.hpp>
// tf2
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
// files
#include <armor_detector/types.hpp>
#include <rm_interfaces/msg/armor.hpp>
#include <rm_utils/pkaLoggerCenter.hpp>

namespace pka::auto_aim {

class ArmorPoseEstimator {
private:
    // cam info
    cv::Mat camera_matrix_;
    Eigen::Matrix3d camera_matrix_eigen_;
    cv::Mat dist_coeffs_;

    // transforms
    Eigen::Matrix3d R_camera2gimbal_;
    Eigen::Matrix3d R_gimbal2odom_;

    // obj points map
    std::unordered_map<ArmorType, std::vector<cv::Point3f>> obj_points_map_;

    // calculate the dist to center
    float cal2CenterDist(const cv::Point2f& image_point);

    // tool func
    Eigen::Matrix3d calculateAsumMatrix(double yaw, double pitch);
    Eigen::Vector3d rotationMatrixToRPY(const Eigen::Matrix3d &R);
    Eigen::Quaterniond rpy2Quaternion(const Eigen::Vector3d& rpy);
    Eigen::Matrix3d rpy2RotationMatrix(const Eigen::Vector3d& rpy);
    double trisectionSearch(const Armor& armor, double l, double r, double eps);
    double shortest_angular_distance(double angle);

    // core
    void optimizeYaw(Armor& armor, double yaw_opt_inclined);
    double calculateReprojectionError(const Armor& armor, double yaw);
    double calculateReprojectionError(const std::vector<cv::Point2f> &image_points,
                                      const cv::Mat &rvec,
                                      const cv::Mat &tvec,
                                      const ArmorType &coord_frame_name) const noexcept;
    bool solvePnP(
        Armor& armor,
        const std::vector<cv::Point3f>& object_points,
        cv::Mat& rvec,
        cv::Mat& tvec,
        cv::SolvePnPMethod solutions = cv::SOLVEPNP_IPPE
    );
    double SJTU_cost(
        const std::vector<cv::Point2f> & cv_refs, const std::vector<cv::Point2f> & cv_pts,
        const double & inclined) const;

public:
    ArmorPoseEstimator() = delete;
    ArmorPoseEstimator(const sensor_msgs::msg::CameraInfo::SharedPtr camera_info);

    std::vector<rm_interfaces::msg::Armor> doPoseExtract(std::vector<Armor>& armors);

    void updateTransforms(const Eigen::Matrix3d& R_camera2gimbal, const Eigen::Matrix3d& R_gimbal2odom);

    // options
    struct Option {
        bool enable_optimize_yaw;
        double inclined;
        double search_range;
    };
    Option option;
};

}

#endif