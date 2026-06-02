#include <armor_detector/pca.hpp>

namespace pka::auto_aim {

void PCA::lightProc(Light& light, cv::Mat& binary_img) {
    // init
    cv::Point2f end1;
    cv::Point2f end2;

    // roi
    cv::Rect light_rect = light.boundingRect();
    int dx = cvRound(light_rect.width * 0.07);
    int dy = cvRound(light_rect.height * 0.07);
    light_rect.x -= dx;
    light_rect.y -= dy;
    light_rect.width += 2 * dx;
    light_rect.height += 2 * dy;
    light_rect &= cv::Rect(0, 0, binary_img.cols, binary_img.rows);
    if (light_rect.width <= 0 || light_rect.height <= 0) return;
    cv::Mat binary_roi = binary_img(light_rect);

    // find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary_roi, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    if (contours.empty()) return;

    // pca
    int idx = 0;
    double max_area = 0.0;
    for (int i = 0; i < static_cast<int>(contours.size()); ++i) {
        double area = std::abs(cv::contourArea(contours[i]));
        if (area > max_area) {
            max_area = area;
            idx = i;
        }
    }
    const std::vector<cv::Point>& target_contour = contours[idx];
    if (target_contour.size() < 2) return;
    cv::Mat data_pts(static_cast<int>(target_contour.size()), 2, CV_32F);
    for (int i = 0; i < static_cast<int>(target_contour.size()); ++i) {
        data_pts.at<float>(i, 0) = static_cast<float>(target_contour[i].x);
        data_pts.at<float>(i, 1) = static_cast<float>(target_contour[i].y);
    }
    cv::PCA pca(data_pts, cv::Mat(), cv::PCA::DATA_AS_ROW);

    // find end points
    cv::Point2f axis(pca.eigenvectors.at<float>(0, 0), pca.eigenvectors.at<float>(0, 1));
    float axis_norm = static_cast<float>(cv::norm(axis));
    if (axis_norm < 1e-6f) return;
    axis /= axis_norm;
    if (axis.y > 0) axis = -axis;
    struct ProjectedPoint {
        cv::Point2f pt;
        float proj;
    };
    std::vector<ProjectedPoint> projected_points;
    projected_points.reserve(target_contour.size());
    for (const auto& p : target_contour) {
        cv::Point2f pf(static_cast<float>(p.x), static_cast<float>(p.y));
        float proj = pf.dot(axis);
        projected_points.push_back({pf, proj});
    }
    std::sort(projected_points.begin(), projected_points.end(), [](const ProjectedPoint& a, const ProjectedPoint& b) {
        return a.proj < b.proj;
    });
    int n = static_cast<int>(projected_points.size());
    int k = std::max(1, static_cast<int>(std::ceil(n * 0.03)));
    for (int i = 0; i < k; ++i) end1 += projected_points[i].pt;
    for (int i = n - k; i < n; ++i) end2 += projected_points[i].pt;
    end1 /= static_cast<float>(k);
    end2 /= static_cast<float>(k);
    end1 += cv::Point2f(static_cast<float>(light_rect.x), static_cast<float>(light_rect.y));
    end2 += cv::Point2f(static_cast<float>(light_rect.x), static_cast<float>(light_rect.y));

    // fix
    light.top = end1;
    light.bottom = end2;
}

void PCA::solvePCA(cv::Mat& binary_img, Armor& armor) {
    this->lightProc(armor.left_light, binary_img);
    this->lightProc(armor.right_light, binary_img);
}

}