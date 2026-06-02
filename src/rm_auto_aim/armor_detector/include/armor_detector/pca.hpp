#ifndef ARMOR_DETECTOR_PCA_HPP_
#define ARMOR_DETECTOR_PCA_HPP_

#include <vector>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <armor_detector/types.hpp>

namespace pka::auto_aim {

class PCA {
private:
    void lightProc(Light& light, cv::Mat& binary_img);

    std::vector<std::vector<cv::Point>> contours;
    
public:
    PCA() = default;

    void solvePCA(cv::Mat& binary_img, Armor& armor);
};

}

#endif