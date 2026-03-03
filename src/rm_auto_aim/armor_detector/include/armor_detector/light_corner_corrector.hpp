// Maintained by Shenglin Qin, Chengfu Zou
// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ARMOR_DETECTOR_LIGHT_CORNER_CORRECTOR_HPP_
#define ARMOR_DETECTOR_LIGHT_CORNER_CORRECTOR_HPP_

// opencv
#include <opencv2/opencv.hpp>
// project
#include "armor_detector/types.hpp"

namespace pka::auto_aim {

// 对称轴
struct SymmetryAxis {
  cv::Point2f centroid; //对称轴质心
  cv::Point2f direction; //对称轴方向
  float mean_val; // Mean brightness //平均亮度
};

// This class is used to improve the precision of the corner points of the light bar.
// First, the PCA algorithm is used to find the symmetry axis of the light bar,
// and then along the symmetry axis to find the corner points of the light bar based on the gradient of brightness.

// 此类用于提高灯条角点的精度。
// 首先，使用PCA算法找到灯条的对称轴，然后沿着对称轴根据亮度的梯度找到灯条的角点。

class LightCornerCorrector {
public:
  explicit LightCornerCorrector() noexcept {}

  // Correct the corners of the armor's lights
  // 修正装甲板灯条角点
  void correctCorners(Armor &armor, const cv::Mat &gray_img);

private:
  // Find the symmetry axis of the light
  // 寻找灯条的对称轴
  SymmetryAxis findSymmetryAxis(const cv::Mat &gray_img, const Light &light);

  // Find the corner of the light
  // 寻找灯条的角点
  cv::Point2f findCorner(const cv::Mat &gray_img,
                         const Light &light,
                         const SymmetryAxis &axis,
                         std::string order);
};

}  // namespace pka::auto_aim
#endif  // ARMOR_DETECTOR_LIGHT_CORNER_CORRECTOR_HPP_
