// Copyright Chen Jun 2023. Licensed under the MIT License.
//
// Additional modifications and features by Chengfu Zou, Labor. Licensed under Apache License 2.0.
//
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

#ifndef ARMOR_DETECTOR_DETECTOR_HPP_
#define ARMOR_DETECTOR_DETECTOR_HPP_

// std
#include <cmath>
#include <rm_utils/common.hpp>
#include <string>
#include <vector>
// third party 
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
// project
#include "armor_detector/light_corner_corrector.hpp"
#include "armor_detector/types.hpp"
#include "armor_detector/number_classifier.hpp"
#include "rm_interfaces/msg/debug_armors.hpp"
#include "rm_interfaces/msg/debug_lights.hpp"

namespace pka::auto_aim {
class Detector {
public:
  // 灯条参数结构体
  struct LightParams {
    // width / height
    double min_ratio;
    double max_ratio;
    // vertical angle
    double max_angle;
    // judge color
    int color_diff_thresh;
  };

  // 装甲板参数结构体
  struct ArmorParams {
    double min_light_ratio;
    // light pairs distance
    double min_small_center_distance;
    double max_small_center_distance;
    double min_large_center_distance;
    double max_large_center_distance;
    // horizontal angle
    double max_angle;
  };

  // 有参构造函数
  Detector(const int &bin_thres, const EnemyColor &color, const LightParams &l, const ArmorParams &a);

  // 识别函数
  std::vector<Armor> detect(const cv::Mat &input_img) noexcept;

  // 预处理函数
  cv::Mat preprocessImage(const cv::Mat &input_img) noexcept;

  //寻找灯条函数
  std::vector<Light> findLights(const cv::Mat &rgb_img,
                                const cv::Mat &binary_img) noexcept;
  std::vector<Armor> matchLights(const std::vector<Light> &lights) noexcept;

  // For debug usage
  cv::Mat getAllNumbersImage() const noexcept;
  void drawResults(cv::Mat &img) const noexcept;

  // Parameters
  int binary_thres;
  EnemyColor detect_color;
  LightParams light_params;
  ArmorParams armor_params;

  std::unique_ptr<NumberClassifier> classifier;
  std::unique_ptr<LightCornerCorrector> corner_corrector;

  // Debug msgs
  
  // 测试数据的消息类型
  // 包括测试灯条数据；测试装甲板数据
  cv::Mat binary_img;
  rm_interfaces::msg::DebugLights debug_lights;
  rm_interfaces::msg::DebugArmors debug_armors;

private:
  bool isLight(const Light &possible_light) noexcept;
  bool containLight(const int i,const int j,const std::vector<Light> &lights) noexcept;
  ArmorType isArmor(const Light &light_1, const Light &light_2) noexcept;

  // 灰度图
  cv::Mat gray_img_;


  // 灯条容器和装甲板容器
  std::vector<Light> lights_;
  std::vector<Armor> armors_;
};

} // namespace pka::auto_aim

#endif // ARMOR_DETECTOR_DETECTOR_HPP_
