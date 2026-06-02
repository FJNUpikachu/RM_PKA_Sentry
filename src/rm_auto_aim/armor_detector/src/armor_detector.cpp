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

#include "armor_detector/armor_detector.hpp"
#include <fmt/core.h>
// std
#include <algorithm>
#include <cmath>
#include <execution>
#include <vector>
// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
// project
#include "armor_detector/types.hpp"
#include "rm_utils/common.hpp"

namespace pka::auto_aim {
Detector::Detector(const int &bin_thres,
                   const EnemyColor &color,
                   const LightParams &l,
                   const ArmorParams &a)
: binary_thres(bin_thres), detect_color(color), light_params(l), armor_params(a) {}

// 识别函数
std::vector<Armor> Detector::detect(const cv::Mat &input_img) noexcept 
{
  // 1. Preprocess the image
  //预处理返回二值化图
  binary_img = preprocessImage(input_img);
  // 2. Find lights
  lights_ = findLights(input_img, binary_img);
  // 3. Match lights to armors
  // 将灯条匹配成装甲板
  armors_ = matchLights(lights_);
  
  if (!armors_.empty() && classifier != nullptr) 
  {
    // Parallel processing
    // 并行处理

    //std::execution::par 指定了并行执行策略，意味着每个装甲板对象的处理可以在不同的线程中并行执行，从而提高性能。
    std::for_each(
      std::execution::par, armors_.begin(), armors_.end(), [this, &input_img](Armor &armor) 
      {
        // 4. Extract the number image
        // 提取数字图像
        armor.number_img = classifier->extractNumber(input_img, armor);
        // 5. Do classification
        // 做分类
        classifier->classify(input_img, armor);
        //提高角点精度
        // 6. Correct the corners of the armor
        if (this->pca_ != nullptr) {
          for (auto& armor : this->armors_) {
            this->pca_->solvePCA(binary_img, armor);
          }
        }
      });

    // 7. Erase the armors with ignore classes
    // 过滤不符合的装甲板
    classifier->eraseIgnoreClasses(armors_);
  }

  return armors_;
}

// 预处理
cv::Mat Detector::preprocessImage(const cv::Mat &rgb_img) noexcept 
{
  //将rgb_img转换成灰度图gray_img_
  cv::cvtColor(rgb_img, gray_img_, cv::COLOR_RGB2GRAY);

  cv::Mat binary_img;
  //二值化 binary_thres为阈值
  cv::threshold(gray_img_, binary_img, binary_thres, 255, cv::THRESH_BINARY);

  return binary_img;
}

// 寻找灯条
std::vector<Light> Detector::findLights(const cv::Mat &rgb_img,
                                        const cv::Mat &binary_img) noexcept 
{
  using std::vector;
  // 定义存储轮廓的二维动态容器；vector<cv::Point>每个轮廓的点；外层vector存储多个轮廓的
  vector<vector<cv::Point>> contours;
  // 用于存储轮廓的层级信息
  /*
  每个 cv::Vec4i 存储轮廓之间的层级关系，具体来说，它包含以下四个值：
  [0]：下一个轮廓的索引（next）。
  [1]：前一个轮廓的索引（prev）。
  [2]：父轮廓的索引（parent）。
  [3]：第一个子轮廓的索引（child）。
  这个信息在多层次轮廓结构中（例如，包含嵌套轮廓的情况）非常有用。
  */
  vector<cv::Vec4i> hierarchy;

  //cv::RETR_EXTERNAL只返回外部轮廓；cv::CHAIN_APPROX_NONE保留轮廓的每一个点
  /*
  ._.
  | |
  | |
  | |
  ._.

  */
  cv::findContours(binary_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

  // 灯条容器
  vector<Light> lights;
  // 先将灯条信息清空
  debug_lights.data.clear();

  // 遍历所有轮廓信息
  for (const auto &contour : contours) 
  {
    // 当轮廓点少于6时就过滤掉
    if (contour.size() < 6) continue;
    
    //计算每个灯条的数据，存储在light中
    auto light = Light(contour);

    // 判断是否为灯条
    if (isLight(light)) 
    {
      int sum_r = 0, sum_b = 0;
      // 遍历每个轮廓点
      for (const auto &point : contour) 
      {
        sum_r += rgb_img.at<cv::Vec3b>(point.y, point.x)[0];
        sum_b += rgb_img.at<cv::Vec3b>(point.y, point.x)[2];
      }
      // 判断是否满足颜色偏差阈值
      if (std::abs(sum_r - sum_b) / static_cast<int>(contour.size()) >light_params.color_diff_thresh) 
      {
        light.color = sum_r > sum_b ? EnemyColor::RED : EnemyColor::BLUE;
      }
      //在容器尾部添加
      lights.emplace_back(light);
    }
  }
  std::sort(lights.begin(), lights.end(), [](const Light &l1, const Light &l2) 
  {
    // 按灯条的中心的x坐标由小到大升序排列
    return l1.center.x < l2.center.x;
  });
  return lights;
}

bool Detector::isLight(const Light &light) noexcept 
{
  // The ratio of light (short side / long side)
  // 灯条的比例（短边/长边）
  float ratio = light.width / light.length;
  // 判断灯条比例是否满足条件
  bool ratio_ok = light_params.min_ratio < ratio && ratio < light_params.max_ratio;
  // 灯条和水平线之间的角度是否满足条件
  bool angle_ok = light.tilt_angle < light_params.max_angle;

  // 在上述两个条件都成立的时候is_light=true，并返回该值
  bool is_light = ratio_ok && angle_ok;

  // Fill in debug information
  // 填充调试信息
  rm_interfaces::msg::DebugLight light_data;
  //灯条中心
  light_data.center_x = light.center.x;
  // 灯条比例
  light_data.ratio = ratio;
  // 灯条与水平线夹角
  light_data.angle = light.tilt_angle;
  // 是否为灯条
  light_data.is_light = is_light;
  // 向尾部添加元素
  this->debug_lights.data.emplace_back(light_data);

  return is_light;
}

// 将灯条匹配成装甲板
std::vector<Armor> Detector::matchLights(const std::vector<Light> &lights) noexcept 
{
  // 初始化装甲板容器
  std::vector<Armor> armors;
  // 清空装甲板容器信息
  this->debug_armors.data.clear();
  // Loop all the pairing of lights
  // 循环每一对灯条
  for (auto light_1 = lights.begin(); light_1 != lights.end(); light_1++) 
  {
    // 若不是要识别的颜色则跳出本次循环
    if (light_1->color != detect_color) continue;
    //max_large_center_distance：大装甲板最大中心距离长宽比
    // max_iter_width：计算出两根灯条可以形成一个装甲板的最大宽度
    double max_iter_width = light_1->length * armor_params.max_large_center_distance;

    for (auto light_2 = light_1 + 1; light_2 != lights.end(); light_2++) 
    {
      // 若不是要识别的颜色则跳出本次循环
      if (light_2->color != detect_color) continue;
      // 检查 2 个灯条形成的 boundingRect（最小外接矩形） 中是否有其他灯条
      if (containLight(light_1 - lights.begin(), light_2 - lights.begin(), lights)) 
      {
        continue;
      }
      // 检查宽度是否在最大宽度之内
      if (light_2->center.x - light_1->center.x > max_iter_width)
      {
        break;
      } 
      auto type = isArmor(*light_1, *light_2);
      if (type != ArmorType::INVALID) 
      {
        auto armor = Armor(*light_1, *light_2);
        armor.type = type;
        // 将装甲板信息添加到armors
        armors.emplace_back(armor);
      }
    }
  }
  return armors;
}

// Check if there is another light in the boundingRect formed by the 2 lights
// 检查 2 个灯条形成的 boundingRect（最小外接矩形） 中是否有其他灯条
bool Detector::containLight(const int i, const int j, const std::vector<Light> &lights) noexcept 
{
  const Light &light_1 = lights.at(i), light_2 = lights.at(j);
  auto points = std::vector<cv::Point2f>{light_1.top, light_1.bottom, light_2.top, light_2.bottom};
  auto bounding_rect = cv::boundingRect(points);
  double avg_length = (light_1.length + light_2.length) / 2.0;
  double avg_width = (light_1.width + light_2.width) / 2.0;
  // Only check lights in between
  // 仅检查两者之间的灯条
  for (int k = i + 1; k < j; k++) 
  {
    const Light &test_light = lights.at(k);

    // 防止数字干扰
    if (test_light.width > 2 * avg_width) 
    {
      continue;
    }
    // 防止红点准星或弹丸干扰
    if (test_light.length < 0.5 * avg_length) 
    {
      continue;
    }

    // 检查是否包含这些元素
    if (bounding_rect.contains(test_light.top) || bounding_rect.contains(test_light.bottom) ||
        bounding_rect.contains(test_light.center)) 
    {
      return true;
    }
  }
  return false;
}

ArmorType Detector::isArmor(const Light &light_1, const Light &light_2) noexcept 
{
  // Ratio of the length of 2 lights (short side / long side)
  // 两根灯条的长度比（短边/长边）
  float light_length_ratio = light_1.length < light_2.length ? light_1.length / light_2.length
                                                             : light_2.length / light_1.length;

  //判断是否满足条件                                                           
  bool light_ratio_ok = light_length_ratio > armor_params.min_light_ratio;

  // Distance between the center of 2 lights (unit : light length)
  // 两根灯条中心的距离

  // 平均灯长
  float avg_light_length = (light_1.length + light_2.length) / 2;
  // 中心距离/灯条平均长度
  float center_distance = cv::norm(light_1.center - light_2.center) / avg_light_length;
  bool center_distance_ok = (armor_params.min_small_center_distance <= center_distance &&
                             center_distance < armor_params.max_small_center_distance) ||
                            (armor_params.min_large_center_distance <= center_distance &&
                             center_distance < armor_params.max_large_center_distance);

  // Angle of light center connection
  // 灯条中心连线和水平线所成角度
  cv::Point2f diff = light_1.center - light_2.center;
  // 弧度转角度
  float angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;
  // 判断角度是否满足范围
  bool angle_ok = angle < armor_params.max_angle;

  // 上述三个条件均成立时判断为一块装甲板
  bool is_armor = light_ratio_ok && center_distance_ok && angle_ok;

  // Judge armor type
  // 判断装甲板类型
  ArmorType type;
  if (is_armor) 
  {
    // 判断是否大于大装甲板的最小的 （中心距离/灯条平均长度） 阈值
    type = center_distance > armor_params.min_large_center_distance ? ArmorType::LARGE
                                                                    : ArmorType::SMALL;
  } 
  // 均不满足为无效装甲板
  else
  {
    type = ArmorType::INVALID;
  }

  // Fill in debug information
  // 填充调试信息
  rm_interfaces::msg::DebugArmor armor_data;
  armor_data.type = armorTypeToString(type);
  // 装甲板的中心坐标x值
  armor_data.center_x = (light_1.center.x + light_2.center.x) / 2;
  armor_data.light_ratio = light_length_ratio;
  armor_data.center_distance = center_distance;
  armor_data.angle = angle;
  this->debug_armors.data.emplace_back(armor_data);

  return type;
}

// 获得所有标签图像
cv::Mat Detector::getAllNumbersImage() const noexcept 
{
  if (armors_.empty()) 
  {
    // 返回20x28 的单通道 8 位图像，默认值为0
    return cv::Mat(cv::Size(20, 28), CV_8UC1);
  } 
  else 
  {
    std::vector<cv::Mat> number_imgs;
    //预分配内存，确保number_imgs容器的容量足够存放所有装甲板的数字图像，避免在push_back时进行多次内存重新分配。
    number_imgs.reserve(armors_.size());
    for (auto &armor : armors_) 
    {
      // 将所有标签图像加入到number_imgs容器中
      number_imgs.emplace_back(armor.number_img);
    }
    cv::Mat all_num_img;
    // vconcat 函数将所有的数字图像按垂直方向（纵向）拼接成一个大的图像
    cv::vconcat(number_imgs, all_num_img);
    return all_num_img;
  }
}

// 框灯条和装甲板
void Detector::drawResults(cv::Mat &img) const noexcept 
{
  // Draw Lights
  // 画灯条
  // for (const auto &light : lights_) 
  //{
  //   auto line_color =
  //     light.color == EnemyColor::RED ? cv::Scalar(0, 255, 255) : cv::Scalar(255, 255, 0);
  //   // cv::ellipse(img, light, line_color, 2);
  //   cv::line(img, light.top, light.bottom, line_color, 1);
  //}

  // Draw armors
  // 画装甲板
  for (const auto &armor : armors_) 
  {
    cv::line(img, armor.left_light.top, armor.right_light.bottom, cv::Scalar(0, 255, 0), 1);
    cv::line(img, armor.left_light.bottom, armor.right_light.top, cv::Scalar(0, 255, 0), 1);

    cv::line(img, armor.left_light.top, armor.left_light.bottom, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);

    cv::line(img, armor.right_light.bottom, armor.right_light.top, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);

    cv::line(img, armor.left_light.top, armor.right_light.top, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);

    cv::line(img,armor.right_light.bottom,armor.left_light.bottom,cv::Scalar(0, 255, 0),1,cv::LINE_AA);
  }
  // Show numbers and confidence
  // 显示标签和置信度
  for (const auto &armor : armors_) 
  {
    std::string text = fmt::format("{} {}", armorTypeToString(armor.type), armor.classfication_result);
    // 在图像上绘制标签
    cv::putText(img, text, armor.left_light.top, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
  }

  // 终端显示最终图片
  // cv::imshow("result",img);
  // cv::waitKey(10);
}

}  // namespace pka::auto_aim
