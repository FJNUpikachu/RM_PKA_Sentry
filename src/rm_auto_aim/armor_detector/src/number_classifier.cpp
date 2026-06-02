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

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
// std
#include <algorithm>
#include <cstddef>
#include <execution>
#include <fstream>
#include <future>
#include <map>
#include <string>
#include <vector>
// 3rd party
#include <fmt/format.h>
// project
#include "armor_detector/number_classifier.hpp"
#include "armor_detector/types.hpp"

namespace pka::auto_aim {
NumberClassifier::NumberClassifier(const std::string &model_path,
                                   const std::string &label_path,
                                   const double thre,
                                   const std::vector<std::string> &ignore_classes)
: threshold(thre), ignore_classes_(ignore_classes) 
{
  net_ = cv::dnn::readNetFromONNX(model_path);
  std::ifstream label_file(label_path);
  std::string line;
  while (std::getline(label_file, line)) 
  {
    class_names_.push_back(line);
  }
}

// 提取数字
cv::Mat NumberClassifier::extractNumber(const cv::Mat &src, const Armor &armor) const noexcept 
{
  // Light length in image
  // 图像中的灯条长度
  static const int light_length = 12;
  // Image size after warp
  // 变换后的图像大小
  static const int warp_height = 28;
  static const int small_armor_width = 32;
  static const int large_armor_width = 54;
  // Number ROI size
  // ROI区域的大小
  static const cv::Size roi_size(20, 28);
  // 输入图像的大小
  static const cv::Size input_size(28, 28);

  // Warp perspective transform
  // 透视变换


  // 提取灯条的四个点
  cv::Point2f lights_vertices[4] = {
    armor.left_light.bottom, armor.left_light.top, armor.right_light.top, armor.right_light.bottom};

  // 
  const int top_light_y = (warp_height - light_length) / 2 - 1;
  const int bottom_light_y = top_light_y + light_length;
  const int warp_width = armor.type == ArmorType::SMALL ? small_armor_width : large_armor_width;
  cv::Point2f target_vertices[4] = 
  {
    cv::Point(0, bottom_light_y),
    cv::Point(0, top_light_y),
    cv::Point(warp_width - 1, top_light_y),
    cv::Point(warp_width - 1, bottom_light_y),
  };
  // 数字图像
  cv::Mat number_image;
  // 旋转矩阵
  // getPerspectiveTransform函数通过原图像lights_vertices和结果图像target_vertices的四个顶点坐标，从四对对应点计算透视变换。
  auto rotation_matrix = cv::getPerspectiveTransform(lights_vertices, target_vertices);
  // 透视变换
  // warp_width为变换的宽度，warp_height为变换的高度
  cv::warpPerspective(src, number_image, rotation_matrix, cv::Size(warp_width, warp_height));

  // Get ROI
  // 获取ROI区域
  number_image = number_image(cv::Rect(cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));

  // Binarize
  // 二值化
  cv::cvtColor(number_image, number_image, cv::COLOR_RGB2GRAY);
  // cv::THRESH_OTSU为自适应二值化
  cv::threshold(number_image, number_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
  // 对图像进行缩放
  cv::resize(number_image, number_image, input_size);
  return number_image;
}

// 分类
void NumberClassifier::classify(const cv::Mat &src, Armor &armor) noexcept 
{
  // Normalize
  // 正常化
  cv::Mat input = armor.number_img / 255.0;

  // Create blob from image
  // 将图像转换为适合神经网络的 blob 格式
  cv::Mat blob;
  cv::dnn::blobFromImage(input, blob);

  // Set the input blob for the neural network
  // 设置神经网络的输入blob
  mutex_.lock();
  net_.setInput(blob);

  // Forward pass the image blob through the model
  // 将图像blob向前传递到模型中
  cv::Mat outputs = net_.forward().clone();
  mutex_.unlock();

  // Decode the output
  // 解码输出

  // 置信度
  double confidence;
  cv::Point class_id_point;
  //minMaxLoc 是 OpenCV 中的一个函数，通常用于查找矩阵中的最小值和最大值的位置。
  //这里的 outputs.reshape(1, 1) 会将输出矩阵展平为一维，并找到其中的最大值（即模型预测的最大概率），它对应的类别就是最终的预测结果。
  minMaxLoc(outputs.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
  // 预测的标签
  int label_id = class_id_point.x;

  armor.confidence = confidence;
  // armor.number为装甲板的标签，即“1 2 3 4 5 outpost ......”等等
  armor.number = class_names_[label_id];

  // 分类结果
  armor.classfication_result = fmt::format("{}:{:.1f}%", armor.number, armor.confidence * 100.0);
}

void NumberClassifier::eraseIgnoreClasses(std::vector<Armor> &armors) noexcept 
{
  armors.erase(
    std::remove_if(armors.begin(),
                   armors.end(),
                   [this](const Armor &armor) 
                   {
                     // 置信度小于阈值时过滤
                     if (armor.confidence < threshold) 
                     {
                       return true;
                     }
                      // 为negative时过滤
                     for (const auto &ignore_class : ignore_classes_) 
                     {
                       if (armor.number == ignore_class) 
                       {
                         return true;
                       }
                     }
                      // 不匹配的装甲类型
                     bool mismatch_armor_type = false;
                     if (armor.type == ArmorType::LARGE) 
                     {
                      // 需测试一下！！！
                       mismatch_armor_type = armor.number == "outpost" || armor.number == "2" ||
                                             armor.number == "sentry";
                     } 
                     else if (armor.type == ArmorType::SMALL) 
                     {
                       mismatch_armor_type = armor.number == "1" || armor.number == "base";
                     }
                     return mismatch_armor_type;
                   }),
    armors.end());
}

}  // namespace pka::auto_aim
