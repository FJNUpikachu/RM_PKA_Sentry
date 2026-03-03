// Created by Chengfu Zou on 2023.10.26
// Maintained by Chengfu Zou
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

#ifndef ARMOR_DETECTOR_TYPES_HPP_
#define ARMOR_DETECTOR_TYPES_HPP_

// std
#include <algorithm>
#include <numeric>
#include <string>
// 3rd party
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <sophus/so3.hpp>
// project
#include "rm_utils/assert.hpp"
#include "rm_utils/common.hpp"

namespace pka::auto_aim {

// Armor size, Unit: m
// 装甲板尺寸，单位为米
// constexpr 编译时常量
constexpr double SMALL_ARMOR_WIDTH = 133.0 / 1000.0; // 135
constexpr double SMALL_ARMOR_HEIGHT = 55.0 / 1000.0; // 55 // 50
constexpr double LARGE_ARMOR_WIDTH = 227.0 / 1000.0; // 225
constexpr double LARGE_ARMOR_HEIGHT = 55.0 / 1000.0; // 55 // 50

// 15 degree in rad
constexpr double FIFTTEN_DEGREE_RAD = 15 * CV_PI / 180;

// Armor type
// 装甲板类型{大，小，无效}
enum class ArmorType { SMALL, LARGE, INVALID };
inline std::string armorTypeToString(const ArmorType &type) 
{
  switch (type) 
  {
    case ArmorType::SMALL:
      return "small";
    case ArmorType::LARGE:
      return "large";
    default:
      return "invalid";
  }
}

// Struct used to store the light bar
// 用于存放灯条数据的结构体 继承于旋转矩形RotatedRect
struct Light : public cv::RotatedRect {
  // 默认构造函数
  Light() = default;
  // 有参构造函数 
  // 将轮廓转换成最小外接矩形，并初始化一个颜色为WHITE
  explicit Light(const std::vector<cv::Point> &contour)
  : cv::RotatedRect(cv::minAreaRect(contour)), color(EnemyColor::WHITE) 
  {
    PKA_ASSERT(contour.size() > 0);

    center = std::accumulate
    (
      contour.begin(),
      contour.end(),
      cv::Point2f(0, 0),
      // n为轮廓点的数量；返回的值为轮廓的所有点质心坐标
      [n = static_cast<float>(contour.size())](const cv::Point2f &a, const cv::Point &b) 
      {
        return a + cv::Point2f(b.x, b.y) / n;
      }
    );

    // p用于存储四个角点的坐标点
    cv::Point2f p[4]; 
    this->points(p);
    // Opencv坐标系
    /*
    
    |------------>x
    |
    |
    |
    |
    y
    */

   // 将四个角点按y轴的坐标从小到大排列
    std::sort(p, p + 4, [](const cv::Point2f &a, const cv::Point2f &b) { return a.y < b.y; });
    // 计算灯条最小外接矩形的顶部坐标和底部坐标
    top = (p[0] + p[1]) / 2;
    bottom = (p[2] + p[3]) / 2;

    length = cv::norm(top - bottom);
    //灯条宽度
    width = cv::norm(p[0] - p[1]);

    // 设顶点为A，底部为B
    //axis = OA-OB(向量相减) = BA向量
    axis = top - bottom;
    // 单位化
    axis = axis / cv::norm(axis);

    // Calculate the tilt angle
    // The angle is the angle between the light bar and the horizontal line
    /*计算倾斜角度；
    角度是灯条和水平线之间的角度*/
    /*
            /
          / a
        /________
      即a的角度
    */
    tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
    // 弧度转角度
    tilt_angle = tilt_angle / CV_PI * 180;
  }


  // 数据成员
  EnemyColor color;
  // 三个坐标点（顶部、底部、中心）
  cv::Point2f top, bottom, center;
  // 
  cv::Point2f axis;
  double length;
  double width;
  float tilt_angle;
};

// Struct used to store the armor
// 用于存储装甲板数据的结构体
struct Armor 
{
  // 定义一个静态编译时常量N_LANDMARKS = 6
  static constexpr const int N_LANDMARKS = 6;
  static constexpr const int N_LANDMARKS_2 = N_LANDMARKS * 2;
  // 装甲板无参构造函数
  Armor() = default;
  // 装甲板有参构造函数
  // 判断左右灯条
  Armor(const Light &l1, const Light &l2) 
  {
    if (l1.center.x < l2.center.x) 
    {
      left_light = l1, right_light = l2;
    } 
    else 
    {
      left_light = l2, right_light = l1;
    }
    // 装甲板中心坐标
    center = (left_light.center + right_light.center) / 2;
  }

  // Build the points in the object coordinate system, start from bottom left in
  // clockwise order 
  // 在对象坐标系中构建点，从左下角开始顺时针构建
  template <typename PointType>
  static inline std::vector<PointType> buildObjectPoints(const double &w,
                                                         const double &h) noexcept 
  {
    if constexpr (N_LANDMARKS == 4) 
    {
      return {PointType(0, w / 2, -h / 2),
              PointType(0, w / 2, h / 2),
              PointType(0, -w / 2, h / 2),
              PointType(0, -w / 2, -h / 2)};
    } 
    else 
    {
      return {PointType(0, w / 2, -h / 2),
              PointType(0, w / 2, 0),
              PointType(0, w / 2, h / 2),
              PointType(0, -w / 2, h / 2),
              PointType(0, -w / 2, 0),
              PointType(0, -w / 2, -h / 2)};
    }
  }

  // Landmarks start from bottom left in clockwise order
  std::vector<cv::Point2f> landmarks() const {
    if constexpr (N_LANDMARKS == 4) 
    {
      return {left_light.bottom, left_light.top, right_light.top, right_light.bottom};
    } 
    else 
    {
      return {left_light.bottom,
              left_light.center,
              left_light.top,
              right_light.top,
              right_light.center,
              right_light.bottom};
    }
  }

  // Light pairs part
  // 灯条对部分
  Light left_light, right_light;
  cv::Point2f center;
  ArmorType type;

  // Number part
  // 数字部分
  cv::Mat number_img;
  std::string number;
  // 置信度
  float confidence;
  // 分类器结果
  std::string classfication_result;
};

}  // namespace pka::auto_aim
#endif  // ARMOR_DETECTOR_ARMOR_HPP_
