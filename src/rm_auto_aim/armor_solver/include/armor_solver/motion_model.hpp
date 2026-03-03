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

#ifndef ARMOR_SOLVER_MOTION_MODEL_HPP_
#define ARMOR_SOLVER_MOTION_MODEL_HPP_

// ceres
#include <ceres/ceres.h>
// project
#include "rm_utils/math/extended_kalman_filter.hpp"

namespace pka::auto_aim {

// 枚举运动模型类
/*  包括:

恒定速度
恒定旋转速度
恒定速度和恒定旋转速度

*/
enum class MotionModel {
  CONSTANT_VELOCITY = 0,  // Constant velocity 恒定速度
  CONSTANT_ROTATION = 1,  // Constant rotation velocity 恒定旋转速度
  CONSTANT_VEL_ROT = 2    // Constant velocity and rotation velocity 恒定速度和旋转速度
};

// X_N: state dimension 状态维度, Z_N: measurement dimension 测量维度
constexpr int X_N = 10, Z_N = 4;

// 预测数据结构体
struct Predict {
  // 预测结构体函数 运动模型的默认参数为恒定速度和恒定旋转速度
  explicit Predict(double dt, MotionModel model = MotionModel::CONSTANT_VEL_ROT)
  : dt(dt), model(model) {}


  // 初始化模板函数operator(),作用是根据给定的运动模型model来更新一个状态向量（从x0->x1）的值
  template <typename T>
  void operator()(const T x0[X_N], T x1[X_N]) 
  {
    //初始化
    for (int i = 0; i < X_N; i++) 
    {
      x1[i] = x0[i];
    }

    // v_xyz
    // x,y,z分量上的速度
    // 如果为恒定速度和旋转速度或者是恒定速度
    /* x向量
    x[0]:该时刻x的位置
    x[1]:该时刻的v_x
    x[2]:该时刻y的位置
    x[3]:该时刻的v_y
    x[4]:该时刻z的位置
    x[5]:该时刻的v_z
    x[6]:该时刻的yaw
    x[7]:该时刻的v_yaw
    x[8]:r
    x[9]:d_za
    */

    if (model == MotionModel::CONSTANT_VEL_ROT || model == MotionModel::CONSTANT_VELOCITY) 
    {
      // linear velocity
      // 线速度
      // x1[0] = x0[0]+x0[1]*dt;
      x1[0] += x0[1] * dt;
      x1[2] += x0[3] * dt;
      x1[4] += x0[5] * dt;
    }
    else 
    {
      // no velocity
      // 没有线速度
      x1[1] *= 0.;
      x1[3] *= 0.;
      x1[5] *= 0.;
    }

    // v_yaw
    // 角速度
    // 如果为恒定速度和旋转速度或者是恒定旋转速度
    if (model == MotionModel::CONSTANT_VEL_ROT || model == MotionModel::CONSTANT_ROTATION) 
    {
      // angular velocity
      // 角速度
      x1[6] += x0[7] * dt;
    }
    else 
    {
      // no rotation
      // 没有角速度
      x1[7] *= 0.;
    }
  }

  double dt;
  MotionModel model;
};




// 观测结构体
struct Measure {

  // 初始化模板函数operator()
  /*
  z向量
  z[0]:该时刻的xa即x_armor
  z[1]:该时刻的ya即y_armor
  z[2]:该时刻的za即z_armor
  z[3]:该时刻的yaw
  */
  template <typename T>
  void operator()(const T x[Z_N], T z[Z_N]) 
  {
    z[0] = x[0] - ceres::cos(x[6]) * x[8];
    z[1] = x[2] - ceres::sin(x[6]) * x[8];
    z[2] = x[4] + x[9];
    z[3] = x[6];
  }
};

using RobotStateEKF = ExtendedKalmanFilter<X_N, Z_N, Predict, Measure>;

}  // namespace pka::auto_aim
#endif
