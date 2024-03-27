/**
 *******************************************************************************
 * @file      : ahrs_base.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2023-12-12      Caikunzhen      1. 完成编写（未测试）
 *  V1.0.0      2023-12-15      Caikunzhen      1. 完成测试
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2023 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ahrs_base.hpp"

#include "arm_math.h"
#include "assert.hpp"

namespace hello_world
{
namespace ahrs
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
 * @brief       AHRS 初始化
 * @param        quat_init: 初始化四元数，[qw qx qy qz]
 * @retval       None
 * @note        初始化四元数需要为单位四元数
 */
Ahrs::Ahrs(const float quat_init[4])
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(quat_init, "Error pointer");
#pragma endregion

  memcpy(quat_, quat_init, sizeof(float) * 4);
}

/**
 * @brief       获取当前姿态对应的四元数
 * @param        quat: 当前姿态对应的四元数
 * @retval       None
 * @note        None
 */
void Ahrs::getQuat(float quat[4]) const
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(quat, "Error pointer");
#pragma endregion
  memcpy(quat, quat_, sizeof(float) * 4);
}

/**
 * @brief       获取当前姿态对应的欧拉角（Z-Y-X）
 * @param        euler_angle: 当前姿态对应的欧拉角（Z-Y-X），
 * [roll pitch yaw]，单位：rad
 * @retval       None
 * @note        None
 */
void Ahrs::getEulerAngle(float euler_angle[3]) const
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(euler_angle, "Error pointer");
#pragma endregion

  arm_atan2_f32(quat_[0] * quat_[1] + quat_[2] * quat_[3],
                quat_[0] * quat_[0] + quat_[3] * quat_[3] - 0.5f,
                euler_angle + 0);
  euler_angle[1] = asinf(-2.0f * (quat_[1] * quat_[3] - quat_[0] * quat_[2]));
  arm_atan2_f32(quat_[0] * quat_[3] + quat_[1] * quat_[2],
                quat_[0] * quat_[0] + quat_[1] * quat_[1] - 0.5f,
                euler_angle + 2);
}

/**
 * @brief       快速计算 1/sqrt(x)
 * @param        x: 数（>0）
 * @retval       计算结果
 * @note        x 必须为正数
 */
float Ahrs::invSqrt(float x) const
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(x > 0, "x <= 0");
#pragma endregion

  if (x < 0) {
    return 0;
  }

  /* http://en.wikipedia.org/wiki/Fast_inverse_square_root */
  float x_2 = 0.5f * x;
  float y = x;
  int32_t i = *reinterpret_cast<int32_t*>(&y);
  i = 0x5f3759df - (i >> 1);
  y = *reinterpret_cast<float*>(&i);
  y = y * (1.5f - (x_2 * y * y));

  return y;
}
}  // namespace ahrs
}  // namespace hello_world
