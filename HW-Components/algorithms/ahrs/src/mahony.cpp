/**
 *******************************************************************************
 * @file      : mahony.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2023-12-12      Caikunzhen      1. 完成编写（未测试）
 *  V1.0.0      2023-12-15      Caikunzhen      1. 完成测试
 *******************************************************************************
 * @attention : 具体算法可查看：https://zhuanlan.zhihu.com/p/342703388
 *******************************************************************************
 *  Copyright (c) 2023 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "mahony.hpp"

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

static inline void cross(
    const float vec1[3], const float vec2[3], float vec_out[3]);

/**
 * @brief       Mahony 初始化
 * @param        samp_freq: 采样频率，单位：Hz
 * @param        kp: 比例系数（>=0）
 * @param        ki: 积分系数（>=0）
 * @retval       None
 * @note        None
 */
Mahony::Mahony(float samp_freq, float kp, float ki)
    : Ahrs(), samp_freq_(samp_freq), dbl_kp_(2 * kp), dbl_ki_(2 * ki)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(samp_freq, "Sample frequence less than 0");
  HW_ASSERT(kp >= 0, "kp < 0");
  HW_ASSERT(ki >= 0, "ki < 0");
#pragma endregion
}

/**
 * @brief       Mahony 初始化
 * @param        quat_init: 初始化四元数，[qw qx qy qz]
 * @param        samp_freq: 采样频率，单位：Hz
 * @param        kp: 比例系数（>=0）
 * @param        ki: 积分系数（>=0）
 * @retval       None
 * @note        初始化四元数需要为单位四元数
 */
Mahony::Mahony(const float quat_init[4], float samp_freq, float kp, float ki)
    : Ahrs(quat_init), samp_freq_(samp_freq), dbl_kp_(2 * kp), dbl_ki_(2 * ki)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(samp_freq, "Sample frequence less than 0");
  HW_ASSERT(kp >= 0, "kp < 0");
  HW_ASSERT(ki >= 0, "ki < 0");
#pragma endregion
}

/**
 * @brief       根据反馈数据进行姿态更新
 * @param        acc_data: 加速度计三轴数据，[ax ay az]，无单位要求
 * @param        gyro_data: 陀螺仪三轴数据，[wx wy wz]，单位：rad/s
 * @retval       None
 * @note        加速度计三轴数据需包含重力加速度项
 */
void Mahony::update(const float acc_data[3], const float gyro_data[3])
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(acc_data, "acc_data is empty ptr");
  HW_ASSERT(gyro_data, "gyro_data is empty ptr");
#pragma endregion

  float norm_scale, err_2[3] = {0, 0, 0};
  if (!(acc_data[0] == 0 && acc_data[1] == 0 && acc_data[2] == 0)) {
    float v_bar[3], v_hat_2[3];
    norm_scale = invSqrt(acc_data[0] * acc_data[0] + acc_data[1] * acc_data[1] +
                         acc_data[2] * acc_data[2]);
    arm_scale_f32(acc_data, norm_scale, v_bar, 3);

    v_hat_2[0] = quat_[1] * quat_[3] - quat_[0] * quat_[2];
    v_hat_2[1] = quat_[2] * quat_[3] + quat_[0] * quat_[1];
    v_hat_2[2] = 0.5f - quat_[1] * quat_[1] - quat_[2] * quat_[2];

    cross(v_bar, v_hat_2, err_2);
  }

  static float w_x_dt_2[3], gyro_err;
  for (uint8_t i = 0; i < 3; i++) {
    i_out_[i] += dbl_ki_ * err_2[i] / samp_freq_;
    gyro_err = dbl_kp_ * err_2[i] + i_out_[i];
    w_x_dt_2[i] = (gyro_data[i] + gyro_err) * 0.5f / samp_freq_;
  }

  float delta_quat[4], w_x_dt_2_ex[4] = {
                           0, w_x_dt_2[0], w_x_dt_2[1], w_x_dt_2[2]};
  arm_quaternion_product_single_f32(quat_, w_x_dt_2_ex, delta_quat);
  float quat[4];
  arm_add_f32(quat_, delta_quat, quat, 4);

  norm_scale = invSqrt(quat[0] * quat[0] + quat[1] * quat[1] +
                       quat[2] * quat[2] + quat[3] * quat[3]);
  arm_scale_f32(quat, norm_scale, quat_, 4);
}

/**
 * @brief       计算两向量的叉乘
 * @param        vec1: 向量1
 * @param        vec2: 向量2
 * @param        vec_out: 叉乘结果
 * @retval       None
 * @note        None
 */
static inline void cross(
    const float vec1[3], const float vec2[3], float vec_out[3])
{
  vec_out[0] = vec1[1] * vec2[2] - vec2[1] * vec1[2];
  vec_out[1] = vec2[0] * vec1[2] - vec1[0] * vec2[2];
  vec_out[2] = vec1[0] * vec2[1] - vec2[0] * vec1[1];
}
}  // namespace ahrs
}  // namespace hello_world
