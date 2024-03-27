/**
 *******************************************************************************
 * @file      : ahrs_base.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONETS_ALGORITHMS_AHRS_AHRS_BASE_HPP_
#define HW_COMPONETS_ALGORITHMS_AHRS_AHRS_BASE_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstring>

#include "allocator.hpp"

namespace hello_world
{
namespace ahrs
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

class Ahrs : public MemMang
{
 public:
  Ahrs(){};
  Ahrs(const float quat_init[4]);
  virtual ~Ahrs() {}

  /**
   * @brief       根据反馈数据进行姿态更新
   * @param        acc_data: 加速度计三轴数据，[ax ay az]，无单位要求
   * @param        gyro_data: 陀螺仪三轴数据，[wx wy wz]，单位：rad/s
   * @retval       None
   * @note        加速度计三轴数据需包含重力加速度项
   */
  virtual void update(const float acc_data[3], const float gyro_data[3]) = 0;

  void getQuat(float quat[4]) const;

  void getEulerAngle(float euler_angle[3]) const;

 protected:
  float invSqrt(float x) const;

  float quat_[4] = {1.0f, 0.0f, 0.0f, 0.0f};
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace ahrs
}  // namespace hello_world

#endif /* HW_COMPONETS_ALGORITHMS_AHRS_AHRS_BASE_HPP_ */
