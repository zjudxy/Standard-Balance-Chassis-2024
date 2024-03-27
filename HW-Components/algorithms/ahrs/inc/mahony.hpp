/**
 *******************************************************************************
 * @file      : mahony.hpp
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
#ifndef HW_COMPONETS_ALGORITHMS_AHRS_MAHONY_HPP_
#define HW_COMPONETS_ALGORITHMS_AHRS_MAHONY_HPP_

/* Includes ------------------------------------------------------------------*/
#include "ahrs_base.hpp"

namespace hello_world
{
namespace ahrs
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

class Mahony : public Ahrs
{
 public:
  Mahony(float samp_freq, float kp, float ki);
  Mahony(const float quat_init[4], float samp_freq, float kp, float ki);
  virtual ~Mahony() {}

  virtual void update(
      const float acc_data[3], const float gyro_data[3]) override;

 private:
  float samp_freq_;
  float dbl_kp_;  ///* 2 * kp
  float dbl_ki_;  ///* 2 * ki
  float i_out_[3] = {0, 0, 0};
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace ahrs
}  // namespace hello_world

#endif /* HW_COMPONETS_ALGORITHMS_AHRS_MAHONY_HPP_ */
