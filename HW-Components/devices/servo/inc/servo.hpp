/**
 *******************************************************************************
 * @file      : servo.hpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2023-11-27      Caikunzhen      1. 未测试版本
 *  V1.0.0      2023-12-15      Caikunzhen      1. 完成测试
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2023 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_SERVO_SERVO_HPP_
#define HW_COMPONENTS_DEVICES_SERVO_SERVO_HPP_

/* Includes ------------------------------------------------------------------*/
#include "allocator.hpp"
#include "stm32_hal.hpp"

namespace hello_world
{
namespace servo
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
struct ServoInfo {
  float operating_freq;         ///* PWM频率，单位：Hz，一般 50 Hz ~ 300 Hz
  uint16_t min_pulse_duration;  ///* 最小脉冲时长，单位：us
  uint16_t max_pulse_duration;  ///* 最大脉冲时长，单位：us
  float angle_range;            ///* 舵机角度范围（从0开始），单位：deg
};

class Servo : public MemMang
{
 public:
  Servo(TIM_HandleTypeDef *htim, uint32_t channel,
        const ServoInfo &servo_info, float init_angle);
  virtual ~Servo() {}

  /**
   * @brief       使能舵机
   * @retval       None
   * @note        None
   */
  void enable() const { HAL_TIM_PWM_Start(kHtim_, kChannel_); }

  /**
   * @brief       失能舵机
   * @retval       None
   * @note        None
   */
  void disable() const { HAL_TIM_PWM_Stop(kHtim_, kChannel_); }

  void setAngle(float angle) const;

  TIM_HandleTypeDef *const kHtim_;
  const uint32_t kChannel_;
  const ServoInfo kServoInfo_;

 private:
  uint32_t angle2Cmp(float angle) const;
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace servo
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_SERVO_SERVO_HPP_ */
