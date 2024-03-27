/**
 *******************************************************************************
 * @file      : servo.cpp
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
/* Includes ------------------------------------------------------------------*/
#include "servo.hpp"

#include "assert.hpp"
#include "base.hpp"

namespace hello_world
{
namespace servo
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
static const float kTimerFreq = 1e6f;
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
 * @brief       舵机初始化
 * @param        htim: 定时器句柄指针
 * @param        channel: 定时器PWM对应输出通道
 * @param        servo_info: 舵机参数
 * @param        init_angle: 舵机初始化角度，单位：deg
 * @retval       None
 * @note        使用前请先确保定时器的频率为 1MHz，
 * 舵机角度会被自动限制在设定的角度范围内，初始化后舵机默认处于开启状态
 */
Servo::Servo(TIM_HandleTypeDef* htim, uint32_t channel,
             const ServoInfo& servo_info, float init_angle)
    : kHtim_(htim), kChannel_(channel), kServoInfo_(servo_info)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(IS_TIM_INSTANCE(kHtim_->Instance), "Error TIM handle");
  HW_ASSERT(IS_TIM_CHANNELS(kChannel_), "Error TIM channel: %d", kChannel_);
  HW_ASSERT(kServoInfo_.operating_freq > 0, "Error Operating freqency: %.1f",
            kServoInfo_.operating_freq);
  HW_ASSERT(kServoInfo_.min_pulse_duration < kServoInfo_.max_pulse_duration &&
                kServoInfo_.min_pulse_duration > 0u &&
                kServoInfo_.max_pulse_duration <
                    static_cast<uint16_t>(1e6f / kServoInfo_.operating_freq),
            "Error pulse duration");
  HW_ASSERT(kServoInfo_.angle_range <= 360.0f,
            "Error angle range: %d", kServoInfo_.angle_range);
#pragma endregion

  uint32_t auto_reload =
      static_cast<uint32_t>(kTimerFreq / kServoInfo_.operating_freq) - 1;
  __HAL_TIM_SET_AUTORELOAD(kHtim_, auto_reload);
  __HAL_TIM_SET_COMPARE(kHtim_, kChannel_, angle2Cmp(init_angle));
  HAL_TIM_PWM_Start(kHtim_, kChannel_);
}

/**
 * @brief       设定舵机角度
 * @param        angle: 舵机角度，单位：deg
 * @retval       None
 * @note        舵机角度会被自动限制在设定的角度范围内
 */
void Servo::setAngle(float angle) const
{
  __HAL_TIM_SET_COMPARE(kHtim_, kChannel_, angle2Cmp(angle));
}

/**
 * @brief       舵机角度转定时器PWM比较值
 * @param        angle: 舵机角度，单位：deg
 * @retval       定时器PWM比较值
 * @note        舵机角度会被自动限制在设定的角度范围内
 */
uint32_t Servo::angle2Cmp(float angle) const
{
  angle = Bound(angle, 0.0f, kServoInfo_.angle_range);
  uint32_t cmp = static_cast<uint32_t>(
      angle *
          (kServoInfo_.max_pulse_duration - kServoInfo_.min_pulse_duration) /
          kServoInfo_.angle_range +
      kServoInfo_.min_pulse_duration);

  return cmp;
}
}  // namespace servo
}  // namespace hello_world
