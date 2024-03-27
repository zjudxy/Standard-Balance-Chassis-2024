/**
 *******************************************************************************
 * @file      : laser.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2023-11-27      Caikunzhen      1. 未测试版本
 *  V1.0.0      2023-12-05      Caikunzhen      1. 完成测试
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2023 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "laser.hpp"

#include "assert.hpp"
#include "base.hpp"

namespace hello_world
{
namespace laser
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
 * @brief       红点激光器初始化
 * @param        htim: 定时器句柄指针
 * @param        channel: 定时器PWM对应输出通道
 * @param        init_lumin_pct: 初始化百分比亮度（0 ~ 100）
 * @retval       None
 * @note        百分比亮度会被自动限制在合理范围内，
 * 初始化后红点激光器默认处于开启状态
 */
Laser::Laser(TIM_HandleTypeDef *htim, uint32_t channel, float init_lumin_pct)
    : kHtim_(htim), kChannel_(channel)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(IS_TIM_INSTANCE(kHtim_->Instance), "Error TIM handle");
  HW_ASSERT(IS_TIM_CHANNELS(kChannel_), "Error TIM channel: %d", kChannel_);
#pragma endregion

  __HAL_TIM_SET_COMPARE(kHtim_, kChannel_, luminPct2Cmp(init_lumin_pct));
  HAL_TIM_PWM_Start(kHtim_, kChannel_);
}

/**
 * @brief       设置红点激光器百分比亮度
 * @param        lumin_pct: 百分比亮度（0 ~ 100）
 * @retval       None
 * @note        百分比亮度会被自动限制在合理范围内
 */
void Laser::setLuminPct(float lumin_pct) const
{
  __HAL_TIM_SET_COMPARE(kHtim_, kChannel_, luminPct2Cmp(lumin_pct));
}

/**
 * @brief       百分比亮度转定时器PWM比较值
 * @param        lumin_pct: 百分比亮度（0 ~ 100）
 * @retval       定时器PWM比较值
 * @note        百分比亮度会被自动限制在合理范围内
 */
uint32_t Laser::luminPct2Cmp(float lumin_pct) const
{
  lumin_pct = Bound(lumin_pct, 0.0f, 100.0f);
  return static_cast<uint32_t>(
      lumin_pct * __HAL_TIM_GET_AUTORELOAD(kHtim_) / 100);
}
}  // namespace laser
}  // namespace hello_world
