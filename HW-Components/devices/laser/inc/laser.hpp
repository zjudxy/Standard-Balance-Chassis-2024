/**
 *******************************************************************************
 * @file      : laser.hpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2023-11-27      Caikunzhen      1. 未测试版本
 *  V1.0.0      2023-12-05      Caikunzhen      1. 完成测试
 *******************************************************************************
 * @attention : 定时器的周期不可太大，否者将引起红点激光的闪烁
 *******************************************************************************
 *  Copyright (c) 2023 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_LASER_LASER_HPP_
#define HW_COMPONENTS_DEVICES_LASER_LASER_HPP_

/* Includes ------------------------------------------------------------------*/
#include "allocator.hpp"
#include "stm32_hal.hpp"

namespace hello_world
{
namespace laser
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

class Laser : public MemMang
{
 public:
  Laser(TIM_HandleTypeDef *htim, uint32_t channel, float init_lumin_pct);
  virtual ~Laser() {}

  /**
   * @brief       启用红点激光器
   * @retval       None
   * @note        None
   */
  void enable() const { HAL_TIM_PWM_Start(kHtim_, kChannel_); }

  /**
   * @brief       关闭红点激光器
   * @retval       None
   * @note        None
   */
  void disable() const { HAL_TIM_PWM_Stop(kHtim_, kChannel_); }

  void setLuminPct(float lumin_pct) const;

  TIM_HandleTypeDef *const kHtim_;
  const uint32_t kChannel_;

 private:
  uint32_t luminPct2Cmp(float lumin_pct) const;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace laser
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_LASER_LASER_HPP_ */
