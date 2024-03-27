/**
 *******************************************************************************
 * @file      : motor.hpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2023-11-25      Caikunzhen      1. 未测试版本
 *  V1.0.0      2023-12-04      Caikunzhen      1. 完成测试
 *  V1.0.1      2023-12-14      Caikunzhen      1. 添加电机简单工厂函数
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2023 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_MOTOR_MOTOR_HPP_
#define HW_COMPONENTS_DEVICES_MOTOR_MOTOR_HPP_

/* Includes ------------------------------------------------------------------*/
#include "motor_A1.hpp"
#include "motor_DM_J4310.hpp"
#include "motor_GM6020.hpp"
#include "motor_M2006.hpp"
#include "motor_M3508.hpp"
#include "motor_M8910.hpp"
#include "motor_MF9025v2.hpp"
#include "motor_base.hpp"

namespace hello_world
{
namespace motor
{
/* Exported macro ------------------------------------------------------------*/

enum MotorType {
  kMotorTypeGM6020,
  kMotorTypeM2006,
  kMotorTypeM3508,
  kMotorTypeM8910,
  kMotorTypeA1,
  kMotorTypeDM_J4310,
  kMotorTypeMF9025v2,
};
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

Motor* CreateMotor(MotorType motor_type, uint8_t id,
                   const OptionalParams& optinal_params = OptionalParams());
}  // namespace motor
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_MOTOR_MOTOR_HPP_ */
