/**
 *******************************************************************************
 * @file      : motor.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2023-12-04      Caikunzhen      1. 完成测试
 *  V1.0.1      2023-12-14      Caikunzhen      1. 添加电机简单工厂函数
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2023 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "motor.hpp"

#include "assert.hpp"

namespace hello_world
{
namespace motor
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
 * @brief       根据所需的电机类型创建电机并初始化
 * @param        motor_type: 电机类型
 * @arg          kMotorTypeGM6020，kMotorTypeM2006，kMotorTypeM3508，
 * kMotorTypeM8910，kMotorTypeA1，kMotorTypeDM_J4310，kMotorTypeMF9025v2,
 * @param        id: 电机 ID
 * @param        optinal_params: 电机可选配置参数
 * @retval       创建的电机类指针
 * @note        返回的电机类是经过内存申请得到的，不使用时需要自行释放内存
 */
Motor* CreateMotor(MotorType motor_type, uint8_t id,
                   const OptionalParams& optinal_params)
{
  switch (motor_type) {
    case kMotorTypeGM6020:
      return new GM6020(id, optinal_params);
    case kMotorTypeM2006:
      return new M2006(id, optinal_params);
    case kMotorTypeM3508:
      return new M3508(id, optinal_params);
    case kMotorTypeM8910:
      return new M8910(id, optinal_params);
    case kMotorTypeA1:
      return new A1(id, optinal_params);
    case kMotorTypeDM_J4310:
      return new DM_J4310(id, optinal_params);
    case kMotorTypeDM_J8006:
      return new DM_J8006(id, optinal_params);
    case kMotorTypeMF9025v2:
      return new MF9025v2(id, optinal_params);
    default:
      HW_ASSERT(0, "Error motor type");
      return nullptr;
  }
}
}  // namespace motor
}  // namespace hello_world
