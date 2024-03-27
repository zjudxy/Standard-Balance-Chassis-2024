/** 
 *******************************************************************************
 * @file      : chassis_motor.hpp
 * @brief     : 底盘电机参数
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0     yyyy-mm-dd        dxy           <note> 
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __chassis_motor_params_H_
#define __chassis_motor_params_H_
/* Includes ------------------------------------------------------------------*/
#include "motor.hpp"
/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
namespace motor = hello_world::motor;
extern motor::Motor* chassis_motor_ptr[6];
extern motor::MotorType chassis_motor_type[6];
extern motor::OptionalParams chassis_motor_params[6];
extern motor::Motor* test_motor_ptr;
extern motor::Motor* yaw_motor_ptr;
/** 
 * @brief      底盘电机实例化
 * @retval      None
 * @note        None
 */
void chassis_motor_Init(void);



#endif /* __FILE_H_ */
