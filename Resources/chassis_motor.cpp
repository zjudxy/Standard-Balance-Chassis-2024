/**
 *******************************************************************************
 * @file      : chassis_motor.cpp
 * @brief     :
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
/* Includes ------------------------------------------------------------------*/
#include "chassis_motor.hpp"
#include "chassis.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
motor::Motor* chassis_motor_ptr[6];
motor::Motor* yaw_motor_ptr=NULL;
motor::MotorType chassis_motor_type[6] = {
    motor::kMotorTypeA1,
    motor::kMotorTypeA1,
    motor::kMotorTypeA1,
    motor::kMotorTypeA1,
    motor::kMotorTypeM8910,
    motor::kMotorTypeM8910
    // motor::kMotorTypeMF9025v2,
    // motor::kMotorTypeMF9025v2
    };
motor::MotorType yaw_motor_type = motor::kMotorTypeDM_J4310; 
uint8_t chassis_motor_id[6]={
    4,5,1,2,1,2
};
//9025
// uint8_t chassis_motor_id[6]={
//     4,5,1,2,2,1
// };
uint8_t yaw_motor_id=2;

motor::OptionalParams chassis_motor_params[6] = {
    motor::OptionalParams{
        .input_type = motor::kInputTypeTorq,                    // 力矩输入
        .angle_range = motor::kAngleRangeNegPiToPosPi,          // 角度范围 [-π, π)
        .dir = motor::kDirFwd,                                  // 电机正方向与规定正方向相反
        // .remove_build_in_reducer = true,                       // 移除自带减速器
        .angle_offset = 0,                                     // 电机输出端实际角度与规定角度的差为 π
        .ex_redu_rat = 1,                                       // 电机额外加的减速器减速比为 1
        // .max_raw_input_lim = std::numeric_limits<float>::max(), // 不对报文进行限制
        .max_torq_input_lim = 30.0f,                             // 限制输出端最大输出力矩为 3N·m
        // .max_curr_input_lim = 30.0f,
    },
    motor::OptionalParams{
        .input_type = motor::kInputTypeTorq,                    // 力矩输入
        .angle_range = motor::kAngleRangeNegPiToPosPi,          // 角度范围 [-π, π)
        .dir = motor::kDirFwd,                                  // 电机正方向与规定正方向相反
        // .remove_build_in_reducer = true,                        // 移除自带减速器
        .angle_offset = 0,                                     // 电机输出端实际角度与规定角度的差为 π
        .ex_redu_rat = 1,                                       // 电机额外加的减速器减速比为 1
        // .max_raw_input_lim = std::numeric_limits<float>::max(), // 不对报文进行限制
        .max_torq_input_lim = 30.0f,                             // 限制输出端最大输出力矩为 3N·m
        // .max_curr_input_lim = 30.0f,
    },
    motor::OptionalParams{
        .input_type = motor::kInputTypeTorq,                    // 力矩输入
        .angle_range = motor::kAngleRangeNegPiToPosPi,          // 角度范围 [-π, π)
        .dir = motor::kDirRev,                                  // 电机正方向与规定正方向相反
        // .remove_build_in_reducer = true,                        // 移除自带减速器
        .angle_offset = 0,                                     // 电机输出端实际角度与规定角度的差为 π
        .ex_redu_rat = 1,                                       // 电机额外加的减速器减速比为 1
        // .max_raw_input_lim = std::numeric_limits<float>::max(), // 不对报文进行限
        .max_torq_input_lim = 30.0f,                             // 限制输出端最大输出力矩为 3N·m
        // .max_curr_input_lim = 30.0f,
    },
    motor::OptionalParams{
        .input_type = motor::kInputTypeTorq,                    // 力矩输入
        .angle_range = motor::kAngleRangeNegPiToPosPi,          // 角度范围 [-π, π)
        .dir = motor::kDirRev,                                  // 电机正方向与规定正方向相反
        // .remove_build_in_reducer = true,                        // 移除自带减速器
        .angle_offset = 0,                                     // 电机输出端实际角度与规定角度的差为 π
        .ex_redu_rat = 1,                                       // 电机额外加的减速器减速比为 1
        // .max_raw_input_lim = std::numeric_limits<float>::max(), // 不对报文进行限制
        .max_torq_input_lim = 30.0f,                             // 限制输出端最大输出力矩为 3N·m
        // .max_curr_input_lim = 30.0f,
    },
    motor::OptionalParams{
        .input_type = motor::kInputTypeTorq,                    // 力矩输入
        .angle_range = motor::kAngleRangeNegPiToPosPi,          // 角度范围 [-π, π)
        .dir = motor::kDirFwd,                                  // 电机正方向与规定正方向相同
        .remove_build_in_reducer = false,                        // 移除自带减速器
        .angle_offset = 0,                                     // 电机输出端实际角度与规定角度的差为 π
        .ex_redu_rat = 14.0/3.0,                                       // 电机额外加的减速器减速比为 1
        // .max_raw_input_lim = std::numeric_limits<float>::max() // 不对报文进行限制
        // .max_torq_input_lim = 8.0f,                             // 限制输出端最大输出力矩为 3N·m
        // .max_curr_input_lim = 10.0f,
    },
    motor::OptionalParams{
        .input_type = motor::kInputTypeTorq,                    // 力矩输入
        .angle_range = motor::kAngleRangeNegPiToPosPi,          // 角度范围 [-π, π)
        .dir = motor::kDirRev,                                  // 电机正方向与规定正方向相反
        .remove_build_in_reducer = false,                        // 移除自带减速器
        .angle_offset = 0,                                     // 电机输出端实际角度与规定角度的差为 π
        .ex_redu_rat = 14.0/3.0,                                       // 电机额外加的减速器减速比为 1
        // .max_raw_input_lim = std::numeric_limits<float>::max() // 不对报文进行限制
        // .max_torq_input_lim = 8.0f,                             // 限制输出端最大输出力矩为 3N·m
        // .max_curr_input_lim = 10.0f,
    }
    //     motor::OptionalParams{
    //     .input_type = motor::kInputTypeTorq,                    // 力矩输入
    //     .angle_range = motor::kAngleRangeNegPiToPosPi,          // 角度范围 [-π, π)
    //     .dir = motor::kDirFwd,                                  // 电机正方向与规定正方向相同
    //     .remove_build_in_reducer = true,                        // 移除自带减速器
    //     .angle_offset = 0,                                     // 电机输出端实际角度与规定角度的差为 π
    //     // .ex_redu_rat = 1,                                       // 电机额外加的减速器减速比为 1
    //     // .max_raw_input_lim = std::numeric_limits<float>::max() // 不对报文进行限制
    //     .max_torq_input_lim = 14.0f,                             // 限制输出端最大输出力矩为 3N·m
    //     // .max_curr_input_lim = 10.0f,
    // },
    // motor::OptionalParams{
    //     .input_type = motor::kInputTypeTorq,                    // 力矩输入
    //     .angle_range = motor::kAngleRangeNegPiToPosPi,          // 角度范围 [-π, π)
    //     .dir = motor::kDirRev,                                  // 电机正方向与规定正方向相反
    //     .remove_build_in_reducer = true,                        // 移除自带减速器
    //     .angle_offset = 0,                                     // 电机输出端实际角度与规定角度的差为 π
    //     // .ex_redu_rat = 1,                                       // 电机额外加的减速器减速比为 1
    //     // .max_raw_input_lim = std::numeric_limits<float>::max() // 不对报文进行限制
    //     .max_torq_input_lim = 14.0f,                             // 限制输出端最大输出力矩为 3N·m
    //     // .max_curr_input_lim = 10.0f,
    // }
};
motor::OptionalParams yaw_motor_params = motor::OptionalParams{
    .input_type = motor::kInputTypeTorq,
    .angle_range = motor::kAngleRangeNegPiToPosPi,
    .dir = motor::kDirRev,  
    .angle_offset = -(0.0f),
    .max_raw_input_lim = std::numeric_limits<float>::max()
};
motor::Motor* test_motor_ptr;

void chassis_motor_Init(void){
    for(int i=0;i<MOTOR_NUM;i++){
    chassis_motor_ptr[i]=motor::CreateMotor(chassis_motor_type[i],chassis_motor_id[i],chassis_motor_params[i]);
    }
    yaw_motor_ptr = motor::CreateMotor(yaw_motor_type,yaw_motor_id,yaw_motor_params);
    // //test
    // motor::OptionalParams a=motor::OptionalParams{
    //     .input_type = motor::kInputTypeTorq,                    // 力矩输入
    //     .angle_range = motor::kAngleRangeNegPiToPosPi,          // 角度范围 [-π, π)
    //     .dir = motor::kDirFwd,                                  // 电机正方向与规定正方向相反
    //     .remove_build_in_reducer = true,                        // 移除自带减速器
    //     .angle_offset = 0,                                     // 电机输出端实际角度与规定角度的差为 π
    //     .ex_redu_rat = 1,                                       // 电机额外加的减速器减速比为 1
    //     .max_raw_input_lim = std::numeric_limits<float>::max(), // 不对报文进行限制
    //     .max_torq_input_lim = 7.0f,                             // 限制输出端最大输出力矩为 3N·m
    //     .max_curr_input_lim = 10.0f,
    // };
    // test_motor_ptr=motor::CreateMotor(motor::kMotorTypeDM_J4310,1,a);
}