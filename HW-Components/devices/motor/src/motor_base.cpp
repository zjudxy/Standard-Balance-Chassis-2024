/**
 *******************************************************************************
 * @file      : motor_base.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2023-11-25      Caikunzhen      1. 未测试版本
 *  V1.0.0      2023-12-04      Caikunzhen      1. 完成测试
 *  V1.0.1      2023-12-30      Caikunzhen      1. 修复电流输入转力矩问题
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2023 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "motor_base.hpp"

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
 * @brief       设定发给电调的期望值
 * @param        input: 发给电调的期望值
 * @retval       运行状态
 * @note        1. 期望值的物理意义与电机当前的输入类型有关，可使用
 *                 get_input_type 方法查看
 *              2. 设定的期望值会自动被限制到允许的范围内，当前实际的设定值可以通
 *                 过 getInput 方法查看
 */
State Motor::setInput(float input)
{
  float actual_input = 0;
  switch (motor_info_.input_type) {
    case kInputTypeRaw:
      raw_input_ = hello_world::Bound(
          input, motor_info_.raw_input_lim, -motor_info_.raw_input_lim);
      torq_input_ = raw2torq(raw_input_);
      curr_input_ = raw2curr(raw_input_);
      actual_input = raw_input_;
      break;
    case kInputTypeTorq:
      torq_input_ = hello_world::Bound(
          input, motor_info_.torq_input_lim, -motor_info_.torq_input_lim);
      raw_input_ = torq2raw(torq_input_);
      curr_input_ = torq2curr(torq_input_);
      actual_input = torq_input_;
      break;
    case kInputTypeCurr:
      curr_input_ = hello_world::Bound(
          input, motor_info_.curr_input_lim, -motor_info_.curr_input_lim);
      raw_input_ = curr2raw(curr_input_);
      torq_input_ = curr2torq(curr_input_);
      actual_input = curr_input_;
      break;
    default:
      return kStateInputTypeError;
  }

  if (fabsf(actual_input) < fabsf(input)) {
    return kStateInputValueOverflow;
  } else {
    return kStateOk;
  }
}

/**
 * @brief       获取发给电调的期望值
 * @retval       发给电调的期望值
 * @note        期望值的物理意义与电机当前的输入类型有关，可使用
 *                 get_input_type 方法查看
 */
float Motor::getInput(void) const
{
  switch (motor_info_.input_type) {
    case kInputTypeRaw:
      return raw_input_;
    case kInputTypeTorq:
      return torq_input_;
    case kInputTypeCurr:
      return curr_input_;
    default:
      return 0;
  }
}

/**
 * @brief       将原始报文内容转换为输出端力矩
 * @param        raw: 原始报文数值
 * @retval       原始报文对应的输出端力矩值，单位：N·m
 * @note        报文的对应的物理意义需要与设定的相符，设定对应情可以通过
 *              motor_info 方法获取电机信息结构体，查看其中的 raw_mapping_type
 *              变量
 */
float Motor::raw2torq(float raw) const
{
  if (motor_info_.raw_mapping_type == kRawMappingTypeTorq) {
    return raw * motor_info_.torq_rat * motor_info_.redu_rat;
  } else if (motor_info_.raw_mapping_type == kRawMappingTypeCurr) {
    return raw * motor_info_.curr_rat * motor_info_.torq_const *
           motor_info_.redu_rat;
  } else {
    return 0;
  }
}

/**
 * @brief       将输出端力矩转换为原始报文内容
 * @param        torq: 输出端力矩值，单位：N·m
 * @retval       输出端力矩值对应的原始报文
 * @note        报文的对应的物理意义需要与设定的相符，设定对应情可以通过
 *              motor_info 方法获取电机信息结构体，查看其中的 raw_mapping_type
 *              变量
 */
float Motor::torq2raw(float torq) const
{
  if (motor_info_.raw_mapping_type == kRawMappingTypeTorq) {
    return torq / motor_info_.redu_rat / motor_info_.torq_rat;
  } else if (motor_info_.raw_mapping_type == kRawMappingTypeCurr) {
    return torq / motor_info_.redu_rat / motor_info_.torq_const /
           motor_info_.curr_rat;
  } else {
    return 0;
  }
}

/**
 * @brief       将原始报文内容转换为转子电流
 * @param        raw: 原始报文数值
 * @retval       原始报文对应的转子电流值，单位：A
 * @note        报文的对应的物理意义需要与设定的相符，设定对应情可以通过
 *              motor_info 方法获取电机信息结构体，查看其中的 raw_mapping_type
 *              变量
 */
float Motor::raw2curr(float raw) const
{
  if (motor_info_.raw_mapping_type == kRawMappingTypeTorq) {
    return raw * motor_info_.torq_rat / motor_info_.torq_const;
  } else if (motor_info_.raw_mapping_type == kRawMappingTypeCurr) {
    return raw * motor_info_.curr_rat;
  } else {
    return 0;
  }
}

/**
 * @brief       将转子电流转换为原始报文内容
 * @param        curr: 转子电流值，单位：A
 * @retval       转子电流值对应的原始报文
 * @note        报文的对应的物理意义需要与设定的相符，设定对应情可以通过
 *              motor_info 方法获取电机信息结构体，查看其中的 raw_mapping_type
 *              变量
 */
float Motor::curr2raw(float curr) const
{
  if (motor_info_.raw_mapping_type == kRawMappingTypeTorq) {
    return curr * motor_info_.torq_const / motor_info_.torq_rat;
  } else if (motor_info_.raw_mapping_type == kRawMappingTypeCurr) {
    return curr / motor_info_.curr_rat;
  } else {
    return 0;
  }
}

/**
 * @brief       将输出端力矩转换为转子电流
 * @param        torq: 输出端力矩值，单位：N·m
 * @retval       输出端力矩值对应的转子电流，单位：A
 * @note        None
 */
float Motor::torq2curr(float torq) const
{
  return torq / motor_info_.redu_rat / motor_info_.torq_const;
}

/**
 * @brief       将转子电流转换为输出端力矩
 * @param        curr: 转子电流值，单位：A
 * @retval       转子电流值对应的输出端力矩，单位：N·m
 * @note        None
 */
float Motor::curr2torq(float curr) const
{
  return curr * motor_info_.torq_const * motor_info_.redu_rat;
}

/**
 * @brief       设置点击的输入类型
 * @param        input_type: 期望输入类型
 * @arg         kInputTypeRaw: 原始报文输入
 * @arg         kInputTypeTorq: 输出端力矩输入
 * @arg         kInputTypeCurr: 转子端电流输入
 * @retval       None
 * @note        None
 */
State Motor::set_input_type(InputType input_type)
{
  if (input_type == kInputTypeRaw || input_type == kInputTypeTorq ||
      input_type == kInputTypeCurr) {
    motor_info_.input_type = input_type;
    return kStateOk;
  } else {
    return kStateInputTypeError;
  }
}

/**
 * @brief       将角度转换到指定范围
 * @param        angle: 待转换的角度，单位：rad
 * @retval       归一化后的角度，单位：rad
 * @note        None
 */
float Motor::normAngle(float angle)
{
  switch (motor_info_.angle_range) {
    case kAngleRangeNegPiToPosPi:
      return NormPeriodData(-PI, PI, angle);
    case kAngleRange0To2Pi:
      return NormPeriodData(0, 2 * PI, angle);
    case kAngleRangeNegInfToPosInf:
    default:
      return angle;
  }
}
}  // namespace motor
}  // namespace hello_world