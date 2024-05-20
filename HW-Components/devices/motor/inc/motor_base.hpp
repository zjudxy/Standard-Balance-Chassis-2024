/**
 *******************************************************************************
 * @file      : motor_base.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_MOTOR_MOTOR_BASE_HPP_
#define HW_COMPONENTS_DEVICES_MOTOR_MOTOR_BASE_HPP_

/* Includes ------------------------------------------------------------------*/
#include <limits>

#include "allocator.hpp"
#include "base.hpp"

namespace hello_world
{
namespace motor
{
/* Exported macro ------------------------------------------------------------*/

enum State : uint8_t {
  kStateOk,                  ///* 正常
  kStateInputValueOverflow,  ///* 输入溢出
  kStateInputTypeError,      ///* 输入类型错误
  kStateIdDismatch,          ///* ID 不匹配
};

enum InputType : uint8_t {
  kInputTypeRaw,   ///* 报文原始输入
  kInputTypeTorq,  ///* 输出端力矩输入
  kInputTypeCurr,  ///* 转子电流输入
  kInputTypeCmd,   ///* 电机指令输入
};

enum RawMappingType : uint8_t {
  kRawMappingTypeTorq,  ///* 原始报文对应力矩
  kRawMappingTypeCurr,  ///* 原始报文对应电流
};

enum Dir : int8_t {  ///* 电机转动正方向与规定正方向的关系
  kDirFwd = 1,       ///* 同向
  kDirRev = -1,      ///* 反向
};

enum AngleRange : uint8_t {
  kAngleRangeNegPiToPosPi,    ///* [-π, π)
  kAngleRange0To2Pi,          ///* [0, 2π)
  kAngleRangeNegInfToPosInf,  ///* (-∞, +∞)
};

/* Exported constants --------------------------------------------------------*/
const float kInvalidValue = 0;
/* Exported types ------------------------------------------------------------*/

struct MotorBaseInfo {
  /** 电机输入限制 */
  float raw_input_lim;   ///* 电调报文输入限制值
  float torq_input_lim;  ///* 电机输出端限制力矩值，单位：N·m
  float curr_input_lim;  ///* 电机电流输入限制值，单位：A

  float torq_const;        ///* 力矩常数，单位：N·m/A
  float redu_rat;          ///* 减速比
  float angle_rat;         ///* 角度分辨率
  float vel_rat;           ///* 角速度分辨率
  float curr_rat;          ///* 电流分辨率
  float torq_rat;          ///* 力矩分辨率
  uint16_t cross_0_value;  ///* 编码器过零值
  RawMappingType raw_mapping_type;
};

struct MotorInfo : public MotorBaseInfo {
  Dir dir;
  uint8_t id;
  AngleRange angle_range;
  InputType input_type;
  uint32_t rx_id;  ///* 电机发回的报文的ID
  uint32_t tx_id;  ///* 发给电机的报文的ID
  float angle_offset;  ///* 电机输出端实际角度与规定角度的差值

  MotorInfo& operator=(const MotorBaseInfo& motor_base_info)
  {
    MotorBaseInfo* base = this;
    *base = motor_base_info;
    return *this;
  }
};

/** 电机初始化可选参数 */
struct OptionalParams {
  InputType input_type = kInputTypeRaw;
  AngleRange angle_range = kAngleRangeNegInfToPosInf;
  Dir dir = kDirFwd;
  /** 是否移除电机自带的减速器 */
  bool remove_build_in_reducer = false;
  /** 电机输出端实际角度与规定角度的差值 */
  float angle_offset = 0;
  /** 电机外置减速器的减速比（额外） */
  float ex_redu_rat = 1;
  /** 报文输入限制 */
  float max_raw_input_lim = std::numeric_limits<float>::max();
  /** 力矩输入限制 */
  float max_torq_input_lim = std::numeric_limits<float>::max();
  /** 电流输入限制 */
  float max_curr_input_lim = std::numeric_limits<float>::max();
};

class Motor : public MemMang
{
 public:
  Motor(){};
  virtual ~Motor(){};

  virtual State setInput(float input);

  float getInput(void) const;

  /**
   * @brief       将当前电机输出端所处位置设置为指定的角度
   * @param        angle: 指定角度，单位：rad
   * @retval       None
   * @note        None
   */
  void setAngleValue(float angle)
  {
    motor_info_.angle_offset = actual_angle_ - angle;
  }

  /**
   * @brief       将要发给电调的期望输值编码为对应的 CAN 报文
   * @param        tx_data: 将要发出的 CAN 报文
   * @param        tx_msg_std_id: 发给电调的报文的ID
   * @retval       当 ID 匹配时返回 kStateOk，否则返回 kStateIdDismatch
   * @note        1. 只有当 tx_msg_std_id 与实际的发给电机报文的 ID 对应时才会生成
   *              报文，该 ID 可以通过 tx_id 方法获取
   *              2. 只会在报文需要的位置上进行修改，区域位置的内容会保持不变
   */
  virtual State encode(uint8_t tx_data[8], uint32_t tx_msg_std_id) = 0;

  /**
   * @brief       将电调发回的 CAN 报文进行解包
   * @param        rx_data: 电调发回的 CAN 报文
   * @param        rx_msg_std_id: 电调发回的报文的ID
   * @retval       当 ID 匹配时返回 kStateOk，否则返回 kStateIdDismatch
   * @note        只有当 rx_msg_std_id 与实际的电机会发回报文的 ID 对应时才会对报文
   *              进行解包，该 ID 可以通过 rx_id 方法获取
   */
  virtual State decode(const uint8_t rx_data[8], uint32_t rx_msg_std_id) = 0;

  virtual float raw2torq(float raw) const;

  virtual float torq2raw(float torq) const;

  virtual float raw2curr(float raw) const;

  virtual float curr2raw(float curr) const;

  virtual float torq2curr(float torq) const;

  virtual float curr2torq(float curr) const;

  const MotorInfo& motor_info(void) const { return motor_info_; }

  Dir dir(void) const { return motor_info_.dir; }

  uint8_t id(void) const { return motor_info_.id; }

  uint32_t rx_id(void) const { return motor_info_.rx_id; }

  uint32_t tx_id(void) const { return motor_info_.tx_id; }

  AngleRange angle_range(void) const { return motor_info_.angle_range; }

  virtual State set_input_type(InputType input_type);

  InputType get_input_type(void) const { return motor_info_.input_type; }

  float angle(void) const { return angle_; }

  float vel(void) const { return vel_; }

  float torq(void) const { return torq_; }

  float curr(void) const { return curr_; }

 protected:
  /** 电机状态 */
  float angle_ = 0;           ///* 电机输出端角度，单位：rad
  float vel_ = 0;             ///* 电机输出单角速度，单位：rad/s
  float torq_ = 0;            ///* 电机输出端实际力矩，单位：N·m
  float curr_ = 0;            ///* 电机转子实际电流，单位：A
  float round_ = 0;           ///* 电机转子端累计圈数
  float last_raw_angle_ = 0;  ///* 电机上次角度原始值
  /** 电机输出端实际输出角度（未扣除角度偏差），单位：rad */
  float actual_angle_ = 0;

  /** 电机输入 */
  float raw_input_ = 0;   ///* 报文原始输入
  float torq_input_ = 0;  ///* 电机输出端期望力矩
  float curr_input_ = 0;  ///* 电机转子期望电流

  MotorInfo motor_info_;  ///* 电机状态

  static constexpr float kCross0ValueThres = 0.5f;  ///* 过零阈值

  float normAngle(float angle);
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace motor
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_MOTOR_MOTOR_BASE_HPP_ */
