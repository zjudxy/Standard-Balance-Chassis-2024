/**
 *******************************************************************************
 * @file      : motor_DM_J8006.hpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2024-05-01      Caikunzhen      1. 未测试版本
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2023 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_MOTOR_MOTOR_DM_J8006_HPP_
#define HW_COMPONENTS_DEVICES_MOTOR_MOTOR_DM_J8006_HPP_

/* Includes ------------------------------------------------------------------*/
#include "motor_base.hpp"

namespace hello_world
{
namespace motor
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

class DM_J8006 : public Motor
{
 public:
  enum StateCode {
    kStateCodeMotorDisable = 0x0,  ///* 电机失能
    kStateCodeMotorEnable = 0x1,   ///* 电机使能
    kStateCodeOverVolt = 0x8,      ///* 过压
    kStateCodeUnderVolt = 0x9,     ///* 欠压
    kStateCodeOverCurr = 0xA,      ///* 过流
    kStateCodeMosOverTemp = 0xB,   ///* MOS过温
    kStateCodeCoilOverTemp = 0xC,  ///* 电机线圈过温
    kStateCodeCommLoss = 0xD,      ///* 通信丢失
    kStateCodeOverload = 0xE,      ///* 过载
  };

  enum Cmd {
    kCmdNone = 0x0,      ///* 无指令
    kCmdClearErr = 0x1,  ///* 清除错误
    kCmdDisable = 0x2,   ///* 电机失能
    kCmdEnable = 0x4,    ///* 电机使能
  };

  explicit DM_J8006(uint8_t id,
                    const OptionalParams &optinal_params = OptionalParams(),
                    bool auto_enable = true);
  virtual ~DM_J8006(){};

  virtual State setInput(float input) override;

  virtual State set_input_type(InputType input_type) override;

  virtual State encode(
      uint8_t tx_data[8], uint32_t tx_msg_std_id) override;

  virtual State decode(
      const uint8_t rx_data[8], uint32_t rx_msg_std_id) override;

  virtual float raw2torq(float raw) const override;

  virtual float torq2raw(float torq) const override;

  virtual float raw2curr(float raw) const override;

  virtual float curr2raw(float curr) const override;

  uint8_t rotor_temp(void) const { return rotor_temp_; }

  uint8_t MOS_temp(void) const { return MOS_temp_; }

  StateCode error_code(void) const { return state_code_; }

 private:
  uint8_t rotor_temp_;      ///* 电机转子温度
  uint8_t MOS_temp_;        ///* 电机MOS管温度
  Cmd wait_to_handle_cmd_;  ///* 待处理的指令
  bool is_connected_;       ///* 电机是否连接
  bool is_enabled_;         ///* 电机是否使能
  StateCode state_code_;    ///* 状态码
  bool auto_enable_;        ///* 电机自动使能

  static constexpr uint32_t kTx0_ = 0x00;
  static constexpr uint32_t kRx0_ = 0x10;
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace motor
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_MOTOR_MOTOR_DM_J8006_HPP_ */
