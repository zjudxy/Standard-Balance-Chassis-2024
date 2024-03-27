/**
 *******************************************************************************
 * @file      : motor_A1.hpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2023-11-25      Caikunzhen      1. 未测试版本
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2023 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_MOTOR_MOTOR_A1_HPP_
#define HW_COMPONENTS_DEVICES_MOTOR_MOTOR_A1_HPP_

/* Includes ------------------------------------------------------------------*/
#include "motor_base.hpp"

namespace hello_world
{
namespace motor
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

class A1 : public Motor
{
 public:
  explicit A1(
      uint8_t id, const OptionalParams &optinal_params = OptionalParams());
  virtual ~A1(){};

  virtual State encode(
      uint8_t tx_data[8], uint32_t tx_msg_std_id) override;

  virtual State decode(
      const uint8_t rx_data[8], uint32_t rx_msg_std_id) override;

  uint8_t temp(void) const { return temp_; }

 private:
  uint8_t temp_;  ///* 电机温度

  static constexpr uint32_t kTx1_3_ = 0x400;
  static constexpr uint32_t kTx4_6_ = 0x3FF;
  static constexpr uint32_t kRx0_ = 0x400;
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace motor
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_MOTOR_MOTOR_A1_HPP_ */
