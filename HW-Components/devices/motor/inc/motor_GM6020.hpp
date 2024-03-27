/**
 *******************************************************************************
 * @file      : motor_GM6020.hpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2023-11-25      Caikunzhen      1. 未测试版本
 *  V1.0.0      2023-12-04      Caikunzhen      1. 完成测试
 *******************************************************************************
 * @attention : raw2x 与 x2raw 方法只适用于反馈报文中的转换关系，无法用于控制报文
 *******************************************************************************
 *  Copyright (c) 2023 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_MOTOR_MOTOR_GM6020_HPP_
#define HW_COMPONENTS_DEVICES_MOTOR_MOTOR_GM6020_HPP_

/* Includes ------------------------------------------------------------------*/
#include "motor_base.hpp"

namespace hello_world
{
namespace motor
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

class GM6020 : public Motor
{
 public:
  explicit GM6020(
      uint8_t id, const OptionalParams &optinal_params = OptionalParams());
  virtual ~GM6020(){};

  virtual State setInput(float input) override;

  virtual State set_input_type(InputType input_type) override;

  virtual State encode(
      uint8_t tx_data[8], uint32_t tx_msg_std_id) override;

  virtual State decode(
      const uint8_t rx_data[8], uint32_t rx_msg_std_id) override;

  uint8_t temp(void) const { return temp_; }

 private:
  uint8_t temp_;  ///* 电机温度

  static constexpr uint32_t kTx1_4_ = 0x1FF;
  static constexpr uint32_t kTx5_7_ = 0x2FF;
  static constexpr uint32_t kRx0_ = 0x204;
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace motor
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_MOTOR_MOTOR_M3508_HPP_ */
