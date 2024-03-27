/**
 *******************************************************************************
 * @file      : motor_M2006.hpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2023-11-25      Caikunzhen      1. 未测试版本
 *  V1.0.0      2023-12-04      Caikunzhen      1. 完成测试
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2023 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_MOTOR_MOTOR_M2006_HPP_
#define HW_COMPONENTS_DEVICES_MOTOR_MOTOR_M2006_HPP_

/* Includes ------------------------------------------------------------------*/
#include "motor_base.hpp"

namespace hello_world
{
namespace motor
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

class M2006 : public Motor
{
 public:
  explicit M2006(
      uint8_t id, const OptionalParams &optinal_params = OptionalParams());
  virtual ~M2006(){};

  virtual State encode(
      uint8_t tx_data[8], uint32_t tx_msg_std_id) override;

  virtual State decode(
      const uint8_t rx_data[8], uint32_t rx_msg_std_id) override;

 private:
  static constexpr uint32_t kTx1_4_ = 0x200;
  static constexpr uint32_t kTx5_8_ = 0x1FF;
  static constexpr uint32_t kRx0_ = 0x200;
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace motor
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_MOTOR_MOTOR_M2006_HPP_ */
