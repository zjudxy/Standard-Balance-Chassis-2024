/**
 *******************************************************************************
 * @file      : td.hpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2023-12-25      CaiKunzhen      1. 完成初版编写
 *  V1.0.0      2023-12-30      CaiKunzhen      1. 完成初版测试
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2023 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_ALGORITHMS_FILTER_TD_HPP_
#define HW_COMPONENTS_ALGORITHMS_FILTER_TD_HPP_

/* Includes ------------------------------------------------------------------*/
#include "filter_base.hpp"

namespace hello_world
{
namespace filter
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

class Td : public Filter
{
 public:
  Td(float r, float Ts, float period = 0.0f, size_t dim = 1);
  virtual ~Td();

  virtual void calc(const float* in_ls, float* out_ls) override;

  virtual void reset(void) override;

  const float kR_;   ///< 截止频率，单位：Hz
  const float kTs_;  ///< 采样周期，单位：s
  const float kPeriod_;  ///< 数据周期，大于0时表示周期性数据

 private:
  float* x_ls_ptrs_[2];
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace filter
}  // namespace hello_world

#endif /* HW_COMPONENTS_ALGORITHMS_FILTER_TD_HPP_ */
