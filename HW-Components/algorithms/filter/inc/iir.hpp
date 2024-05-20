/**
 *******************************************************************************
 * @file      : iir.hpp
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
#ifndef HW_COMPONENTS_ALGORITHMS_FILTER_IIR_HPP_
#define HW_COMPONENTS_ALGORITHMS_FILTER_IIR_HPP_

/* Includes ------------------------------------------------------------------*/
#include "filter_base.hpp"

namespace hello_world
{
namespace filter
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

class Queue;
class Iir : public Filter
{
 public:
  Iir(const float* b_ls, const float* a_ls, size_t M, size_t N, size_t dim = 1);
  virtual ~Iir();

  virtual void calc(const float* in_ls, float* out_ls) override;

  virtual void reset(void) override;

  const size_t kM_;  ///< 前向系数阶数
  const size_t kN_;  ///< 反馈系数阶数

 private:
  Queue* x_ls_ptrs_;
  Queue* y_ls_ptrs_;
  float* b_ls_;
  float* a_ls_;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace filter
}  // namespace hello_world

#endif /* HW_COMPONENTS_ALGORITHMS_FILTER_IIR_HPP_ */
