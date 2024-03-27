/**
 *******************************************************************************
 * @file      : filter_base.hpp
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
#ifndef HW_COMPONETS_ALGORITHMS_FILTER_FILTER_BASE_HPP_
#define HW_COMPONETS_ALGORITHMS_FILTER_FILTER_BASE_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstddef>

#include "allocator.hpp"

namespace hello_world
{
namespace filter
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

class Filter : public MemMang
{
 public:
  Filter(size_t dim) : kDim_(dim){};
  virtual ~Filter() = default;

  /**
   * @brief       滤波计算
   * @param        in_ls: 输入数据
   * @param        out_ls: 输出数据
   * @retval       None
   * @note        None
   */
  virtual void calc(const float* in_ls, float* out_ls) = 0;

  /**
   * @brief       重置
   * @param        None
   * @retval       None
   * @note        None
   */
  virtual void reset(void) = 0;

  const size_t kDim_;  ///< 输入输出维度
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace filter
}  // namespace hello_world

#endif /* HW_COMPONETS_ALGORITHMS_FILTER_FILTER_BASE_HPP_ */
