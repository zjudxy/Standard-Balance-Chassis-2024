/**
 *******************************************************************************
 * @file      : queue.hpp
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
#ifndef HW_COMPONENTS_ALGORITHMS_FILTER_QUEUE_HPP_
#define HW_COMPONENTS_ALGORITHMS_FILTER_QUEUE_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstddef>
#include <cstring>

#include "allocator.hpp"
#include "assert.hpp"

namespace hello_world
{
namespace filter
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

class Queue : public MemMang
{
 public:
  explicit Queue(size_t len) : kLen_(len), start_idx_(0)
  {
    data_ls = Allocator<float>::allocate(kLen_);
    memset(data_ls, 0, sizeof(float) * kLen_);
  };

  virtual ~Queue(void)
  {
    Allocator<float>::deallocate(data_ls, kLen_);
  };

  /**
   * @brief       入队
   * @param        in: 输入数据
   * @retval       None
   * @note        None
   */
  void push(float in)
  {
    data_ls[start_idx_] = in;
    start_idx_ = (start_idx_ + 1) % kLen_;
  };

  /**
   * @brief       重置
   * @retval       None
   * @note        None
   */
  void reset(void)
  {
    start_idx_ = 0;
    memset(data_ls, 0, sizeof(float) * kLen_);
  };

  /**
   * @brief       获取队列中的数据
   * @param        idx: 数据索引
   * @retval       队列中的数据
   * @note        None
   */
  float at(size_t idx)
  {
/* 变量检查 */
#pragma region
    HW_ASSERT(idx < kLen_, "idx must be less than %d", kLen_);
#pragma endregion

    return data_ls[(start_idx_ + idx) % kLen_];
  };

  const size_t kLen_;  ///< 队列长度

 private:
  float* data_ls;
  size_t start_idx_;  ///< 队列起始索引
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace filter
}  // namespace hello_world

#endif /* HW_COMPONENTS_ALGORITHMS_FILTER_QUEUE_HPP_ */
