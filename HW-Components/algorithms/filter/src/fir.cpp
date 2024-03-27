/**
 *******************************************************************************
 * @file      : fir.cpp
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
/* Includes ------------------------------------------------------------------*/
#include "fir.hpp"

#include "assert.hpp"
#include "queue.hpp"

namespace hello_world
{
namespace filter
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
 * @brief       FIR滤波器初始化
 * @param        h_ls: 前向系数，[h0, h1, h2, ..., hM], 注意大小为M+1
 * @param        M: 前向系数阶数（>0）
 * @param        dim: 输入输出维度
 * @retval       None
 * @note        H(z) = h0 + h1*z^(-1) + h2*z^(-2) + ... + hM*z^(-M)
 */
Fir::Fir(const float* h_ls, size_t M, size_t dim) : Filter(dim), kM_(M)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(dim > 0, "dim must be greater than 0");
  HW_ASSERT(M > 0, "M must be greater than 0");
#pragma endregion

  x_ls_ptrs_ = Allocator<Queue>::allocate(kDim_);
  for (size_t i = 0; i < kDim_; i++) {
    Allocator<Queue>::construct(x_ls_ptrs_ + i, kM_ + 1);
  }

  h_ls_ = Allocator<float>::allocate(kM_ + 1);
  memcpy(h_ls_, h_ls, sizeof(float) * (kM_ + 1));
}

Fir::~Fir()
{
  delete[] x_ls_ptrs_;
  Allocator<float>::deallocate(h_ls_, kM_ + 1);
}

/**
 * @brief       计算输入数据的滤波结果
 * @param        in_ls: 输入数据
 * @param        out_ls: 输出数据
 * @note        None
 */
void Fir::calc(const float* in_ls, float* out_ls)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(in_ls != nullptr, "in must not be nullptr");
  HW_ASSERT(out_ls != nullptr, "out must not be nullptr");
#pragma endregion

  for (size_t i = 0; i < kDim_; i++) {
    out_ls[i] = 0;
    x_ls_ptrs_[i].push(in_ls[i]);
    for (size_t j = 0; j < kM_ + 1; j++) {
      out_ls[i] += h_ls_[j] * x_ls_ptrs_[i].at(kM_ - j);
    }
  }
}

/**
 * @brief       重置
 * @param        None
 * @retval       None
 * @note        None
 */
void Fir::reset()
{
  for (size_t i = 0; i < kDim_; i++) {
    x_ls_ptrs_[i].reset();
  }
}
}  // namespace filter
}  // namespace hello_world