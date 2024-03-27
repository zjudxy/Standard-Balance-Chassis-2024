/**
 *******************************************************************************
 * @file      : iir.cpp
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
#include "iir.hpp"

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
 * @brief       IIR滤波器初始化
 * @param        b_ls: 前向系数，[b0, b1, b2, ..., bM], 注意大小为M+1
 * @param        a_ls: 反馈系数，[a1, a2, ..., aN], 注意大小为N
 * @param        M: 前向系数阶数（>=0）
 * @param        N: 反馈系数阶数（>0）
 * @param        dim: 输入输出维度
 * @retval       None
 * @note        注意M<=N，
 *                       b0 + b1*z^(-1) + b2*z^(-2) + ... + bM*z^(-M)
 *              H(z) = ----------------------------------------------
 *                       1 + a1*z^(-1) + a2*z^(-2) + ... + aN*z^(-N)
 */
Iir::Iir(const float* b_ls, const float* a_ls, size_t M, size_t N, size_t dim)
    : Filter(dim), kM_(M), kN_(N)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(dim > 0, "dim must be greater than 0");
  HW_ASSERT(M >= 0, "M must be greater than or equal to 0");
  HW_ASSERT(N > 0, "N must be greater than 0");
  HW_ASSERT(M <= N, "M must be less than or equal to N");
#pragma endregion

  x_ls_ptrs_ = Allocator<Queue>::allocate(kDim_);
  y_ls_ptrs_ = Allocator<Queue>::allocate(kDim_);
  for (size_t i = 0; i < kDim_; i++) {
    Allocator<Queue>::construct(x_ls_ptrs_ + i, kM_ + 1);
    Allocator<Queue>::construct(y_ls_ptrs_ + i, kN_ + 1);
  }

  b_ls_ = Allocator<float>::allocate(kM_ + 1);
  a_ls_ = Allocator<float>::allocate(kN_);
  memcpy(b_ls_, b_ls, sizeof(float) * (kM_ + 1));
  memcpy(a_ls_, a_ls, sizeof(float) * kN_);
}

Iir::~Iir()
{
  delete[] x_ls_ptrs_;
  delete[] y_ls_ptrs_;
  Allocator<float>::deallocate(b_ls_, kM_ + 1);
  Allocator<float>::deallocate(a_ls_, kN_);
}

/**
 * @brief       计算输入数据的滤波结果
 * @param        in_ls: 输入数据
 * @param        out_ls: 输出数据
 * @note        None
 */
void Iir::calc(const float* in_ls, float* out_ls)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(in_ls != nullptr, "in must not be nullptr");
  HW_ASSERT(out_ls != nullptr, "out must not be nullptr");
#pragma endregion

  for (size_t i = 0; i < kDim_; i++) {
    float yn = 0;
    x_ls_ptrs_[i].push(in_ls[i]);
    for (size_t j = 0; j < kM_ + 1; j++) {
      yn += b_ls_[j] * x_ls_ptrs_[i].at(kM_ - j);
    }
    for (size_t j = 0; j < kN_; j++) {
      yn -= a_ls_[j] * y_ls_ptrs_[i].at(kN_ - j);
    }

    y_ls_ptrs_[i].push(yn);
    out_ls[i] = yn;
  }
}

/**
 * @brief       重置
 * @param        None
 * @retval       None
 * @note        None
 */
void Iir::reset()
{
  for (size_t i = 0; i < kDim_; i++) {
    x_ls_ptrs_[i].reset();
    y_ls_ptrs_[i].reset();
  }
}
}  // namespace filter
}  // namespace hello_world