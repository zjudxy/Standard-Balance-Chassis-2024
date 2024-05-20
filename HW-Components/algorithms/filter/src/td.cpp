/**
 *******************************************************************************
 * @file      : td.cpp
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
#include "td.hpp"

#include <cstring>

#include "assert.hpp"
#include "base.hpp"

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
 * @brief       微分跟踪器初始化
 * @param        r: 截止频率，单位：Hz
 * @param        Ts: 采样周期，单位：s
 * @param        period: 数据周期，大于0时表示周期性数据
 * @param        dim: 输入输出维度
 * @retval       None
 * @note        None
 */
Td::Td(float r, float Ts, float period, size_t dim)
    : Filter(dim), kR_(r), kTs_(Ts), kPeriod_(period)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(dim > 0, "dim must be greater than 0");
  HW_ASSERT(r > 0, "r must be greater than 0");
  HW_ASSERT(Ts > 0, "Ts must be greater than 0");
#pragma endregion

  x_ls_ptrs_[0] = Allocator<float>::allocate(kDim_);
  x_ls_ptrs_[1] = Allocator<float>::allocate(kDim_);
}

Td::~Td()
{
  Allocator<float>::deallocate(x_ls_ptrs_[0], kDim_);
  Allocator<float>::deallocate(x_ls_ptrs_[1], kDim_);
}

/**
 * @brief       计算输入数据的微分
 * @param        in_ls: 输入数据
 * @param        out_ls: 输出数据
 * @note        None
 */
void Td::calc(const float* in_ls, float* out_ls)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(in_ls != nullptr, "in must not be nullptr");
  HW_ASSERT(out_ls != nullptr, "out must not be nullptr");
#pragma endregion

  float diff_x = 0;
  for (size_t i = 0; i < kDim_; i++) {
    x_ls_ptrs_[0][i] = x_ls_ptrs_[0][i] + kTs_ * x_ls_ptrs_[1][i];

    if (kPeriod_ > 0) {
      x_ls_ptrs_[0][i] = NormPeriodData(0, kPeriod_, x_ls_ptrs_[0][i]);
      diff_x = PeriodDataSub(x_ls_ptrs_[0][i], in_ls[i], kPeriod_);
    } else {
      diff_x = x_ls_ptrs_[0][i] - in_ls[i];
    }

    x_ls_ptrs_[1][i] =
        x_ls_ptrs_[1][i] -
        kTs_ * (kR_ * kR_ * diff_x + 2 * kR_ * x_ls_ptrs_[1][i]);
  }

  memcpy(out_ls, x_ls_ptrs_[1], kDim_ * sizeof(float));
}

/**
 * @brief       重置
 * @param        None
 * @retval       None
 * @note        None
 */
void Td::reset()
{
  memset(x_ls_ptrs_[0], 0, kDim_ * sizeof(float));
  memset(x_ls_ptrs_[1], 0, kDim_ * sizeof(float));
}
}  // namespace filter
}  // namespace hello_world