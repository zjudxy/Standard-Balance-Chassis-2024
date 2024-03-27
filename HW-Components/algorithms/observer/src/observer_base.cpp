/**
 *******************************************************************************
 * @file      : observer_base.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2023-12-27      Caikunzhen      1. 完成初版编写（未测试）
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2023 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "observer_base.hpp"

#include "assert.hpp"

namespace hello_world
{
namespace observer
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

Observer::Observer(size_t x_dim, size_t z_dim, size_t u_dim)
    : kXDim_(x_dim), kZDim_(z_dim), kUDim_(u_dim)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(kXDim_ > 0, "x_dim must be greater than 0");
  HW_ASSERT(kZDim_ > 0, "z_dim must be greater than 0");
  HW_ASSERT(kUDim_ > 0, "u_dim must be greater than 0");
#pragma endregion

  x_hat_ = Allocator<float>::allocate(kXDim_);
  memset(x_hat_, 0, kXDim_ * sizeof(float));
}

Observer::~Observer()
{
  Allocator<float>::deallocate(x_hat_, kXDim_);
}

/**
 * @brief       获取最优状态估计量
 * @param        x_hat: 最优状态估计量，大小为kXDim_
 * @retval       None
 * @note        None
 */
void Observer::getX(float* x_hat) const
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(x_hat != nullptr, "x_hat must not be nullptr");
#pragma endregion

  memcpy(x_hat, x_hat_, kXDim_ * sizeof(float));
};
}  // namespace observer
}  // namespace hello_world