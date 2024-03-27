/**
 *******************************************************************************
 * @file      : luenberger.cpp
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
#include "luenberger.hpp"

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

/**
 * @brief       龙伯格观测器初始化（离散）
 * @param        config_d: 离散系统配置，初始化后可释放
 * @retval       None
 * @note        配置参数中的矩阵需要为离散系统的矩阵
 */
Luenberger::Luenberger(const Config& config_d)
    : Observer(config_d.x_dim, config_d.z_dim, config_d.u_dim)
{
/* 变量检查 */
#pragma region
  HW_ASSERT(config_d.F != nullptr, "F must not be nullptr");
  HW_ASSERT(config_d.B != nullptr, "B must not be nullptr");
  HW_ASSERT(config_d.H != nullptr, "H must not be nullptr");
  HW_ASSERT(config_d.L != nullptr, "L must not be nullptr");
#pragma endregion

  Fd_ = Allocator<float>::allocate(kXDim_ * kXDim_);
  memcpy(Fd_, config_d.F, kXDim_ * kXDim_ * sizeof(float));
  arm_mat_init_f32(&Fd_mat_, kXDim_, kXDim_, Fd_);

  Bd_ = Allocator<float>::allocate(kXDim_ * kUDim_);
  memcpy(Bd_, config_d.B, kXDim_ * kUDim_ * sizeof(float));
  arm_mat_init_f32(&Bd_mat_, kXDim_, kUDim_, Bd_);

  Hd_ = Allocator<float>::allocate(kZDim_ * kXDim_);
  memcpy(Hd_, config_d.H, kZDim_ * kXDim_ * sizeof(float));
  arm_mat_init_f32(&Hd_mat_, kZDim_, kXDim_, Hd_);

  Ld_ = Allocator<float>::allocate(kXDim_ * kZDim_);
  memcpy(Ld_, config_d.L, kXDim_ * kZDim_ * sizeof(float));
  arm_mat_init_f32(&Ld_mat_, kXDim_, kZDim_, Ld_);

  if (config_d.x0 != nullptr) {
    setX0(config_d.x0);
  }
}

/**
 * @brief       龙伯格观测器初始化（连续）
 * @param        config_c: 连续系统配置，初始化后可释放
 * @param        Ts: 离散化时间间隔
 * @retval       None
 * @note        配置参数中的矩阵需要为连续系统的矩阵，使用前向欧拉法离散化
 */
Luenberger::Luenberger(const Config& config_c, float Ts)
    : Observer(config_c.x_dim, config_c.z_dim, config_c.u_dim)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(config_c.F != nullptr, "F must not be nullptr");
  HW_ASSERT(config_c.B != nullptr, "B must not be nullptr");
  HW_ASSERT(config_c.H != nullptr, "H must not be nullptr");
  HW_ASSERT(config_c.L != nullptr, "L must not be nullptr");
#pragma endregion

  /* 系统矩阵离散化 */
  Fd_ = Allocator<float>::allocate(kXDim_ * kXDim_);
  {
    arm_matrix_instance_f32 Fc_mat;
    arm_mat_init_f32(&Fc_mat, kXDim_, kXDim_, config_c.F);

    float* I = Allocator<float>::allocate(kXDim_ * kXDim_);
    memset(I, 0, kXDim_ * kXDim_ * sizeof(float));
    for (size_t i = 0; i < kXDim_; i++) {
      I[i * kXDim_ + i] = 1.0f;
    }
    arm_matrix_instance_f32 I_mat;
    arm_mat_init_f32(&I_mat, kXDim_, kXDim_, I);

    float* tmp = Allocator<float>::allocate(kXDim_ * kXDim_);
    arm_matrix_instance_f32 tmp_mat;
    arm_mat_init_f32(&tmp_mat, kXDim_, kXDim_, tmp);
    arm_mat_init_f32(&Fd_mat_, kXDim_, kXDim_, Fd_);
    arm_mat_scale_f32(&Fc_mat, Ts, &tmp_mat);
    arm_mat_add_f32(&tmp_mat, &I_mat, &Fd_mat_);

    Allocator<float>::deallocate(tmp, kXDim_ * kXDim_);
    Allocator<float>::deallocate(I, kXDim_ * kXDim_);
  }

  /* 控制矩阵离散化 */
  Bd_ = Allocator<float>::allocate(kXDim_ * kUDim_);
  {
    arm_matrix_instance_f32 Bc_mat;
    arm_mat_init_f32(&Bc_mat, kXDim_, kUDim_, config_c.B);

    arm_mat_init_f32(&Bd_mat_, kXDim_, kUDim_, Bd_);
    arm_mat_scale_f32(&Bc_mat, Ts, &Bd_mat_);
  }

  /* 观测矩阵离散化 */
  Hd_ = Allocator<float>::allocate(kZDim_ * kXDim_);
  memcpy(Hd_, config_c.H, kZDim_ * kXDim_ * sizeof(float));
  arm_mat_init_f32(&Hd_mat_, kZDim_, kXDim_, Hd_);

  /* 龙伯格增益矩阵离散化 */
  Ld_ = Allocator<float>::allocate(kXDim_ * kZDim_);
  {
    arm_matrix_instance_f32 Lc_mat;
    arm_mat_init_f32(&Lc_mat, kXDim_, kZDim_, config_c.L);

    arm_mat_init_f32(&Ld_mat_, kXDim_, kZDim_, Ld_);
    arm_mat_scale_f32(&Lc_mat, Ts, &Ld_mat_);
  }

  if (config_c.x0 != nullptr) {
    setX0(config_c.x0);
  }
}

Luenberger::~Luenberger()
{
  Allocator<float>::deallocate(Ld_, kXDim_ * kZDim_);
  Allocator<float>::deallocate(Hd_, kZDim_ * kXDim_);
  Allocator<float>::deallocate(Bd_, kXDim_ * kUDim_);
  Allocator<float>::deallocate(Fd_, kXDim_ * kXDim_);
}

/**
 * @brief       观测计算
 * @param        u: 控制量，大小为kUDim_
 * @param        z: 观测量，大小为kZDim_
 * @retval       None
 * @note        None
 */
void Luenberger::calc(const float* u, const float* z)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(u != nullptr, "u must not be nullptr");
  HW_ASSERT(z != nullptr, "z must not be nullptr");
#pragma endregion

  float* x_hat_bar = Allocator<float>::allocate(kXDim_);
  float* tmp1 = Allocator<float>::allocate(kXDim_);
  float* tmp2 = Allocator<float>::allocate(kXDim_);

  /* 预测 */
  /* x_hat_bar = Fd * x_hat_ + Bd * u */
  arm_mat_vec_mult_f32(&Fd_mat_, x_hat_, tmp1);
  arm_mat_vec_mult_f32(&Bd_mat_, u, tmp2);
  arm_add_f32(tmp1, tmp2, x_hat_bar, kXDim_);

  /* 校正 */
  /* x_hat_ = x_hat_bar + Ld * (z - Hd * x_hat_bar) */
  float* z_hat_bar = Allocator<float>::allocate(kZDim_);
  float* err = Allocator<float>::allocate(kZDim_);
  arm_mat_vec_mult_f32(&Hd_mat_, x_hat_bar, z_hat_bar);
  arm_sub_f32(z, z_hat_bar, err, kZDim_);
  arm_mat_vec_mult_f32(&Ld_mat_, err, tmp1);
  arm_add_f32(x_hat_bar, tmp1, x_hat_, kXDim_);

  Allocator<float>::deallocate(err, kZDim_);
  Allocator<float>::deallocate(z_hat_bar, kZDim_);
  Allocator<float>::deallocate(tmp2, kXDim_);
  Allocator<float>::deallocate(tmp1, kXDim_);
  Allocator<float>::deallocate(x_hat_bar, kXDim_);
}

/**
 * @brief       设置初始状态
 * @param        x: 初始状态，大小为kXDim_
 * @retval       None
 * @note        None
 */
void Luenberger::setX0(const float* x)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(x != nullptr, "x must not be nullptr");
#pragma endregion

  memcpy(x_hat_, x, kXDim_ * sizeof(float));
}

}  // namespace observer
}  // namespace hello_world
