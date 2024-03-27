/**
 *******************************************************************************
 * @file      : ekf.cpp
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
#include "ekf.hpp"

#include <cstring>

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
 * @brief       扩展卡尔曼滤波器初始化
 * @param        config: 离散系统配置，初始化后可释放
 * @retval       None
 * @note        配置参数中的矩阵需要为离散系统的矩阵
 */
Ekf::Ekf(const Config& config)
    : Observer(config.x_dim, config.z_dim, config.u_dim),
      f_(config.f),
      h_(config.h),
      GetJ_F_(config.GetJ_F),
      GetJ_H_(config.GetJ_H)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(config.f != nullptr, "f must not be nullptr");
  HW_ASSERT(config.h != nullptr, "h must not be nullptr");
  HW_ASSERT(config.GetJ_F != nullptr, "GetJ_F must not be nullptr");
  HW_ASSERT(config.GetJ_H != nullptr, "GetJ_H must not be nullptr");
  HW_ASSERT(config.Q != nullptr, "Q must not be nullptr");
  HW_ASSERT(config.R != nullptr, "R must not be nullptr");
#pragma endregion
  Q_ = Allocator<float>::allocate(kXDim_ * kXDim_);
  memcpy(Q_, config.Q, kXDim_ * kXDim_ * sizeof(float));
  arm_mat_init_f32(&Q_mat_, kXDim_, kXDim_, Q_);

  R_ = Allocator<float>::allocate(kZDim_ * kZDim_);
  memcpy(R_, config.R, kZDim_ * kZDim_ * sizeof(float));
  arm_mat_init_f32(&R_mat_, kZDim_, kZDim_, R_);

  P_ = Allocator<float>::allocate(kXDim_ * kXDim_);
  if (config.P != nullptr) {
    memcpy(P_, config.P, kXDim_ * kXDim_ * sizeof(float));
  } else {
    memset(P_, 0, kXDim_ * kXDim_ * sizeof(float));
  }
  arm_mat_init_f32(&P_mat_, kXDim_, kXDim_, P_);

  if (config.x0 != nullptr) {
    setX0(config.x0);
  }
}

Ekf::~Ekf(void)
{
  Allocator<float>::deallocate(Q_, kXDim_);
  Allocator<float>::deallocate(R_, kZDim_);
  Allocator<float>::deallocate(P_, kXDim_);
}

/**
 * @brief       扩展卡尔曼滤波器计算
 * @param        u: 控制量，大小为kUDim_
 * @param        z: 观测量，大小为kZDim_
 * @retval       None
 * @note        None
 */
void Ekf::calc(const float* u, const float* z)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(u != nullptr, "u must not be nullptr");
  HW_ASSERT(z != nullptr, "z must not be nullptr");
#pragma endregion

  float* x_hat_bar = Allocator<float>::allocate(kXDim_);
  float* P_bar = Allocator<float>::allocate(kXDim_ * kXDim_);
  arm_matrix_instance_f32 P_bar_mat;
  arm_mat_init_f32(&P_bar_mat, kXDim_, kXDim_, P_bar);

  /* 预测 */
  predict(u, x_hat_bar, &P_bar_mat);

  /* 校正 */
  update(z, x_hat_bar, P_bar_mat);

  Allocator<float>::deallocate(P_bar, kXDim_ * kXDim_);
  Allocator<float>::deallocate(x_hat_bar, kXDim_);
}

/**
 * @brief       设置初始状态
 * @param        x0: 初始状态
 * @retval       None
 * @note        None
 */
void Ekf::setX0(const float* x0)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(x0 != nullptr, "x0 must not be nullptr");
#pragma endregion

  memcpy(x_hat_, x0, kXDim_ * sizeof(float));
}

/**
 * @brief       扩展卡尔曼滤波器预测
 * @param        u: 控制量，大小为kUDim_
 * @param        x_hat_bar: 返回预测状态量，大小为kXDim_
 * @param        P_bar_mat_ptr: 返回预测状态协方差矩阵，大小为kXDim_*kXDim_
 * @retval       None
 * @note        None
 */
void Ekf::predict(const float* u, float* x_hat_bar,
                  arm_matrix_instance_f32* P_bar_mat_ptr) const
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(u != nullptr, "u must not be nullptr");
  HW_ASSERT(x_hat_bar != nullptr, "x_hat_bar must not be nullptr");
  HW_ASSERT(P_bar_mat_ptr != nullptr, "P_bar_mat_ptr must not be nullptr");
#pragma endregion

  /* x_hat_bar = f(x_hat_, u) */
  f_(x_hat_, u, x_hat_bar);

  /* P_bar = J_F * P * J_F' + Q */
  float* J_F = Allocator<float>::allocate(kXDim_ * kXDim_);
  float* mat_tmp1 = Allocator<float>::allocate(kXDim_ * kXDim_);
  float* mat_tmp2 = Allocator<float>::allocate(kXDim_ * kXDim_);
  GetJ_F_(x_hat_, u, J_F);
  arm_matrix_instance_f32 J_F_mat, mat_tmp_mat1, mat_tmp_mat2;
  arm_mat_init_f32(&J_F_mat, kXDim_, kXDim_, J_F);
  arm_mat_init_f32(&mat_tmp_mat1, kXDim_, kXDim_, mat_tmp1);
  arm_mat_init_f32(&mat_tmp_mat2, kXDim_, kXDim_, mat_tmp2);

  arm_mat_trans_f32(&J_F_mat, &mat_tmp_mat1);                // J_F'
  arm_mat_mult_f32(&P_mat_, &mat_tmp_mat1, &mat_tmp_mat2);   // P * J_F'
  arm_mat_mult_f32(&J_F_mat, &mat_tmp_mat2, &mat_tmp_mat1);  // J_F * P * J_F'
  arm_mat_add_f32(&mat_tmp_mat1, &Q_mat_, P_bar_mat_ptr);    // J_F * P * J_F' + Q

  Allocator<float>::deallocate(mat_tmp2, kXDim_ * kXDim_);
  Allocator<float>::deallocate(mat_tmp1, kXDim_ * kXDim_);
  Allocator<float>::deallocate(J_F, kXDim_ * kXDim_);
}

/**
 * @brief       扩展卡尔曼滤波器校正
 * @param        z: 观测量，大小为kZDim_
 * @param        x_hat_bar: 预测状态量，大小为kXDim_
 * @param        P_bar_mat: 预测状态协方差矩阵，大小为kXDim_*kXDim_
 * @retval       None
 * @note        None
 */
void Ekf::update(const float* z, const float* x_hat_bar,
                 const arm_matrix_instance_f32& P_bar_mat)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(z != nullptr, "z must not be nullptr");
  HW_ASSERT(x_hat_bar != nullptr, "x_hat_bar must not be nullptr");
#pragma endregion

  float* K = Allocator<float>::allocate(kXDim_ * kZDim_);
  float* J_H = Allocator<float>::allocate(kZDim_ * kXDim_);
  GetJ_H_(x_hat_bar, J_H);
  arm_matrix_instance_f32 K_mat, J_H_mat;
  arm_mat_init_f32(&K_mat, kXDim_, kZDim_, K);
  arm_mat_init_f32(&J_H_mat, kZDim_, kXDim_, J_H);

  /* K = P_bar * J_H' * (J_H * P_bar * J_H' + R)^(-1) */
  {
    float* mat_tmp1 = Allocator<float>::allocate(kXDim_ * kZDim_);
    float* mat_tmp2 = Allocator<float>::allocate(kXDim_ * kZDim_);
    float* mat_tmp3 = Allocator<float>::allocate(kZDim_ * kZDim_);
    float* mat_tmp4 = Allocator<float>::allocate(kZDim_ * kZDim_);
    arm_matrix_instance_f32 mat_tmp_mat1, mat_tmp_mat2, mat_tmp_mat3,
        mat_tmp_mat4;
    arm_mat_init_f32(&mat_tmp_mat1, kXDim_, kZDim_, mat_tmp1);
    arm_mat_init_f32(&mat_tmp_mat2, kXDim_, kZDim_, mat_tmp2);
    arm_mat_init_f32(&mat_tmp_mat3, kZDim_, kZDim_, mat_tmp3);
    arm_mat_init_f32(&mat_tmp_mat4, kZDim_, kZDim_, mat_tmp4);

    arm_mat_trans_f32(&J_H_mat, &mat_tmp_mat1);                  // J_H'
    arm_mat_mult_f32(&P_bar_mat, &mat_tmp_mat1, &mat_tmp_mat2);  // P_bar * J_H'
    arm_mat_mult_f32(&J_H_mat, &mat_tmp_mat2, &mat_tmp_mat3);    // J_H * P_bar * J_H'
    arm_mat_add_f32(&mat_tmp_mat3, &R_mat_, &mat_tmp_mat4);      // J_H * P_bar * J_H' + R
    arm_mat_inverse_f32(&mat_tmp_mat4, &mat_tmp_mat3);           // (J_H * P_bar * J_H' + R)^(-1)
    arm_mat_mult_f32(&mat_tmp_mat2, &mat_tmp_mat3, &K_mat);      // P_bar * J_H' * (J_H * P_bar * J_H' + R)^(-1)

    Allocator<float>::deallocate(mat_tmp4, kZDim_ * kZDim_);
    Allocator<float>::deallocate(mat_tmp3, kZDim_ * kZDim_);
    Allocator<float>::deallocate(mat_tmp2, kXDim_ * kZDim_);
    Allocator<float>::deallocate(mat_tmp1, kXDim_ * kZDim_);
  }

  /* x_hat_ = x_hat_bar + K * (z - h(x_hat_bar)) */
  {
    float* z_hat = Allocator<float>::allocate(kZDim_);
    float* tmp1 = Allocator<float>::allocate(kZDim_);
    float* tmp2 = Allocator<float>::allocate(kXDim_);
    h_(x_hat_bar, z_hat);                          // h(x_hat_bar)
    arm_sub_f32(z, z_hat, tmp1, kZDim_);           // z - h(x_hat_bar)
    arm_mat_vec_mult_f32(&K_mat, tmp1, tmp2);      // K * (z - h(x_hat_bar))
    arm_add_f32(x_hat_bar, tmp2, x_hat_, kXDim_);  // x_hat_bar + K * (z - h(x_hat_bar))

    Allocator<float>::deallocate(tmp2, kXDim_);
    Allocator<float>::deallocate(tmp1, kZDim_);
    Allocator<float>::deallocate(z_hat, kZDim_);
  }

  /* P = (I - K * J_H) * P_bar */
  {
    float* mat_tmp1 = Allocator<float>::allocate(kXDim_ * kXDim_);
    float* mat_tmp2 = Allocator<float>::allocate(kXDim_ * kXDim_);
    float* I = Allocator<float>::allocate(kXDim_ * kXDim_);
    memset(I, 0, kXDim_ * kXDim_ * sizeof(float));
    for (size_t i = 0; i < kXDim_; i++) {
      I[i * kXDim_ + i] = 1.0f;
    }

    arm_matrix_instance_f32 mat_tmp_mat1, mat_tmp_mat2, I_mat;
    arm_mat_init_f32(&mat_tmp_mat1, kXDim_, kXDim_, mat_tmp1);
    arm_mat_init_f32(&mat_tmp_mat2, kXDim_, kXDim_, mat_tmp2);
    arm_mat_init_f32(&I_mat, kXDim_, kXDim_, I);

    arm_mat_mult_f32(&K_mat, &J_H_mat, &mat_tmp_mat1);      // K * J_H
    arm_mat_sub_f32(&I_mat, &mat_tmp_mat1, &mat_tmp_mat2);  // I - K * J_H
    arm_mat_mult_f32(&mat_tmp_mat2, &P_bar_mat, &P_mat_);   // (I - K * J_H) * P_bar

    Allocator<float>::deallocate(I, kXDim_ * kXDim_);
    Allocator<float>::deallocate(mat_tmp2, kXDim_ * kXDim_);
    Allocator<float>::deallocate(mat_tmp1, kXDim_ * kXDim_);
  }

  Allocator<float>::deallocate(J_H, kZDim_ * kXDim_);
  Allocator<float>::deallocate(K, kXDim_ * kZDim_);
}
}  // namespace observer
}  // namespace hello_world