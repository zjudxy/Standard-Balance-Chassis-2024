/**
 *******************************************************************************
 * @file      : kalman.cpp
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
#include "kalman.hpp"

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
 * @brief       卡尔曼滤波器初始化
 * @param        config_d: 离散系统配置，初始化后可释放
 * @retval       None
 * @note        配置参数中的矩阵需要为离散系统的矩阵
 */
Kalman::Kalman(const Config& config_d)
    : Observer(config_d.x_dim, config_d.z_dim, config_d.u_dim)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(config_d.F != nullptr, "F must not be nullptr");
  HW_ASSERT(config_d.B != nullptr, "B must not be nullptr");
  HW_ASSERT(config_d.H != nullptr, "H must not be nullptr");
  HW_ASSERT(config_d.Q != nullptr, "Q must not be nullptr");
  HW_ASSERT(config_d.R != nullptr, "R must not be nullptr");
#pragma endregion

  F_ = Allocator<float>::allocate(kXDim_ * kXDim_);
  memcpy(F_, config_d.F, kXDim_ * kXDim_ * sizeof(float));
  arm_mat_init_f32(&F_mat_, kXDim_, kXDim_, F_);

  B_ = Allocator<float>::allocate(kXDim_ * kUDim_);
  memcpy(B_, config_d.B, kXDim_ * kUDim_ * sizeof(float));
  arm_mat_init_f32(&B_mat_, kXDim_, kUDim_, B_);

  H_ = Allocator<float>::allocate(kZDim_ * kXDim_);
  memcpy(H_, config_d.H, kZDim_ * kXDim_ * sizeof(float));
  arm_mat_init_f32(&H_mat_, kZDim_, kXDim_, H_);

  Q_ = Allocator<float>::allocate(kXDim_ * kXDim_);
  memcpy(Q_, config_d.Q, kXDim_ * kXDim_ * sizeof(float));
  arm_mat_init_f32(&Q_mat_, kXDim_, kXDim_, Q_);

  R_ = Allocator<float>::allocate(kZDim_ * kZDim_);
  memcpy(R_, config_d.R, kZDim_ * kZDim_ * sizeof(float));
  arm_mat_init_f32(&R_mat_, kZDim_, kZDim_, R_);

  P_ = Allocator<float>::allocate(kXDim_ * kXDim_);
  if (config_d.P != nullptr) {
    memcpy(P_, config_d.P, kXDim_ * kXDim_ * sizeof(float));

  } else {
    memset(P_, 0, kXDim_ * kXDim_ * sizeof(float));
  }
  arm_mat_init_f32(&P_mat_, kXDim_, kXDim_, P_);

  if (config_d.x0 != nullptr) {
    setX0(config_d.x0);
  }
}

Kalman::~Kalman()
{
  Allocator<float>::deallocate(P_, kXDim_ * kXDim_);
  Allocator<float>::deallocate(R_, kZDim_ * kZDim_);
  Allocator<float>::deallocate(Q_, kXDim_ * kXDim_);
  Allocator<float>::deallocate(H_, kZDim_ * kXDim_);
  Allocator<float>::deallocate(B_, kXDim_ * kUDim_);
  Allocator<float>::deallocate(F_, kXDim_ * kXDim_);
}

/**
 * @brief       设置初始状态
 * @param        x0: 初始状态
 * @retval       None
 * @note        None
 */
void Kalman::setX0(const float* x0)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(x0 != nullptr, "x0 must not be nullptr");
#pragma endregion

  memcpy(x_hat_, x0, kXDim_ * sizeof(float));
}

/**
 * @brief       卡尔曼滤波器计算
 * @param        u: 控制量，大小为kUDim_
 * @param        z: 观测量，大小为kZDim_
 * @retval       None
 * @note        None
 */
void Kalman::calc(const float* u, const float* z)
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
 * @brief       卡尔曼滤波器预测
 * @param        u: 控制量，大小为kUDim_
 * @param        x_hat_bar: 返回预测状态量，大小为kXDim_
 * @param        P_bar_mat_ptr: 返回预测状态协方差矩阵，大小为kXDim_ * kXDim_
 * @retval       None
 * @note        None
 */
void Kalman::predict(const float* u, float* x_hat_bar,
                     arm_matrix_instance_f32* P_bar_mat_ptr) const
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(u != nullptr, "u must not be nullptr");
  HW_ASSERT(x_hat_bar != nullptr, "x_hat_bar must not be nullptr");
  HW_ASSERT(P_bar_mat_ptr != nullptr, "P_bar_mat_ptr must not be nullptr");
#pragma endregion

  /* x_hat_bar = F * x_hat_ + B * u */
  {
    float* tmp1 = Allocator<float>::allocate(kXDim_);
    float* tmp2 = Allocator<float>::allocate(kXDim_);
    arm_mat_vec_mult_f32(&F_mat_, x_hat_, tmp1);  // F * x_hat_
    arm_mat_vec_mult_f32(&B_mat_, u, tmp2);       // B * u
    arm_add_f32(tmp1, tmp2, x_hat_bar, kXDim_);   // F * x_hat_ + B * u

    Allocator<float>::deallocate(tmp2, kXDim_);
    Allocator<float>::deallocate(tmp1, kXDim_);
  }

  /* P_bar = F * P * F' + Q */
  {
    float* mat_tmp1 = Allocator<float>::allocate(kXDim_ * kXDim_);
    float* mat_tmp2 = Allocator<float>::allocate(kXDim_ * kXDim_);
    arm_matrix_instance_f32 mat_tmp_mat1, mat_tmp_mat2;
    arm_mat_init_f32(&mat_tmp_mat1, kXDim_, kXDim_, mat_tmp1);
    arm_mat_init_f32(&mat_tmp_mat2, kXDim_, kXDim_, mat_tmp2);

    arm_mat_trans_f32(&F_mat_, &mat_tmp_mat1);                // F'
    arm_mat_mult_f32(&P_mat_, &mat_tmp_mat1, &mat_tmp_mat2);  // P * F'
    arm_mat_mult_f32(&F_mat_, &mat_tmp_mat2, &mat_tmp_mat1);  // F * P * F'
    arm_mat_add_f32(&mat_tmp_mat1, &Q_mat_, P_bar_mat_ptr);   // F * P * F' + Q

    Allocator<float>::deallocate(mat_tmp2, kXDim_ * kXDim_);
    Allocator<float>::deallocate(mat_tmp1, kXDim_ * kXDim_);
  }
}

/**
 * @brief       卡尔曼滤波器校正
 * @param        z: 观测量，大小为kZDim_
 * @param        x_hat_bar: 预测状态量，大小为kXDim_
 * @param        P_bar_mat: 预测状态协方差矩阵，大小为kXDim_ * kXDim_
 * @retval       None
 * @note        None
 */
void Kalman::update(const float* z, const float* x_hat_bar,
                    const arm_matrix_instance_f32& P_bar_mat)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(z != nullptr, "z must not be nullptr");
  HW_ASSERT(x_hat_bar != nullptr, "x_hat_bar must not be nullptr");
#pragma endregion

  float* K = Allocator<float>::allocate(kXDim_ * kZDim_);
  arm_matrix_instance_f32 K_mat;
  arm_mat_init_f32(&K_mat, kXDim_, kZDim_, K);

  /* K = P_bar * H' * (H * P_bar * H' + R)^(-1) */
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

    arm_mat_trans_f32(&H_mat_, &mat_tmp_mat1);                   // H'
    arm_mat_mult_f32(&P_bar_mat, &mat_tmp_mat1, &mat_tmp_mat2);  // P_bar * H'
    arm_mat_mult_f32(&H_mat_, &mat_tmp_mat2, &mat_tmp_mat3);     // H * P_bar * H'
    arm_mat_add_f32(&mat_tmp_mat3, &R_mat_, &mat_tmp_mat4);      // H * P_bar * H' + R
    arm_mat_inverse_f32(&mat_tmp_mat4, &mat_tmp_mat3);           // (H * P_bar * H' + R)^(-1)
    arm_mat_mult_f32(&mat_tmp_mat2, &mat_tmp_mat3, &K_mat);      // P_bar * H' * (H * P_bar * H' + R)^(-1)

    Allocator<float>::deallocate(mat_tmp4, kZDim_ * kZDim_);
    Allocator<float>::deallocate(mat_tmp3, kZDim_ * kZDim_);
    Allocator<float>::deallocate(mat_tmp2, kXDim_ * kZDim_);
    Allocator<float>::deallocate(mat_tmp1, kXDim_ * kZDim_);
  }

  /* x_hat_ = x_hat_bar + K * (z - H * x_hat_bar) */
  {
    float* tmp1 = Allocator<float>::allocate(kZDim_);
    float* tmp2 = Allocator<float>::allocate(kZDim_);
    float* tmp3 = Allocator<float>::allocate(kXDim_);
    arm_mat_vec_mult_f32(&H_mat_, x_hat_bar, tmp1);  // H * x_hat_bar
    arm_sub_f32(z, tmp1, tmp2, kZDim_);              // z - H * x_hat_bar
    arm_mat_vec_mult_f32(&K_mat, tmp2, tmp3);        // K * (z - H * x_hat_bar)
    arm_add_f32(x_hat_bar, tmp3, x_hat_, kXDim_);    // x_hat_bar + K * (z - H * x_hat_bar)

    Allocator<float>::deallocate(tmp3, kXDim_);
    Allocator<float>::deallocate(tmp2, kZDim_);
    Allocator<float>::deallocate(tmp1, kZDim_);
  }

  /* P = (I - K * H) * P_bar */
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

    arm_mat_mult_f32(&K_mat, &H_mat_, &mat_tmp_mat1);       // K * H
    arm_mat_sub_f32(&I_mat, &mat_tmp_mat1, &mat_tmp_mat2);  // I - K * H
    arm_mat_mult_f32(&mat_tmp_mat2, &P_bar_mat, &P_mat_);   // (I - K * H) * P_bar

    Allocator<float>::deallocate(I, kXDim_ * kXDim_);
    Allocator<float>::deallocate(mat_tmp2, kXDim_ * kXDim_);
    Allocator<float>::deallocate(mat_tmp1, kXDim_ * kXDim_);
  }

  Allocator<float>::deallocate(K, kXDim_ * kZDim_);
}
}  // namespace observer
}  // namespace hello_world
