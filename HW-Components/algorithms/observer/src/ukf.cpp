/**
 *******************************************************************************
 * @file      : ukf.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2023-12-29      Caikunzhen      1. 完成初版编写（未测试）
 *  V1.0.0      2024-01-03      Caikunzhen      1. 完成测试
 *******************************************************************************
 * @attention : https://blog.csdn.net/qq_41011336/article/details/84401691
 *******************************************************************************
 *  Copyright (c) 2023 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ukf.hpp"

#include <cstring>

#include "arm_math.h"
#include "assert.hpp"

namespace hello_world
{
namespace observer
{
/* Private macro -------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

static void CholeskyDecomposition(size_t n, float* A);

Ukf::Ukf(const Config& config)
    : Observer(config.x_dim, config.z_dim, config.u_dim),
      kQDim_(config.Q_dim),
      kRDim_(config.R_dim),
      kNa_(kXDim_ + kQDim_ + kRDim_),
      kAlpha_(config.alpha),
      kBeta_(config.beta),
      kKappa_(config.kappa),
      kLamda_(kAlpha_ * kAlpha_ * (kNa_ + kKappa_) - kNa_),
      f_(config.f),
      h_(config.h)
{
  /* 变量检查 */
#pragma reginon
  HW_ASSERT(config.Q_dim > 0, "Q_dim must be greater than 0");
  HW_ASSERT(config.R_dim > 0, "R_dim must be greater than 0");
  HW_ASSERT(config.alpha >= 0 && config.alpha <= 1,
            "alpha must be in [0, 1]");
  HW_ASSERT(config.beta >= 0, "beta must be greater than 0");
  HW_ASSERT(config.kappa >= 0, "kappa must be greater than 0");
  HW_ASSERT(config.f != nullptr, "f must not be nullptr");
  HW_ASSERT(config.h != nullptr, "h must not be nullptr");
#pragma endreginon

  Q_ = Allocator<float>::allocate(kQDim_ * kQDim_);
  memcpy(Q_, config.Q, sizeof(float) * kQDim_ * kQDim_);
  R_ = Allocator<float>::allocate(kRDim_ * kRDim_);
  memcpy(R_, config.R, sizeof(float) * kRDim_ * kRDim_);

  /* 计算增广sigma点中噪声的部分 */
  float gamma = sqrtf(kNa_ + kLamda_);
  /* gamma * sqrt(Q)的列向量 */
  X_w_ = Allocator<float>::allocate(2 * kQDim_ * kQDim_);
  CholeskyDecomposition(kQDim_, Q_);
  for (size_t i = 0; i < kQDim_; i++) {
    for (size_t j = 0; j < kQDim_; j++) {
      X_w_[i * kQDim_ + j] = gamma * Q_[j * kQDim_ + i];
      X_w_[(i + kQDim_) * kQDim_ + j] = -X_w_[i * kQDim_ + j];
    }
  }

  /* gamma * sqrt(R)的列向量 */
  X_v_ = Allocator<float>::allocate(2 * kRDim_ * kRDim_);
  CholeskyDecomposition(kRDim_, R_);
  for (size_t i = 0; i < kRDim_; i++) {
    for (size_t j = 0; j < kRDim_; j++) {
      X_v_[i * kRDim_ + j] = gamma * R_[j * kRDim_ + i];
      X_v_[(i + kRDim_) * kRDim_ + j] = -X_v_[i * kRDim_ + j];
    }
  }

  w_mean_ = Allocator<float>::allocate(kQDim_);
  memset(w_mean_, 0, sizeof(float) * kQDim_);
  v_mean_ = Allocator<float>::allocate(kRDim_);
  memset(v_mean_, 0, sizeof(float) * kRDim_);

  P_ = Allocator<float>::allocate(kXDim_ * kXDim_);
  memcpy(P_, config.P, sizeof(float) * kXDim_ * kXDim_);

  if (config.x0 != nullptr) {
    setX0(config.x0);
  }
}

Ukf::~Ukf()
{
  Allocator<float>::deallocate(Q_, kQDim_ * kQDim_);

  Allocator<float>::deallocate(v_mean_, kRDim_);
  Allocator<float>::deallocate(w_mean_, kQDim_);

  Allocator<float>::deallocate(X_v_, 2 * kRDim_ * kRDim_);
  Allocator<float>::deallocate(X_w_, 2 * kQDim_ * kQDim_);

  Allocator<float>::deallocate(R_, kRDim_ * kRDim_);
  Allocator<float>::deallocate(Q_, kQDim_ * kQDim_);
}

void Ukf::calc(const float* u, const float* z)
{
  /* 变量检查 */
#pragma reginon
  HW_ASSERT(u != nullptr, "u must not be nullptr");
  HW_ASSERT(z != nullptr, "z must not be nullptr");
#pragma endreginon

  float* x_hat_bar = Allocator<float>::allocate(kXDim_);
  float* P_bar = Allocator<float>::allocate(kXDim_ * kXDim_);
  float* X_x_bar =
      Allocator<float>::allocate((2 * (kXDim_ + kQDim_) + 1) * kXDim_);

  predict(u, x_hat_bar, P_bar, X_x_bar);

  update(z, x_hat_bar, P_bar, X_x_bar);

  Allocator<float>::deallocate(X_x_bar, (2 * (kXDim_ + kQDim_) + 1) * kXDim_);
  Allocator<float>::deallocate(P_bar, kXDim_ * kXDim_);
  Allocator<float>::deallocate(x_hat_bar, kXDim_);
}

/**
 * @brief       设置初始状态
 * @param        x: 初始状态，大小为kXDim_
 * @retval       None
 * @note        None
 */
void Ukf::setX0(const float* x)
{
  /* 变量检查 */
#pragma reginon
  HW_ASSERT(x != nullptr, "x must not be nullptr");
#pragma endreginon

  memcpy(x_hat_, x, sizeof(float) * kXDim_);
}

/**
 * @brief       获取sigma点
 * @param        x: 状态量，大小为n
 * @param        P: 状态估计协方差矩阵
 * @param        n: 状态量维度
 * @param        scale: 缩放因子
 * @param        X: 返回sigma点，大小为(2 * n + 1, n)
 * @retval       None
 * @note        None
 */
void Ukf::getSigmaPnts(const float* x, const float* P, size_t n, float scale,
                       float* X) const
{
  /* 变量检查 */
#pragma reginon
  HW_ASSERT(x != nullptr, "x must not be nullptr");
  HW_ASSERT(P != nullptr, "P must not be nullptr");
  HW_ASSERT(X != nullptr, "X must not be nullptr");
#pragma endreginon

  float* L = Allocator<float>::allocate(n * n);
  /* 只获取下三角部分 */
  for (size_t i = 0; i < n; i++) {
    for (size_t j = 0; j <= i; j++) {
      L[i * n + j] = scale * P[i * n + j];
    }
  }
  CholeskyDecomposition(n, L);

  memcpy(X, x, sizeof(float) * n);                // i=0
  for (size_t i = 0; i < n; i++) {                // i=1~n，n+1~2n
    for (size_t j = 0; j < n; j++) {
      X[(i + 1) * n + j] = x[j] + L[j * n + i];
      X[(i + 1 + n) * n + j] = x[j] - L[j * n + i];
    }
  }

  Allocator<float>::deallocate(L, n * n);
}

/**
 * @brief       预测
 * @param        u: 控制量，大小为kUDim_
 * @param        x_hat_bar: 返回预测后的状态量，大小为kXDim_
 * @param        P_bar: 返回预测后的状态估计协方差矩阵，大小为(kXDim_, kXDim_)
 * @param        X_x_bar: 返回预测后的状态量的sigma点，
 * 大小为(2 * (kXDim_ + kQDim) + 1, kXDim_)
 * @retval       None
 * @note        None
 */
void Ukf::predict(const float* u, float* x_hat_bar, float* P_bar,
                  float* X_x_bar) const
{
  /* 变量检查 */
#pragma reginon
  HW_ASSERT(u != nullptr, "u must not be nullptr");
  HW_ASSERT(x_hat_bar != nullptr, "x_hat_bar must not be nullptr");
  HW_ASSERT(P_bar != nullptr, "P_bar must not be nullptr");
  HW_ASSERT(X_x_bar != nullptr, "X_x_bar must not be nullptr");
#pragma endreginon

  const size_t kXxBarLsLen = 2 * (kXDim_ + kQDim_) + 1;
  const size_t kXxLsLen = 2 * kXDim_ + 1;

  /* 含非线性噪声影响的下一时刻sigma点的预测值 */
  float(*X_x_bar_ls)[kXDim_] = reinterpret_cast<float(*)[kXDim_]>(X_x_bar);
  {
    float* X_x_ls_data = Allocator<float>::allocate(kXxLsLen * kXDim_);
    float(*X_x_ls)[kXDim_] = reinterpret_cast<float(*)[kXDim_]>(X_x_ls_data);
    getSigmaPnts(x_hat_, P_, kXDim_, kNa_ + kLamda_, X_x_ls_data);

    /* 平均值权重 */
    const float kWm = 1 / (2 * (kLamda_ + kNa_));
    const float kWm0 = kLamda_ / (kLamda_ + kNa_) + kRDim_ / (kLamda_ + kNa_);
    memset(x_hat_bar, 0, sizeof(float) * kXDim_);

    float w_m = kWm0;
    for (size_t i = 0; i < kXxBarLsLen; i++) {
      if (i < kXxLsLen) {
        f_(X_x_ls[i], u, w_mean_, X_x_bar_ls[i]);
      } else {
        /* 噪声部分 */
        f_(X_x_ls[0], u, X_w_ + (i - kXxLsLen) * kQDim_,
           X_x_bar_ls[i]);
      }

      for (size_t j = 0; j < kXDim_; j++) {
        x_hat_bar[j] += w_m * X_x_bar_ls[i][j];
      }
      w_m = kWm;
    }

    Allocator<float>::deallocate(X_x_ls_data, kXxLsLen * kXDim_);
  }

  /* 协方差权重 */
  const float kWc = 1 / (2 * (kNa_ + kLamda_));
  const float kWc0 = kLamda_ / (kLamda_ + kNa_) + 1 - kAlpha_ * kAlpha_ +
                     kBeta_ + kRDim_ / (kLamda_ + kNa_);
  memset(P_bar, 0, sizeof(float) * kXDim_ * kXDim_);

  float w_c = kWc0;
  for (size_t i = 0; i < kXxBarLsLen; i++) {
    for (size_t j = 0; j < kXDim_; j++) {    // 行
      for (size_t k = 0; k < kXDim_; k++) {  // 列
        P_bar[j * kXDim_ + k] += w_c * (X_x_bar_ls[i][j] - x_hat_bar[j]) *
                                 (X_x_bar_ls[i][k] - x_hat_bar[k]);
      }
    }
    w_c = kWc;
  }
}

/**
 * @brief       更新
 * @param        z: 观测量，大小为kZDim_
 * @param        x_hat_bar: 预测后的状态量，大小为kXDim_
 * @param        P_bar: 预测后的状态估计协方差矩阵，大小为(kXDim_, kXDim_)
 * @param        X_x_bar: 预测后的状态量的sigma点，
 * 大小为(2 * (kXDim_ + kQDim) + 1, kXDim_)
 * @retval       None
 * @note        None
 */
void Ukf::update(const float* z, const float* x_hat_bar, const float* P_bar,
                 const float* X_x_bar)
{
  /* 变量检查 */
#pragma reginon
  HW_ASSERT(z != nullptr, "z must not be nullptr");
  HW_ASSERT(x_hat_bar != nullptr, "x_hat_bar must not be nullptr");
  HW_ASSERT(P_bar != nullptr, "P_bar must not be nullptr");
  HW_ASSERT(X_x_bar != nullptr, "X_x_bar must not be nullptr");
#pragma endreginon

  float* Pzz = Allocator<float>::allocate(kZDim_ * kZDim_);
  float* K = Allocator<float>::allocate(kXDim_ * kZDim_);
  arm_matrix_instance_f32 K_mat, Pzz_mat;
  {
    float* Pxz = Allocator<float>::allocate(kXDim_ * kZDim_);
    float* z_hat = Allocator<float>::allocate(kZDim_);
    getOutputStatData(x_hat_bar, P_bar, X_x_bar, Pzz, Pxz, z_hat);

    float* Pzz_inv = Allocator<float>::allocate(kZDim_ * kZDim_);
    float* Pzz_tmp = Allocator<float>::allocate(kZDim_ * kZDim_);
    memcpy(Pzz_tmp, Pzz, sizeof(float) * kZDim_ * kZDim_);
    arm_matrix_instance_f32 Pxz_mat, Pzz_inv_mat;
    arm_mat_init_f32(&K_mat, kXDim_, kZDim_, K);
    arm_mat_init_f32(&Pzz_mat, kZDim_, kZDim_, Pzz);
    arm_mat_init_f32(&Pxz_mat, kXDim_, kZDim_, Pxz);
    arm_mat_init_f32(&Pzz_inv_mat, kZDim_, kZDim_, Pzz_inv);
    arm_mat_inverse_f32(&Pzz_mat, &Pzz_inv_mat);
    memcpy(Pzz, Pzz_tmp, sizeof(float) * kZDim_ * kZDim_);
    arm_mat_mult_f32(&Pxz_mat, &Pzz_inv_mat, &K_mat);

    float* err = Allocator<float>::allocate(kZDim_);
    float* delta_x = Allocator<float>::allocate(kXDim_);
    arm_sub_f32(z, z_hat, err, kZDim_);
    arm_mat_vec_mult_f32(&K_mat, err, delta_x);
    arm_add_f32(x_hat_bar, delta_x, x_hat_, kXDim_);

    Allocator<float>::deallocate(delta_x, kXDim_);
    Allocator<float>::deallocate(err, kZDim_);
    Allocator<float>::deallocate(Pzz_tmp, kZDim_ * kZDim_);
    Allocator<float>::deallocate(Pzz_inv, kZDim_ * kZDim_);
    Allocator<float>::deallocate(z_hat, kZDim_);
    Allocator<float>::deallocate(Pxz, kXDim_ * kZDim_);
  }

  float* K_T = Allocator<float>::allocate(kZDim_ * kXDim_);
  float* tmp1 = Allocator<float>::allocate(kXDim_ * kZDim_);
  float* tmp2 = Allocator<float>::allocate(kXDim_ * kXDim_);
  arm_matrix_instance_f32 K_T_mat, tmp1_mat, tmp2_mat;
  arm_mat_init_f32(&K_T_mat, kZDim_, kXDim_, K_T);
  arm_mat_init_f32(&tmp1_mat, kXDim_, kZDim_, tmp1);
  arm_mat_init_f32(&tmp2_mat, kXDim_, kXDim_, tmp2);
  arm_mat_trans_f32(&K_mat, &K_T_mat);
  arm_mat_mult_f32(&K_mat, &Pzz_mat, &tmp1_mat);
  arm_mat_mult_f32(&tmp1_mat, &K_T_mat, &tmp2_mat);
  arm_sub_f32(P_bar, tmp2, P_, kXDim_ * kXDim_);

  Allocator<float>::deallocate(tmp2, kXDim_ * kXDim_);
  Allocator<float>::deallocate(tmp1, kXDim_ * kZDim_);
  Allocator<float>::deallocate(K_T, kZDim_ * kXDim_);

  Allocator<float>::deallocate(K, kXDim_ * kZDim_);
  Allocator<float>::deallocate(Pzz, kZDim_ * kZDim_);
}

/**
 * @brief       获取估计输出统计数据
 * @param        x_hat_bar: 预测后的状态量，大小为kXDim_
 * @param        P_bar: 预测后的状态估计协方差矩阵，大小为(kXDim_, kXDim_)
 * @param        X_x_bar: 预测后的状态量的sigma点，
 * 大小为(2 * (kXDim_ + kQDim) + 1, kXDim_)
 * @param        Pzz: 返回Pzz，大小为(kZDim_, kZDim_)
 * @param        Pxz: 返回Pxz，大小为(kXDim_, kZDim_)
 * @param        z_hat: 返回z_hat，大小为kZDim_
 * @retval       None
 * @note        None
 */
void Ukf::getOutputStatData(
    const float* x_hat_bar, const float* P_bar, const float* X_x_bar,
    float* Pzz, float* Pxz, float* z_hat)
{
  /* 变量检查 */
#pragma reginon
  HW_ASSERT(x_hat_bar != nullptr, "x_hat_bar must not be nullptr");
  HW_ASSERT(P_bar != nullptr, "P_bar must not be nullptr");
  HW_ASSERT(X_x_bar != nullptr, "X_x_bar must not be nullptr");
  HW_ASSERT(Pzz != nullptr, "Pzz must not be nullptr");
  HW_ASSERT(Pxz != nullptr, "Pxz must not be nullptr");
  HW_ASSERT(z_hat != nullptr, "z_hat must not be nullptr");
#pragma endreginon
  const size_t kXxBarLsLen = 2 * (kXDim_ + kQDim_) + 1;
  const size_t kZHatLsLen = 2 * kNa_ + 1;

  const float(*X_x_bar_ls)[kXDim_] =
      reinterpret_cast<const float(*)[kXDim_]>(X_x_bar);

  /* 获取Pzz，Pxz与z_hat */
  /* 含噪声影响的下一时刻sigma点的观察值 */
  float* Z_hat_ls_data = Allocator<float>::allocate(kZHatLsLen * kZDim_);
  float(*Z_hat_ls)[kZDim_] = reinterpret_cast<float(*)[kZDim_]>(Z_hat_ls_data);

  /* 平均值权重 */
  const float kWm = 1 / (2 * (kNa_ + kLamda_));
  const float kWm0 = kLamda_ / (kLamda_ + kNa_);
  memset(z_hat, 0, sizeof(float) * kZDim_);

  float w_m = kWm0;
  for (size_t i = 0; i < kZHatLsLen; i++) {
    if (i < kXxBarLsLen) {
      h_(X_x_bar_ls[i], v_mean_, Z_hat_ls[i]);
    } else {
      /* 噪声部分 */
      h_(X_x_bar_ls[0], X_v_ + (i - kXxBarLsLen) * kRDim_,
         Z_hat_ls[i]);
    }

    for (size_t j = 0; j < kZDim_; j++) {
      z_hat[j] += w_m * Z_hat_ls[i][j];
    }
    w_m = kWm;
  }

  /* 协方差权重 */
  const float kWc = 1 / (2 * (kNa_ + kLamda_));
  const float kWc0 = kLamda_ / (kLamda_ + kNa_) + 1 - kAlpha_ * kAlpha_ +
                     kBeta_;
  memset(Pzz, 0, sizeof(float) * kZDim_ * kZDim_);
  memset(Pxz, 0, sizeof(float) * kXDim_ * kZDim_);

  float w_c = kWc0;
  for (size_t i = 0; i < kZHatLsLen; i++) {
    for (size_t k = 0; k < kZDim_; k++) {    // 列
      for (size_t j = 0; j <= k; j++) {      // 行
        Pzz[j * kZDim_ + k] += w_c * (Z_hat_ls_data[i * kZDim_ + j] - z_hat[j]) *
                               (Z_hat_ls_data[i * kZDim_ + k] - z_hat[k]);
        if (j != k) {
          Pzz[k * kZDim_ + j] = Pzz[j * kZDim_ + k];
        }
      }

      for (size_t j = 0; j < kXDim_; j++) {  // 行
        if (i < kXxBarLsLen) {
          Pxz[j * kZDim_ + k] += w_c * (X_x_bar_ls[i][j] - x_hat_bar[j]) *
                                 (Z_hat_ls_data[i * kZDim_ + k] - z_hat[k]);
        } else {
          /* 噪声部分 */
          Pxz[j * kZDim_ + k] += w_c * (X_x_bar_ls[0][j] - x_hat_bar[j]) *
                                 (Z_hat_ls_data[i * kZDim_ + k] - z_hat[k]);
        }
      }
    }
    w_c = kWc;
  }

  Allocator<float>::deallocate(Z_hat_ls_data, kZHatLsLen * kZDim_);
}

/**
 * @brief       Cholesky分解
 * @param        A: 待分解矩阵，可只含有下三角部分，返回分解后的下三角部分
 * @param        n: 矩阵维度
 * @retval       None
 * @note        None
 */
static void CholeskyDecomposition(const size_t n, float* A)
{
  /* 变量检查 */
#pragma reginon
  HW_ASSERT(A != nullptr, "A must not be nullptr");
  HW_ASSERT(n > 0, "n must be greater than 0");
#pragma endreginon

  float(*A_mat)[n] = reinterpret_cast<float(*)[n]>(A);
  for (size_t i = 0; i < n; i++) {    // 行
    for (size_t j = 0; j <= i; j++) {  // 列
      float sum = 0;
      if (i > j) {  // 下三角部分
        for (size_t k = 0; k < j; k++) {
          sum += A_mat[i][k] * A_mat[j][k];
        }
        A_mat[i][j] = (A_mat[i][j] - sum) / A_mat[j][j];
      } else {  // 对角线
        for (size_t k = 0; k < i; k++) {
          sum += A_mat[i][k] * A_mat[i][k];
        }
        A_mat[i][i] = sqrtf(A_mat[i][i] - sum);
      }
    }
    /* 上三角部分 */
    memset(A_mat[i] + i + 1, 0, sizeof(float) * (n - i - 1));
  }
}
}  // namespace observer
}  // namespace hello_world