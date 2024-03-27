/**
 *******************************************************************************
 * @file      : ekf.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONETS_ALGORITHMS_OBSERVER_EKF_HPP_
#define HW_COMPONETS_ALGORITHMS_OBSERVER_EKF_HPP_

/* Includes ------------------------------------------------------------------*/
#include "arm_math.h"
#include "observer_base.hpp"

namespace hello_world
{
namespace observer
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

class Ekf : public Observer
{
 public:
  /*<! 状态方程 x_k = f(x_(k-1), u_k) */
  typedef void (*pStateFunc)(const float* x_k_1, const float* u_k, float* x_k);

  /*<! 输出方程 z_k = h(x_k) */
  typedef void (*pOutputFunc)(const float* x_k, float* z_k);

  /*<! 获取状态方程雅可比矩阵 J_F = partial f / partial x_(k-1) | (x_hat_k_1, u_k)*/
  typedef void (*pGetJ_F)(const float* x_hat_k_1, const float* u_k, float* J_F);

  /*<! 获取输出方程雅可比矩阵 J_H = partial h / partial x_k | (x_hat_k) */
  typedef void (*pGetJ_H)(const float* x_hat_k, float* J_H);

  struct Config {
    size_t x_dim;  //!< 状态量维度
    size_t z_dim;  //!< 观测量维度
    size_t u_dim;  //!< 控制量维度

    pStateFunc f;    //!< 状态方程
    pOutputFunc h;   //!< 输出方程
    pGetJ_F GetJ_F;  //!< 获取状态方程雅可比矩阵
    pGetJ_H GetJ_H;  //!< 获取输出方程雅可比矩阵

    float* Q;   //!< 状态噪声协方差矩阵
    float* R;   //!< 观测噪声协方差矩阵
    float* P;   //!< 状态估计协方差矩阵，不使用时置为nullptr，此时设置为零状态
    float* x0;  //!< 初始状态，不使用时置为nullptr，此时设置为零状态
  };

  Ekf(const Config& config);
  virtual ~Ekf(void);

  virtual void calc(const float* u, const float* z) override;

  /**
   * @brief       重置
   * @param        None
   * @retval       None
   * @note        None
   */
  void reset(void) override
  {
    memset(x_hat_, 0, kXDim_ * sizeof(float));
    memset(P_, 0, kXDim_ * kXDim_ * sizeof(float));
  };

  void setX0(const float* x) override;

 private:
  void predict(const float* u, float* x_hat_bar,
               arm_matrix_instance_f32* P_bar_mat_ptr) const;

  void update(const float* z, const float* x_hat_bar,
              const arm_matrix_instance_f32& P_bar_mat);

  pStateFunc f_;
  pOutputFunc h_;
  pGetJ_F GetJ_F_;
  pGetJ_H GetJ_H_;

  float* P_;
  float* Q_;
  float* R_;

  arm_matrix_instance_f32 P_mat_;
  arm_matrix_instance_f32 Q_mat_;
  arm_matrix_instance_f32 R_mat_;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace observer
}  // namespace hello_world

#endif /* HW_COMPONETS_ALGORITHMS_OBSERVER_EKF_HPP_ */
