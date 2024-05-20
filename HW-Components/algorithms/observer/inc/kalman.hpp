/**
 *******************************************************************************
 * @file      : kalman.hpp
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
#ifndef HW_COMPONENTS_ALGORITHMS_OBSERVER_KALMAN_HPP_
#define HW_COMPONENTS_ALGORITHMS_OBSERVER_KALMAN_HPP_

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

class Kalman : public Observer
{
 public:
  struct Config {
    size_t x_dim;  //!< 状态量维度
    size_t z_dim;  //!< 观测量维度
    size_t u_dim;  //!< 控制量维度

    float* F;   //!< 系统矩阵
    float* B;   //!< 控制矩阵
    float* H;   //!< 观测矩阵
    float* Q;   //!< 状态噪声协方差矩阵
    float* R;   //!< 观测噪声协方差矩阵
    float* P;   //!< 状态估计协方差矩阵，不使用时置为nullptr，此时设置为零状态
    float* x0;  //!< 初始状态，不使用时置为nullptr，此时设置为零状态
  };

  Kalman(const Config& config);
  virtual ~Kalman(void);

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
  }

  void setX0(const float* x0) override;

 private:
  void predict(const float* u, float* x_hat_bar,
               arm_matrix_instance_f32* P_bar_mat_ptr) const;

  void update(const float* z, const float* x_hat_bar,
              const arm_matrix_instance_f32& P_bar_mat);

  float* F_;
  float* B_;
  float* H_;
  float* Q_;
  float* R_;
  float* P_;

  arm_matrix_instance_f32 F_mat_;
  arm_matrix_instance_f32 B_mat_;
  arm_matrix_instance_f32 H_mat_;
  arm_matrix_instance_f32 Q_mat_;
  arm_matrix_instance_f32 R_mat_;
  arm_matrix_instance_f32 P_mat_;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace observer
}  // namespace hello_world

#endif /* HW_COMPONENTS_ALGORITHMS_OBSERVER_KALMAN_HPP_ */
