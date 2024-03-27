/**
 *******************************************************************************
 * @file      : ukf.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_ALGORITHMS_OBSERVER_UKF_HPP_
#define HW_COMPONENTS_ALGORITHMS_OBSERVER_UKF_HPP_

/* Includes ------------------------------------------------------------------*/
#include "arm_math.h"
#include "observer.hpp"

namespace hello_world
{
namespace observer
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

class Ukf : public Observer
{
 public:
  /*<! 状态方程 x_k = f(x_(k-1), u_k, w_k)，噪声非线性 */
  typedef void (*pStateFunc)(
      const float* x_k_1, const float* u_k, const float* w_k, float* x_k);

  /*<! 输出方程 z_k = h(x_k, v_k)，噪声非线性 */
  typedef void (*pOutputFunc)(const float* x_k, const float* v_k, float* z_k);

  struct Config {  //!< 噪声非线性观测器配置
    size_t x_dim;  //!< 状态量维度
    size_t z_dim;  //!< 观测量维度
    size_t u_dim;  //!< 控制量维度

    size_t Q_dim;  //!< 状态噪声协方差矩阵维度
    size_t R_dim;  //!< 观测噪声协方差矩阵维度

    float* Q;   //!< 状态噪声协方差矩阵
    float* R;   //!< 观测噪声协方差矩阵
    float* P;   //!< 状态估计协方差矩阵，不可为零矩阵
    float* x0;  //!< 初始状态，不使用时置为nullptr，此时设置为零状态

    float alpha = 0.001f;  //!< 用于确定sigma点的分布
    float beta = 2.0f;     //!< 用于确定高斯分布的权重
    float kappa = 0.0f;    //!< 用于确定高斯分布的权重

    pStateFunc f;   //!< 状态方程
    pOutputFunc h;  //!< 输出方程
  };

  Ukf(const Config& config);
  virtual ~Ukf();

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

  const float kAlpha_;
  const float kBeta_;
  const float kKappa_;

 private:
  void getSigmaPnts(const float* x, const float* P, size_t n, float scale,
                    float* X) const;

  void predict(const float* u, float* x_hat_bar, float* P_bar,
               float* X_x_bar) const;

  void update(const float* z, const float* x_hat_bar, const float* P_bar,
              const float* X_x_bar);

  void getOutputStatData(const float* x_hat_bar, const float* P_bar,
                         const float* X_x_bar, float* Pzz, float* Pxz,
                         float* z_hat);

  float* Q_;
  float* R_;
  float* P_;

  float* X_w_;           //!< 状态噪声sigma点
  float* X_v_;           //!< 观测噪声sigma点
  float* w_mean_;        //!< 状态噪声均值
  float* v_mean_;        //!< 观测噪声均值

  pStateFunc f_;
  pOutputFunc h_;

  const size_t kQDim_;
  const size_t kRDim_;
  const size_t kNa_;  //!< 增广状态量维度

  const float kLamda_;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace observer
}  // namespace hello_world

#endif /* HW_COMPONENTS_ALGORITHMS_OBSERVER_UKF_HPP_ */
