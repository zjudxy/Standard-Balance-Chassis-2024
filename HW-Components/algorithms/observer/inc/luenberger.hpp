/**
 *******************************************************************************
 * @file      : luenberger.hpp
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
#ifndef HW_COMPONENTS_ALGORITHMS_OBSERVER_LUENBERGER_HPP_
#define HW_COMPONENTS_ALGORITHMS_OBSERVER_LUENBERGER_HPP_

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

class Luenberger : public Observer
{
 public:
  struct Config {
    size_t x_dim;  //!< 状态量维度
    size_t z_dim;  //!< 观测量维度
    size_t u_dim;  //!< 控制量维度

    float* F;   //!< 系统矩阵
    float* B;   //!< 控制矩阵
    float* H;   //!< 观测矩阵
    float* L;   //!< 龙伯格增益矩阵
    float* x0;  //!< 初始状态，不使用时置为nullptr，此时设置为零状态
  };

  Luenberger(const Config& config_d);
  Luenberger(const Config& config_c, float Ts);
  virtual ~Luenberger(void);

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
  }

  void setX0(const float* x) override;

 private:
  float* Fd_;
  arm_matrix_instance_f32 Fd_mat_;  //!< 离散系统矩阵
  float* Bd_;
  arm_matrix_instance_f32 Bd_mat_;  //!< 离散控制矩阵
  float* Hd_;
  arm_matrix_instance_f32 Hd_mat_;  //!< 离散观测矩阵
  float* Ld_;
  arm_matrix_instance_f32 Ld_mat_;  //!< 离散龙伯格增益矩阵
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace observer
}  // namespace hello_world

#endif /* HW_COMPONENTS_ALGORITHMS_OBSERVER_LUENBERGER_HPP_ */
