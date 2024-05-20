/**
 *******************************************************************************
 * @file      : observer_base.hpp
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
#ifndef HW_COMPONENTS_ALGORITHMS_OBSERVER_OBSERVER_BASE_HPP_
#define HW_COMPONENTS_ALGORITHMS_OBSERVER_OBSERVER_BASE_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstddef>
#include <cstring>

#include "allocator.hpp"

namespace hello_world
{
namespace observer
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

class Observer : public MemMang
{
 public:
  Observer(size_t x_dim, size_t z_dim, size_t u_dim);
  virtual ~Observer(void);

  /**
   * @brief       观测计算
   * @param        u: 控制量，大小为kUDim_
   * @param        z: 观测量，大小为kYDim_
   * @retval       None
   * @note        None
   */
  virtual void calc(const float* u, const float* z) = 0;

  void getX(float* x_hat) const;

  /**
   * @brief       设置初始状态
   * @param        x: 初始状态，大小为kXDim_
   * @retval       None
   * @note        None
   */
  virtual void setX0(const float* x) = 0;

  /**
   * @brief       重置
   * @param        None
   * @retval       None
   * @note        None
   */
  virtual void reset(void) = 0;

  const size_t kXDim_;  ///< 状态量维数
  const size_t kZDim_;  ///< 观测量维数
  const size_t kUDim_;  ///< 控制量维数

 protected:
  float* x_hat_;  ///< 最优状态估计量
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace observer
}  // namespace hello_world

#endif /* HW_COMPONENTS_ALGORITHMS_OBSERVER_OBSERVER_BASE_HPP_ */
