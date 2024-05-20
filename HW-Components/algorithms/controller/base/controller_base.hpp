/**
 *******************************************************************************
 * @file      controller_base.hpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2023-11-25      Caikunzhen      1. 未测试版本
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2023 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_ALGORITHMS_CONTROLLER_CONTROLLER_BASE_HPP_
#define HW_COMPONENTS_ALGORITHMS_CONTROLLER_CONTROLLER_BASE_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstddef>

#include "allocator.hpp"
namespace hello_world
{

/* Exported macro ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
enum ControllerState {
  kControllerStateOk,
  kControllerStateError,
};

class Controller : public MemMang
{
 public:
  typedef ControllerState State;
  explicit Controller() = default;
  explicit Controller(size_t ref_dim, size_t fdb_dim, size_t out_dim, size_t ffd_dim = 0)
      : ref_dim_(ref_dim), fdb_dim_(fdb_dim), ffd_dim_(ffd_dim), out_dim_(out_dim){};

  virtual ~Controller() {}
  virtual State calc(const float* ref_ls, const float* fdb_ls, const float* ffd_ls, float* out_ls) = 0;

  virtual State reset() = 0;

 protected:
  size_t ref_dim_ = 0;
  size_t fdb_dim_ = 0;
  size_t ffd_dim_ = 0;
  size_t out_dim_ = 0;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

}  // namespace hello_world

#endif /* HW_COMPONENTS_ALGORITHMS_CONTROLLER_CONTROLLER_BASE_HPP_ */
