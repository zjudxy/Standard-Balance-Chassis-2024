/**
 *******************************************************************************
 * @file      : fsm.hpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_ALGORITHMS_FSM_FSM_HPP_
#define HW_COMPONENTS_ALGORITHMS_FSM_FSM_HPP_

/* Includes ------------------------------------------------------------------*/
#include <functional>

#include "allocator.hpp"

namespace hello_world
{
namespace fsm
{
/* Exported macro ------------------------------------------------------------*/
enum ReturnState {
  kReturnStateOk = 0x0,
  kReturnStateStateOutOfRange,
  kReturnStateChangeState,
};
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

class FsmBase : MemMang
{
 public:
  typedef size_t State;

  FsmBase(size_t state_num, std::function<State(void)> update_func);

  virtual ~FsmBase();

  ReturnState update(void);

  ReturnState run(void);

  ReturnState registerStateFunc(State state, std::function<void(void)> func);

  State get_state(void) const { return state_; }

  const size_t kStateNum_;

 private:
  State state_;  ///< 当前状态

  std::function<State(void)> update_func_;  ///< 状态更新函数

  std::function<void(void)>* state_func_ls_;  ///< 状态回调函数列表
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace fsm
}  // namespace hello_world

#endif /* HW_COMPONENTS_ALGORITHMS_FSM_FSM_HPP_ */
