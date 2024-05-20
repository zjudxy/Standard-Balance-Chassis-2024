/**
 *******************************************************************************
 * @file      : fsm.cpp
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
/* Includes ------------------------------------------------------------------*/
#include "fsm.hpp"

#include "assert.hpp"
#include "construct.hpp"

namespace hello_world
{
namespace fsm
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
 * @brief       状态机初始化
 * @param        state_num: 状态数量
 * @param        update_func: 状态更新函数，在update中自动调用
 * @note        None
 */
FsmBase::FsmBase(size_t state_num, std::function<State(void)> update_func)
    : kStateNum_(state_num), update_func_(update_func)
{
  state_ = 0;

  state_func_ls_ =
      Allocator<std::function<void(void)>>::allocate(kStateNum_);
  for (size_t i = 0; i < kStateNum_; i++) {
    Allocator<std::function<void(void)>>::construct(state_func_ls_ + i);
  }
}

/**
 * @brief       状态机析构
 * @param        None
 * @note        None
 */
FsmBase::~FsmBase()
{
  for (size_t i = 0; i < kStateNum_; i++) {
    Allocator<std::function<void(void)>>::destroy(state_func_ls_ + i);
  }
  Allocator<std::function<void(void)>>::deallocate(state_func_ls_, kStateNum_);
}

/**
 * @brief       状态机状态更新
 * @retval       ReturnState
 * @note        None
 */
ReturnState FsmBase::update(void)
{
  State state = update_func_();

  if (state < kStateNum_) {
    if (state != state_) {
      state_ = state;
      return kReturnStateChangeState;
    } else {
      return kReturnStateOk;
    }
  } else {
    return kReturnStateStateOutOfRange;
  }
}

/**
 * @brief       状态机运行
 * @param        None
 * @retval       ReturnState
 * @note        None
 */
ReturnState FsmBase::run(void)
{
  state_func_ls_[state_]();
  return kReturnStateOk;
}

/**
 * @brief       状态机注册状态
 * @param        state: 状态
 * @param        func: 回调函数，在run中自动调用
 * @retval       ReturnState
 * @note        状态超出范围时不会注册
 */
ReturnState FsmBase::registerStateFunc(State state, std::function<void(void)> func)
{
  if (state < kStateNum_) {
    state_func_ls_[state] = func;
    return kReturnStateOk;
  } else {
    return kReturnStateStateOutOfRange;
  }
}
}  // namespace fsm
}  // namespace hello_world
