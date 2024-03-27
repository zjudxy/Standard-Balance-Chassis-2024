/**
 *******************************************************************************
 * @file      : construct.hpp
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
#ifndef HW_COMPONENTS_TOOLS_CONSTRUCT_HPP_
#define HW_COMPONENTS_TOOLS_CONSTRUCT_HPP_

/* Includes ------------------------------------------------------------------*/
#include <new>
#include <type_traits>
#include<utility>

#include "construct_prv.hpp"

namespace hello_world
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

template <typename T>
void Construct(T* ptr)
{
  ::new (ptr) T();
}

template <typename T1, typename T2>
void Construct(T1* ptr, const T2& value)
{
  ::new (ptr) T1(value);
}

template <typename T, typename... Args>
void Construct(T* ptr, Args&&... args)
{
  ::new (ptr) T(std::forward<Args>(args)...);
}

template <typename T>
void Destroy(T* pointer)
{
  prv::_DestroyOne(pointer, std::is_trivially_destructible<T>());
}

template <typename T>
void Destroy(T* first, T* last)
{
  prv::_DestroyRange(first, last, std::is_trivially_destructible<T>());
}
}  // namespace hello_world

#endif /* HW_COMPONENTS_TOOLS_CONSTRUCT_HPP_ */
