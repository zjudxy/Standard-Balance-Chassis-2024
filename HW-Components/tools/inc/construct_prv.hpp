/**
 *******************************************************************************
 * @file      : construct_prv.hpp
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
#ifndef HW_COMPONENTS_TOOLS_CONSTRUCT_PRV_HPP_
#define HW_COMPONENTS_TOOLS_CONSTRUCT_PRV_HPP_

/* Includes ------------------------------------------------------------------*/
#include <type_traits>

namespace hello_world
{
namespace prv
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
template <typename T>
void _DestroyOne(T*, std::true_type)
{
}

template <typename T>
void _DestroyOne(T* pointer, std::false_type)
{
  if (pointer != nullptr) {
    pointer->~T();
  }
}

template <typename T>
void _DestroyRange(T*, T*, std::true_type)
{
}

template <typename T>
void _DestroyRange(T* first, T* last, std::false_type)
{
  for (; first != last; ++first) {
    Destroy(first);
  }
}
}  // namespace prv
}  // namespace hello_world
#endif /* HW_COMPONENTS_TOOLS_CONSTRUCT_PRV_HPP_ */
