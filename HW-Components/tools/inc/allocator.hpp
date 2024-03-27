/**
 *******************************************************************************
 * @file      : allocator.hpp
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
#ifndef HW_COMPONENTS_TOOLS_ALLOCATOR_HPP_
#define HW_COMPONENTS_TOOLS_ALLOCATOR_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdlib>
#include <utility>

#include "construct.hpp"
#include "system.hpp"

namespace hello_world
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

template <typename T>
class Allocator
{
 public:
  using value_type = T;
  using pointer = T*;
  using const_pointer = const T*;
  using reference = T&;
  using const_reference = const T&;
  using size_type = size_t;
  using difference_type = ptrdiff_t;

  template <typename U>
  struct rebind {
    using other = Allocator<U>;
  };

  Allocator() = default;
  ~Allocator() = default;
  Allocator(const Allocator&) = default;
  Allocator& operator=(const Allocator&) = default;

  template <typename U>
  Allocator(const Allocator<U>&)
  {
  }

  static pointer address(reference x)
  {
    return &x;
  }

  static const_pointer address(const_reference x)
  {
    return &x;
  }

  static pointer allocate(size_type n, const void* hint = 0)
  {
    return static_cast<pointer>(malloc(n * sizeof(T)));
  }

  static void deallocate(pointer p, size_type n)
  {
    free(p);
  }

  static size_type max_size()
  {
    return size_type(-1) / sizeof(T);
  }

  template <typename U, typename... Args>
  static void construct(U* p, Args&&... args)
  {
    Construct(p, std::forward<Args>(args)...);
  }

  template <typename U>
  static void destroy(U* p)
  {
    Destroy(p);
  }
};

template <>
class Allocator<void>
{
 public:
  using value_type = void;
  using pointer = void*;
  using const_pointer = const void*;
  using size_type = size_t;
  using difference_type = ptrdiff_t;

  template <typename U>
  struct rebind {
    using other = Allocator<U>;
  };

  Allocator() = default;
  ~Allocator() = default;
  Allocator(const Allocator&) = default;
  Allocator& operator=(const Allocator&) = default;

  template <typename U>
  Allocator(const Allocator<U>&)
  {
  }

  static pointer allocate(size_type n, const void* hint = 0)
  {
    return static_cast<pointer>(malloc(n));
  }

  static void deallocate(pointer p, size_type n)
  {
    free(p);
  }

  template <typename U, typename... Args>
  static void construct(U* p, Args&&... args)
  {
    Construct(p, std::forward<Args>(args)...);
  }

  template <typename U>
  static void destroy(U* p)
  {
    Destroy(p);
  }
};

/** 用于重载类的内存申请，所有可能需要内存申请的类需要为该类的子类 */
class MemMang
{
 public:
  void* operator new(size_t size) { return Allocator<void>::allocate(size); }
  void* operator new[](size_t size) { return Allocator<void>::allocate(size); }
  void operator delete(void* ptr) { Allocator<void>::deallocate(ptr, 0); }
  void operator delete[](void* ptr) { Allocator<void>::deallocate(ptr, 0); }
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace hello_world

#endif /* HW_COMPONENTS_TOOLS_ALLOCATOR_HPP_ */
