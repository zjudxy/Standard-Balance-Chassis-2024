

/**
 * @file      system.hpp
 * @brief
 * @author    ZhouShichan (zsc19823382069@163.com)
 * @version   1.0.0
 * @date      2023-11-25
 *
 * @copyright Copyright (c) 2023 Hello World Team, Zhejiang University. All Rights Reserved.
 *
 * @attention
 *
 * @par history
 * | Version | Date | Author | Description |
 * | :---: | :---: | :---: | :---: |
 * | 1.0.0 | {now_year} | ZhouShichan | description |
 * @par last editor  ZhouShichan (zsc19823382069@163.com)
 * @par last edit time  2023-11-25
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_TOOLS_SYSTEM_HPP_
#define HW_COMPONENTS_TOOLS_SYSTEM_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdlib>

namespace hello_world
{
/* Exported macro ------------------------------------------------------------*/
/* memory manage and RTOS features */
#if USE_FREERTOS_CMSIS
#include "cmsis_os.h"
#define MALLOC pvPortMalloc
#define FREE vPortFree
#define ENTER_CRITICAL() taskENTER_CRITICAL()
#define EXIT_CRITICAL() taskEXIT_CRITICAL()
#define ENTER_CRITICAL_FROM_ISR() taskENTER_CRITICAL_FROM_ISR()
#define EXIT_CRITICAL_FROM_ISR() taskEXIT_CRITICAL_FROM_ISR()
#define Mutex_t SemaphoreHandle_t
#define MUTEX_INIT() xSemaphoreCreateMutex()
#define MUTEX_LOCK(xSemaphore) pdFALSE == xSemaphoreTake(xSemaphore, (TickType_t)10) ? SYS_ERROR : SYS_OK
#define MUTEX_UNLOCK(xSemaphore) pdFALSE == xSemaphoreGive(xSemaphore) ? SYS_ERROR : SYS_OK
#define MUTEX_LOCK_FROM_ISR(xSemaphore, pxHigherPriorityTaskWoken) \
  pdFALSE == xSemaphoreTakeFromISR(xSemaphore, pxHigherPriorityTaskWoken) ? SYS_ERROR : SYS_OK
#define MUTEX_UNLOCK_FROM_ISR(xSemaphore, pxHigherPriorityTaskWoken) \
  pdFALSE == xSemaphoreGiveFromISR(xSemaphore, pxHigherPriorityTaskWoken) ? SYS_ERROR : SYS_OK
#else
#define ENTER_CRITICAL() \
  do {                   \
    __disable_irq();     \
  } while (0);
#define EXIT_CRITICAL() \
  do {                  \
    __enable_irq();     \
  } while (0);
#define Mutex_t bool
#define MUTEX_INIT() true
#define MUTEX_LOCK(x) \
  do {                \
    (x) = false;      \
  } while (0)
#define MUTEX_UNLOCK(x) \
  do {                  \
    (x) = true;         \
  } while (0)
#endif
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace hello_world

// inline void* operator new(size_t size) noexcept { return MALLOC(size); }
// inline void operator delete(void* ptr) noexcept { return FREE(ptr); }
// inline void* operator new[](size_t size) noexcept { return MALLOC(size); }
// inline void operator delete[](void* ptr) noexcept { return FREE(ptr); }

#endif /* HW_COMPONENTS_TOOLS_SYSTEM_HPP_ */
