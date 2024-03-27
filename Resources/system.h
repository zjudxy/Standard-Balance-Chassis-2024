/**
 *******************************************************************************
 * @file      : system.h
 * @brief     : stm32 system configurations
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2022-12-06      Hello World     1. Developing
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2022 HelloWorld, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SYSTEM_H_
#define __SYSTEM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"
#include "arm_math.h"
#include "stm32h7xx_hal.h"
#include "usart.h"
#include "iwdg.h"
#include "fdcan.h"
#include "spi.h"


/* ! system options: user config --------------------------------------------*/
/* user usage */
#define USE_FREERTOS_CMSIS 0
#define USE_LOG 0
#define USE_SYS_TICK 1

/* system tick source */
#if (!USE_SYS_TICK)
// ! if not using systick as system timer, set to specific tim handle, e.g. htim2
#define SYS_TIM htim2
#endif

/* end of user config -------------------------------------------------------*/

/* device status */
typedef enum {
    DEV_OK,
    DEV_ERROR,
    DEV_BUSY,
    DEV_EXISTED,
    DEV_UNREGISTED,
    DEV_NOSTATE,
} DevStatus_t;

/* asserts */
#if USE_LOG
#include "log.h"
#define SYS_ASSERT(x)                                                              \
    if (!(x)) {                                                                    \
        LogAssert("System Config ERROR !!! %s %s %s", !(##x), __FILE__, __LINE__); \
        while (1)                                                                  \
            ;                                                                      \
    }

#define DEV_ASSERT(x)                                                             \
    if (!(x)) {                                                                   \
        LogAssert("Device Param ERROR !!! %s %s %s", !(##x), __FILE__, __LINE__); \
        while (1)                                                                 \
            ;                                                                     \
    }

#define ALG_ASSERT(x)                                                                \
    if (!(x)) {                                                                      \
        LogAssert("Algorithm Param ERROR !!! %s %s %s", !(##x), __FILE__, __LINE__); \
        while (1)                                                                    \
            ;                                                                        \
    }
#else
#define SYS_ASSERT(x) assert_param((x))
#define DEV_ASSERT(x) assert_param((x))
#define ALG_ASSERT(x) assert_param((x))
#endif

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
#define MALLOC malloc
#define FREE free
#define ENTER_CRITICAL() \
    do {                 \
        __disable_irq(); \
    } while (0);
#define EXIT_CRITICAL() \
    do {                \
        __enable_irq(); \
    } while (0);
#define Mutex_t bool
#define MUTEX_INIT() true
#define MUTEX_LOCK(x) \
    do {              \
        (x) = false;  \
    } while (0)
#define MUTEX_UNLOCK(x) \
    do {                \
        (x) = true;     \
    } while (0)
#endif

/* system tick */
static inline uint32_t GetTickMs(void)
{
    return (uint32_t)(HAL_GetTick() * HAL_GetTickFreq());
}

static inline uint32_t GetTickUs(void)
{
#if (USE_SYS_TICK)
    return (uint32_t)(GetTickMs() * 1000 + SysTick->VAL * 1000 / SysTick->LOAD);
#else
    SYS_ASSERT(&(SYS_TIM));
    return (uint32_t)(GetTickMs() * 1000 + __HAL_TIM_GET_COUNTER(&(SYS_TIM)) % 1000);
#endif
}

#ifdef __cplusplus
}
#endif

#endif /* __SYSTEM_H_ */