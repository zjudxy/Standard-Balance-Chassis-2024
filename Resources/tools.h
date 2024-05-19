/**
 *******************************************************************************
 * @file      : tools.h
 * @brief     : tools
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
#ifndef __TOOLS_H_
#define __TOOLS_H_
/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Exported macro ------------------------------------------------------------*/

#ifndef M_PI
#define M_PI (3.14159265359f)
#endif

#define SIGN(x) ((x) > 0   ? 1  \
                 : (x) < 0 ? -1 \
                           : 0)

#define ANGLE_RANGE_HANDLE_RAD(x) \
    do {                          \
        while ((x) >= M_PI) {     \
            (x) -= 2 * M_PI;      \
        }                         \
        while ((x) < -M_PI) {     \
            (x) += 2 * M_PI;      \
        }                         \
    } while (0)

#define ANGLE_ACROSS0_HANDLE_RAD(ref, fdb) \
    do {                                   \
        while ((ref) - (fdb) >= M_PI) {    \
            (ref) -= 2 * M_PI;             \
        }                                  \
        while ((ref) - (fdb) < -M_PI) {    \
            (ref) += 2 * M_PI;             \
        }                                  \
    } while (0)

#define ANGLE_ACROSS0_HANDLE_DEGREE(ref, fdb) \
    do {                                      \
        while ((ref) - (fdb) >= 180) {        \
            (ref) -= 2 * 180;                 \
        }                                     \
        while ((ref) - (fdb) < -180) {        \
            (ref) += 2 * 180;                 \
        }                                     \
    } while (0)

#define LIMIT_MAX(data, lmt1, lmt2)       \
    do {                                  \
        if ((lmt1) <= (lmt2)) {           \
            if ((data) < (lmt1)) {        \
                (data) = (lmt1);          \
            } else if ((data) > (lmt2)) { \
                (data) = (lmt2);          \
            }                             \
        } else if ((lmt2) < (lmt1)) {     \
            if ((data) < (lmt2)) {        \
                (data) = (lmt2);          \
            } else if ((data) > (lmt1)) { \
                (data) = (lmt1);          \
            }                             \
        }                                 \
    } while (0)

/*
  y = ax + b
 */
#define LINEAR_FUNC(a, b, x) ((a) * (x) + (b))

/*          a
  y = ----------------
      1 + e ^ (bx + c)
 */
#define SIGMOID_FUNC(a, b, c, x) ((a) * (1.0f / (1.0f + exp((b) * (x) + (c)))))

/* Exported constants --------------------------------------------------------*/
static const int8_t kCcwDir = 1;
static const int8_t kCwDir = -1;
static const float kRpm2Radps = M_PI / 30.0f;
static const float kRadps2Rpm = 30.0f / M_PI;
static const float kMa2C620Tx = 16384.0f / 20000.0f;  ///< [-20, 20]A to [-16384, 16384]
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

/**
 * @brief       assert if given value is in the given range
 * @param       val:
 * @param       bound1:
 * @param       bound2:
 * @retval      true or false, type: bool
 * @note        None
 */
static inline bool IsInRange(float val, float bound1, float bound2)
{
    if (bound1 <= bound2)
        return val >= bound1 && val <= bound2;
    else
        return val <= bound1 && val >= bound2;
}

/**
 * @brief       get sign of variable
 * @param       val:
 * @retval      None
 * @note        None
 */
static inline int8_t GetSign(float val)
{
    if (!val)
        return 0;
    return val > 0 ? 1 : -1;
}


/**
 * @brief       compute polynomial
 * @param       *p: ptr to array of polynomial coefficient
 * @param       x: independent variable
 * @param       n: highest power of polynomial
 * @retval      None
 * @note        polynomial power from n to 0
 */
static float PolyVal(const float *p, float x, uint8_t n)
{
    float result = 0;
    for (uint8_t i = 0; i <= n; i++)
    {
        result = result * x + p[i];
    }

    return result;
}

/**
 * @brief       compute polynomial
 * @param       *p: ptr to array of polynomial coefficient
 * @param       *x: independent variable array
 * @param       *n: highest power of polynomial
 * @retval      None
 * @note        polynomial power from n to 0
 */
static float MultiPolyVal(const float *p, float *x, uint8_t* n)
{
    return 0;
}
#endif /* __TOOLS_H_ */