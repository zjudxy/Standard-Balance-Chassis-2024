/**
 *******************************************************************************
 * @file      : assert.hpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2023 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_TOOLS_ASSERT_HPP_
#define HW_COMPONENTS_TOOLS_ASSERT_HPP_

/* Includes ------------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#ifndef USE_CUSTOM_DEFINE_ASSERT
#define USE_CUSTOM_DEFINE_ASSERT 0  //< 启用自定义 assert
#endif

#if USE_CUSTOM_DEFINE_ASSERT

/**
 * @brief       进行变量检查
 * @param        expr: 待检查表达式
 * @param        format: 格式化字符串
 * @retval       None
 * @note        None
 */
#define HW_ASSERT(expr, format, ...) \
  ((expr) ? (void)0U : Loop())
#else
#define HW_ASSERT(expr, format, ...) ((void)0U)
#endif
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

static inline void Loop(void)
{
  while (1) {
  }
}

#endif /* HW_COMPONENTS_TOOLS_ASSERT_HPP_ */
