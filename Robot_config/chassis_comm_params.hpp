/**
 *******************************************************************************
 * @file      : chassis_comm_params.h
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
#ifndef __CHASSIS_COMM_PARAMS_H_
#define __CHASSIS_COMM_PARAMS_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "chassis_params.hpp"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

static const float kNoGyroMinSpeed = kMaxMovingSpeed * 1.1f;  // unit: m/s

static const uint16_t kBalancingHPThres = 250;
static const uint8_t kColorIDThres = 100;
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_COMM_PARAMS_H_ */
