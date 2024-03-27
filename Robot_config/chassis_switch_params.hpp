/**
 *******************************************************************************
 * @file      : chassis_switch_params.h
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
#ifndef __CHASSIS_SWITCh_PARAMS_HPP_
#define __CHASSIS_SWITCh_PARAMS_HPP_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

static const float kSteerSwitchSpeedThres = 0.3f;   // unit: m/s
static const float kSteerSwitchYawOmgThres = 6.0f;  // unit: rad/s

static const float kPowerOnDelay = 3.0f;  // unit: s
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_SWITCh_PARAMS_HPP_ */
