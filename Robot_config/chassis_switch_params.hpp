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

static const float kSteerSwitchSpeedThres = 1.0f;   // unit: m/s
static const float kSteerSwitchYawOmgThres = 6.0f;  // unit: rad/s

static const float kGyroSwitchomg = 4.0f;  // unit: rad/s^2
static const float kGyroSwitchSpeed= 7.0f;  // unit: rad/s

static const float kSteerSwitchLegAngleThres =0.5f;  // unit: rad

static const float kPowerOnDelay = 3.0f;  // unit: s
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_SWITCh_PARAMS_HPP_ */
