/**
 *******************************************************************************
 * @file      : chassis_cmd_params.h
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
#ifndef __CHASSIS_CMD_PARAMS_H_
#define __CHASSIS_CMD_PARAMS_H_


/* Includes ------------------------------------------------------------------*/
#include "chassis_params.hpp"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

static const float kMaxGyroSpeed = 8.0f;  // unit: rad/s
static const float kMinGyroSpeed = 6.0f;   // unit: rad/s

static const float kMaxGyroAcc = 4.0f;     // unit: rad/s^2
static const float kMaxGyroForwardAcc = 1.0f;
static const float kTurnScale = 0.003f;
static float kMoveScaleCoff;

static const float kDepartTurnScale = 0.001f;

static const float kTuneTurnScale = 0.001f;
static const float kTuneMoveScale = 0.6f;

static const float kMouseMoveLim = 50.0f;

static const float kHeightAdjDiff = 0.1f;  // unit: m/s

static const float kMaxRotSpeed = 6.0f;  // unit: rad/s

static const float kYawAngRefMaxBias = D2R(70);  // unit: rad

static const float kSteerModeSwitchHeightMin = 0.13f;
static const float kSteerModeSwitchHeightMax = 0.26f;

static const float kRollMaxAng = 0.2f;   // unit: rad
static const float kRollAngDiff = 1.0f;  // unit: rad/s

static const float kAutoModeDeadZoneTimeThres = 0.05f;  // unit: s

// static const float kGyrotimeafter=0.422f;  // unit: s

float kGyrotimeafter=0.720261867f;  // unit: s

/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/


#endif /* __CHASSIS_CMD_PARAMS_H_ */
