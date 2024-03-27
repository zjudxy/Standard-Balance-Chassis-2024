/** 
 *******************************************************************************
 * @file      : chassis_detect_params.hpp
 * @brief     : 
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0     yyyy-mm-dd        dxy           <note> 
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CHASSIS_DETECT_PARAMS__
#define __CHASSIS_DETECT_PARAMS__
/* Includes ------------------------------------------------------------------*/
#include "chassis.hpp"
/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

static const float kOffGroundTimeThres = 0.010f;  // unit: s
static const float kOnGroundThres = 0.8f;
static const float kOnGroundTimeThres = 0.1f;  // unit: s

// static const float kAbnormalTimeThres = 0.15f;     // unit: s
// static const float kThetaAbnormalThres = 0.65f;  // unit: rad
// static const float kDThetaAbnormalThres = 5.0f;    // unit: rad/s
// static const float kPhiAbnormalThres = 0.37f;      // unit: rad
// static const float kDPhiAbnormalThres = 2.0f;      // unit: rad/s
static const float kWheelAbnormalThres = 65.0f;      // unit: rad/s


static const float kAbnormalTimeThres = 0.15f;     // unit: s
static const float kThetaAbnormalThres = D2R(45);  // unit: rad
static const float kDThetaAbnormalThres = 5.0f;    // unit: rad/s
static const float kPhiAbnormalThres = 0.37f;      // unit: rad
static const float kDPhiAbnormalThres = 2.0f;      // unit: rad/s

static const float kClose2ObsDistThres = 0.40f;  // unit: m

static const float kLowBatteryPctThres = 0.2f;
static const float kLowBatteryPctTimeThres = 0.05f;  // unit: s

static const float kNormalMaxYawOmg = 8;  // unit: rad/s


static const float kSlipDOmgErrThres = 5.5f;

static const float kEndSlipOmgErrThres = 3.0f;

static const float kEndSlipDPosThres = 0.5f;

static const float kSlipDDPosThres = 2500.0f;  // unit: s

static const float kEndSlipDDPosThres =1500.0f;  // unit: m



#endif /* __CHASSIS_DETECT_PARAMS__ */
