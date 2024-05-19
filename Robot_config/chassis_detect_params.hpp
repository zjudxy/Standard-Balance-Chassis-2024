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

static const float kOffGroundTimeThres = 0.03f;  // unit: s
static const float kOnGroundThres = 0.8f;
static const float kOnGroundTimeThres = 0.02f;  // unit: s

// static const float kAbnormalTimeThres = 0.15f;     // unit: s
// static const float kThetaAbnormalThres = 0.65f;  // unit: rad
// static const float kDThetaAbnormalThres = 5.0f;    // unit: rad/s
// static const float kPhiAbnormalThres = 0.37f;      // unit: rad
// static const float kDPhiAbnormalThres = 2.0f;      // unit: rad/s
static const float kWheelAbnormalThres = 8.0f;      // unit: m/s


static const float kAbnormalTime1Thres = 0.20f;     // unit: s
static const float kAbnormalTime2Thres = 0.2f;     // unit: s


#if defined(WHEEL_MOTOR_3508_120mm)
static const float kThetaAbnormalThres = D2R(42.0f);  // unit: rad
#elif defined(WHEEL_MOTOR_8910_200mm)
static const float kThetaAbnormalThres = D2R(50.0f);  // unit: rad
#elif defined(WHEEL_MOTOR_8910_150mm)
static const float kThetaAbnormalThres = D2R(50.0f);  // unit: rad
#endif



// static const float kThetaAbnormalThres = D2R(50.0f);  // unit: rad
static const float kDThetaAbnormalThres = 20.0f;    // unit: rad/s
static const float kPhiAbnormalThres = 0.39f;      // unit: m
static const float kDPhiAbnormalThres = 2.0f;      // unit: rad/s

static const float kClose2ObsDistThres = 0.38f;  // unit: m
static const float kDisemeasure2ArmorDis = 0.23f;  // unit: m
static const float kClose2ObsTimeThres = 0.020f;  // unit: s

static const float kLowBatteryPctThres = 0.2f;
static const float kLowBatteryPctTimeThres = 0.05f;  // unit: s

static const float kNormalMaxYawOmg = 8;  // unit: rad/s


static const float kSlipDOmgErrThres = 5.5f;

static const float kEndSlipOmgErrThres = 3.0f;

static const float kEndSlipDPosThres = 0.5f;

static const float kSlipDDPosThres = 2500.0f;  // unit: s

static const float kEndSlipDDPosThres =1500.0f;  // unit: m



#endif /* __CHASSIS_DETECT_PARAMS__ */
