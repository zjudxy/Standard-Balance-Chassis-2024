/**
 *******************************************************************************
 * @file      : chassis_sense_params.h
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
#ifndef __CHASSIS_SENSE_PARAMS_H_
#define __CHASSIS_SENSE_PARAMS_H_


/* Includes ------------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

static const float kImuTempSp = 35;

static const float kDThetaTdR = 250;
static const float kDHeightTdR = 250;
static const float kDOmgErrTdR = 20;

static const float kDDThetaTdR = 200;
static const float kDDPhiTdR = 200;
static const float kDDHeightTdR = 450;
static const float kDPosTdR = 150;

static const float kSlipDOmgErrThres = 5.5f;
static const float kEndSlipOmgErrThres = 3.0f;
static const float kEndSlipDPosThres = 0.5f;

static const float kTransRollBias = -0.5f;  // unit: degree
static const float kTransPitchBias = 0.2f;  // unit: degree

static const float kNormalMaxYawOmg = 8;  // unit: rad/s

static const float kAmpHighThres = 30000;
static const float kAmpLowThres = 100;

static const float kNoObsDist = 3.0f;

static float kTaKa[9] =
    {0.9588646291, -0.0066800186, -0.0495182314,
     0.0000000000, 0.9520669928, -0.0332895141,
     0.0000000000, 0.0000000000, 0.8938214731};

static const float kBa[3] =
    {-0.4929776474,
     -0.4717978958,
     -1.1511527453};

static float kTgKg[9] =
    {1.0000000000, -0.0756915911, -1.0227322914,
     0.0148852095, 1.0000000000, 0.0721415547,
     -0.1258179088, 0.1660858755, 1.0000000000};
static const float kBg[3] =
    {0.0082027854,
     0.0030427904,
     0.0029046593};

static float kTrans[9] =
    {-1, 0, 0,
     0, 1, 0,
     0, 0, -1};

// static const PidParams_t kImuTempPidParams = {
//     .kType = PID_DEFAULT,
//     .kImprvOption = INTE_SEPARATION,
//     .kp = 4000.0f,
//     .ki = 0.6f,
//     .kd = 0.0f,
//     .kOutMax = 65535,
//     .kOutMin = 0,
//     .kWindUpOutMax = 25000,
//     .kISeparThresLower = -7,
//     .kISeparThresUpper = 7,
// };
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/


#endif /* __CHASSIS_SENSE_PARAMS_H_ */
