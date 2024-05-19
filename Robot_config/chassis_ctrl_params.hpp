/** 
 *******************************************************************************
 * @file      : chassis_ctrl_params.hpp
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
#ifndef __chassis_ctrl_params_H_
#define __chassis_ctrl_params_H_


/* Includes ------------------------------------------------------------------*/
#include "chassis_ctrl_params.hpp"

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/


#include "system.hpp"
#include "pid.hpp"
#include "chassis.hpp"

#if defined(WHEEL_MOTOR_3508_120mm)
#include "motor_3508_120mm.hpp"
#elif defined(WHEEL_MOTOR_8910_200mm)
#include "motor_8910_200mm.hpp"
#elif defined(WHEEL_MOTOR_8910_150mm)
#include "motor_8910_150mm.hpp"
#endif


namespace pid = hello_world::pid;
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

static const float kRecoveryHeightThres = 0.05f;      // unit: m
static const float kRecoveryHeightTimeThres = 0.1f;   // unit: s
static const float kRecoveryLockTimeThres = 0.3f;   // unit: s
static const float kRecoveryThetaThres = D2R(10); // unit: rad
static const float kRecoveryDPosThres = 0.20f;   // unit: m/s

static const float kRecoveryStableDPhiThres = 1.0f;   // unit: rad/s
static const float kRecoveryStableDPosThres = 0.05f;  // unit: m/s
static const float kRecoveryStableTimeThres = 0.3f;   // unit: s
static const float kRecoveryStableJointvelThres = 0.3f;   // unit: s

static const float kComfirmMotorSpeed = 3.0f;      // unit: rad
static const float kComfirmStopSpeedThres = 0.3f;  // unit: rad
static const float kComfirmStopTimeThres = 1.0f;   // unit: s

static const float kJumpHeightThres = 0.02f;    // unit: m
static const float kJumpAirTimeThres = 0.055f;  // unit: s
static const float kJumpTq = 30.0f;             // unit: N·m

/* leg length control params */
static const float kLegLenCtrlK1 = 450;
static const float kLegLenCtrlK3 = 0;
static const float kLegLenCtrlB = 50;
static const float kLegLenCtrlKi = 2.0f;
static const float kLegLenCtrlMaxIout = 100.0f;
static const float kLegLenCtrlMaxOut = 350;
static const float kLegLenCtrlMinOut = -100;


static const float kMaxDDPos = 4.0f;  // unit: m/s

static const float kMaxDH = 0.30f;  // unit: m/s

static const float kAirDH = 0.60f;  // unit: m/s

static const float kPitchMaxDAng = 2.0f;  // unit: rad/s

static const float kPosMaxBias = 2.5f;     // unit: m
static const float kAbnormalPosBias = 0.3f;  //uint: m
static const float kAbnormalDPosBias = 5.0f;  //uint: m
static const float kRotPosMaxBias = 2.0f;  // unit: m

static const float kAirPosMaxBias = 1.4f;  // unit: m

static const float kSpeedDiffLimitSlope = 2.0f;

static const float kSlipWheelTorCoff = 0.1f; //打滑补偿系数
static const float kSlipWheelTorLimit = 2.0f; //打滑补偿力矩

/* joint motors contorl pid params */

extern const pid::MultiNodesPid::Params kJointPidsParams[2];
extern const pid::MultiNodesPid::Params kJointConfirmPidsParams;
extern const pid::MultiNodesPid::Params kWheelPidsParams;
extern const pid::MultiNodesPid::Params kLegAngPidParams;
extern const pid::MultiNodesPid::Params kYawTqPidParams[2];
// extern const pid::BasicPid::Params kYawTqPidParams;

extern const pid::MultiNodesPid::Params kRollPidParams;
extern const pid::MultiNodesPid::Params kYawMotorPidParams[2];
extern const pid::MultiNodesPid::Params kWheelRecoveryPidsParams[2];




#endif /* __FILE_H_ */
