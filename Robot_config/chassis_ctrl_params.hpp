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


namespace pid = hello_world::pid;
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

static const float kRecoveryHeightThres = 0.05f;      // unit: m
static const float kRecoveryHeightTimeThres = 0.1f;   // unit: s
static const float kRecoveryStableDPhiThres = 1.0f;   // unit: rad/s
static const float kRecoveryStableDPosThres = 0.05f;  // unit: m/s
static const float kRecoveryStableTimeThres = 0.2f;   // unit: s
static const float kRecoveryStableJointvelThres = 0.3f;   // unit: s

static const float kComfirmMotorSpeed = 3.0f;      // unit: rad
static const float kComfirmStopSpeedThres = 0.3f;  // unit: rad
static const float kComfirmStopTimeThres = 1.0f;   // unit: s

static const float kJumpHeightThres = 0.02f;    // unit: m
static const float kJumpAirTimeThres = 0.055f;  // unit: s
static const float kJumpTq = 30.0f;             // unit: N·m

/* leg length control patams */
static const float kLegLenCtrlK1 = 700;
static const float kLegLenCtrlK3 = 0;
static const float kLegLenCtrlB = 70;
static const float kLegLenCtrlKi = 1.5f;
static const float kLegLenCtrlMaxIout = 10.0f;
static const float kLegLenCtrlMaxOut = 350;
static const float kLegLenCtrlMinOut = -100;

static const float kMaxDH = 0.19f;  // unit: m/s

static const float kAirDH = 0.45f;  // unit: m/s

static const float kPitchMaxDAng = 2.0f;  // unit: rad/s

static const float kPosMaxBias = 2.5f;     // unit: m
static const float kAirPosMaxBias = 1.4f;  // unit: m

static const float kSpeedDiffLimitSlope = 2.0f;


/* mature contorller LQR matrix polynomial coefficient */
static const float kMatureLqrPoly[2][6][4] = {
    {
        {-353.691, 315.815, -94.7964, 8.0691},
        {-337.787, 302.598, -91.582, 7.558},
        {-157.637, 146.467, -40.2765, 11.0255},
        {-29.7111, 31.9336, -14.4021, 1.48083},
        {-708.195, 606.334, -179.963, -59.0952},
        {-145.318, 124.718, -36.5275, -9.83445},
    },
    {
        {-28.0825, 19.5779, -4.21477, -12.1922},
        {-6.1386, 2.10639, -4.49391, -11.6029},
        {-203.352, 228.758, -160.942, -3.52184},
        {4.4762, -10.6729, -16.7839, -0.374223},
        {2208.19, -2015.49, 637.802, -76.8586},
        {392.279, -356.277, 110.496, -14.1702},
    },
};


/* joint motors contorl pid params */

extern const pid::MultiNodesPid::Params kJointPidsParams[2];
extern const pid::MultiNodesPid::Params kJointConfirmPidsParams;
extern const pid::MultiNodesPid::Params kWheelPidsParams;
extern const pid::MultiNodesPid::Params kLegAngPidParams;
extern const pid::MultiNodesPid::Params kYawTqPidParams[2];
extern const pid::MultiNodesPid::Params kRollPidParams;
extern const pid::MultiNodesPid::Params kYawMotorPidParams[2];
extern const pid::MultiNodesPid::Params kWheelRecoveryPidsParams[2];
#endif /* __FILE_H_ */
