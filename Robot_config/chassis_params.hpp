/** 
 *******************************************************************************
 * @file      : chassis_params.hpp
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
#ifndef __CHASSIS_PARAMS__H_
#define __CHASSIS_PARAMS__H_
/* Includes ------------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#include "system.h"
#include "five_rods_calc.h"
#include "chassis_motor.hpp"
#include "chassis.hpp"
/* Exported macro ------------------------------------------------------------*/
#define MOTOR_NUM 6

#define STATE_NUM 8  // leg state numbers
#define POS 0
#define DOT_POS 1
#define THETA 2
#define DOT_THETA 3
#define PHI 4
#define DOT_PHI 5
#define HEIGHT 6
#define DOT_HEIGHT 7

#define LEFT 0
#define RIGHT 1
#define BODY 2
#define FRONT 0
#define BEHIND 1

#define SUP_CAP_RX_ID 0x111
#define SUP_CAP_TX_ID 0x112

#define F_DIST_MEASUREMENT_SCL_PORT GPIOB
#define F_DIST_MEASUREMENT_SCL_PIN GPIO_PIN_14
#define F_DIST_MEASUREMENT_SDA_PORT GPIOF
#define F_DIST_MEASUREMENT_SDA_PIN GPIO_PIN_0

#define B_DIST_MEASUREMENT_SCL_PORT GPIOB
#define B_DIST_MEASUREMENT_SCL_PIN GPIO_PIN_15
#define B_DIST_MEASUREMENT_SDA_PORT GPIOF
#define B_DIST_MEASUREMENT_SDA_PIN GPIO_PIN_1
/* Exported constants --------------------------------------------------------*/

static const float32_t kGravAcc = 9.81f;  // unit: m/s^2
#if defined(WHEEL_MOTOR_3508_120mm)
static const float32_t kWheelDiam = 0.120f;           // unit: m
static const float32_t kWheelMass = 0.62030f;            // unit: kg
static const float32_t kLegMass = 1.1149f;               //* unit: kg
static const float32_t kMaxMovingSpeed = 2.8f;  // unit: m/s
static const float32_t kMaxLimSpeed = 2.7f;     // unit: m/s
static const float kJumpHeightMin = 0.150f;  // unit: m
static const float32_t kMechanicalLimAng[6] = {-D2R(27.5), -D2R(152.5), -D2R(27.5), -D2R(152.5), 0, 0};
#elif defined(WHEEL_MOTOR_8910_200mm)
static const float32_t kWheelDiam = 0.200f;           // unit: m
static const float32_t kWheelMass = 1.06863f;            // unit: kg
static const float32_t kLegMass = 1.06025f;               //* unit: kg
static const float32_t kMaxMovingSpeed = 2.8f;  // unit: m/s
static const float32_t kMaxLimSpeed = 3.2f;     // unit: m/s
static const float kJumpHeightMin = 0.130f;  // unit: m
static const float32_t kMechanicalLimAng[6] = {-D2R(27.5+30), -D2R(152.5-30), -D2R(27.5+30), -D2R(152.5-30), 0, 0};
#elif defined(WHEEL_MOTOR_8910_150mm)
static const float32_t kWheelDiam = 0.150f;           // unit: m
static const float32_t kWheelMass = 0.86347f;            // unit: kg
static const float32_t kLegMass = 1.06025f;               //* unit: kg
static const float32_t kMaxMovingSpeed = 2.8f;  // unit: m/s
static const float32_t kMaxLimSpeed = 2.8f;     // unit: m/s
static const float kJumpHeightMin = 0.180f;  // unit: m
static const float32_t kMechanicalLimAng[6] = {-D2R(27.5), -D2R(152.5), -D2R(27.5), -D2R(152.5), 0, 0};
#endif



static const float32_t kWheelBase = 0.50f;             //* unit: m
static const float32_t kBodyMass = 14.01f;            //* unit: kg


static const float32_t kBarycenterHeight = 0.02398f;  // unit: m

static const float32_t kJointMotorsDis = 0.150f;  // *unit: m
static const float32_t kThighLen = 0.130f;        // *unit: m
static const float32_t kLegLen = 0.25f;           // *unit: ms

static const float32_t kDistSensor2Jointdisty = 0.125f;  // unit: m
static const float32_t kDistSensor2Jointdistx = 0.07f;    // unit: m
static const float32_t kDistSensorangleoffset = 0.0f; //uint: rad



// static const float32_t kMechanicalLimAng[6] = {-D2R(16), -D2R(166.9), -D2R(16), -D2R(166.9), 0, 0};

// static const float32_t kMechanicalLimAng[6] = {-D2R(23.89), -D2R(166.9), -D2R(23.89), -D2R(166.9), 0, 0};
static const float32_t kYawMotorAngOffset = -1.40836632;

static const float32_t kAbsMechanicalLimAng[2][2] = {{PI / 2, -D2R(30)}, {D2R(210), PI / 2}};

static const float32_t kJointMotorExReduRat = 1.0f;
static const float32_t kWheelMotorExReduRat = 13.0f / 3;

static const float32_t kSupCapMaxVolt = 26.6f;
static const float32_t kSupCapMinVolt = 14.0f;

static const float32_t kCtrlPeriod = 0.001f;    // unit: s

static const float32_t kAutoJumpSpeed = 1.6f;   // unit: m/s (1.65m/s: 0.32)

static const float32_t kWheelMaxTq = 16.0f;    // unit: N·m
static const float32_t kWheelMaxCurr = 40.0f;  // unit: A
static const float32_t kJointMaxTq = 30.0f;    // unit: N·m

static const float32_t kHeightMax = 0.36f;   // unit: m
static const float32_t kHeightMid = 0.21f;  // unit: m
static const float32_t kHeightMin = 0.110f;   // unit: m

static const float32_t kCmdHeightMax = 0.34f;   // unit: m
static const float32_t kCmdHeightMid = 0.210f;  // unit: m
static const float32_t kCmdHeightMin = 0.125f;   // unit: m

static const float kJumpHeightMid = 0.21f;   // unit: m
static const float kJumpHeightMax = 0.37f;   // unit: m


static const float kAirHeightMin = 0.115f;
static const float kAirHeightMax = 0.33f;

static const float kAirDiffHeight = 0.05f;

static const float32_t kGyroHeightMax = 0.19f;  // unit: m
static const float32_t kGyroHeightMin = 0.13f;  // unit: m

static const bool kWheelEncoderSide = FRONT;  // 编码器所在位置

/* reference state of LQR */
static const float32_t kRefState[STATE_NUM] = {0, 0, 0, 0, 0, 0, kCmdHeightMin, 0};

/* Exported types ------------------------------------------------------------*/
#endif /* __FILE_H_ */
