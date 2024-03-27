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

static const float32_t kGravAcc = 9.79f;  // unit: m/s^2

static const float32_t kWheelDiam = 0.150f;           // unit: m
static const float32_t kWheelBase = 0.50f;             //* unit: m
static const float32_t kBodyMass = 13.57245f;            //* unit: kg
static const float32_t kWheelMass = 0.65f;            // unit: kg
static const float32_t kLegMass = 0.97167f;               //* unit: kg
static const float32_t kBarycenterHeight = 0.13257f;  // unit: m

static const float32_t kJointMotorsDis = 0.150f;  // *unit: m
static const float32_t kThighLen = 0.130f;        // *unit: m
static const float32_t kLegLen = 0.25f;           // *unit: ms

// static const float32_t kMechanicalLimAng[6] = {-D2R(27.5), -D2R(152.5), -D2R(27.5), -D2R(152.5), 0, 0};

static const float32_t kMechanicalLimAng[6] = {-D2R(16), -D2R(166.9), -D2R(16), -D2R(166.9), 0, 0};

// static const float32_t kMechanicalLimAng[6] = {-D2R(23.89), -D2R(166.9), -D2R(23.89), -D2R(166.9), 0, 0};
static const float32_t kYawMotorAngOffset = -1.40836632;

static const float32_t kAbsMechanicalLimAng[2][2] = {{PI / 2, -D2R(30)}, {D2R(210), PI / 2}};

static const float32_t kJointMotorExReduRat = 1.0f;
static const float32_t kWheelMotorExReduRat = 13.0f / 3;

static const float32_t kSupCapMaxVolt = 26.6f;
static const float32_t kSupCapMinVolt = 14.0f;

static const float32_t kCtrlPeriod = 0.001f;    // unit: s
static const float32_t kMaxMovingSpeed = 2.2f;  // unit: m/s
static const float32_t kMaxLimSpeed = 2.1f;     // unit: m/s
static const float32_t kAutoJumpSpeed = 1.6f;   // unit: m/s (1.65m/s: 0.32)

static const float32_t kWheelMaxTq = 16.0f;    // unit: N·m
static const float32_t kWheelMaxCurr = 40.0f;  // unit: A
static const float32_t kJointMaxTq = 30.0f;    // unit: N·m

static const float32_t kHeightMax = 0.36f;   // unit: m
static const float32_t kHeightMid = 0.190f;  // unit: m
static const float32_t kHeightMin = 0.120f;   // unit: m

static const float kJumpHeightMid = 0.15f;   // unit: m
static const float kJumpHeightMax = 0.20f;   // unit: m
static const float kJumpHeightMin = 0.116f;  // unit: m

static const float kAirHeightMin = 0.11f;
static const float kAirHeightMax = 0.23f;

static const float32_t kGyroHeightMax = 0.24f;  // unit: m
static const float32_t kGyroHeightMin = 0.15f;  // unit: m

static const bool kWheelEncoderSide = FRONT;  // 编码器所在位置

/* reference state of LQR */
static const float32_t kRefState[STATE_NUM] = {0, 0, 0, 0, 0, 0, kHeightMin, 0};

/* Exported types ------------------------------------------------------------*/
#endif /* __FILE_H_ */
