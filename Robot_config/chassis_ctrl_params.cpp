/**
*******************************************************************************
* @file      : <file>.c
* @brief     :
* @history   :
*  Version     Date            Author          Note
*  V0.9.0      yyyy-mm-dd      <author>        1. <note>
*******************************************************************************
* @attention :
*******************************************************************************
*  Copyright (c) 2022 Hello World Team, Zhejiang University.
*  All Rights Reserved.
*******************************************************************************
*/
/* Includes ------------------------------------------------------------------*/
#include "chassis_ctrl_params.hpp"
#include "pid.hpp"

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
namespace pid = hello_world::pid;
/* joint motor pid params */

const pid::BasicPid::Params kJointPidsParams[2] = {
    {.auto_reset = false,
     .kp = 8.0f,
     .ki = 0.0f,
     .kd = 0.0f,
     .max_interval_ms = 10,
     .period_sub = pid::PeriodSub(true, 2 * M_PI),
     .inte_anti_windup = pid::InteAntiWindup(true, -1.0f, 1.0f),
     .out_limit = pid::OutLimit(true, -20.0f, 20.0f)},
    {.auto_reset = false,
     .kp = 1.5f,
     .ki = 0.0001f,
     .kd = 0.0f,
     .max_interval_ms = 10,
     .inte_anti_windup = pid::InteAntiWindup(true, -1.0f, 1.0f),
     .out_limit = pid::OutLimit(true, -30.0f, 30.0f)}};

/* joint motor confirm pid params */

const pid::BasicPid::Params kJointConfirmPidsParams = {
    .auto_reset = false,
    .kp = 5.0f,
    .ki = 0.001f,
    .kd = 0.0f,
    .max_interval_ms = 10,
    .inte_anti_windup = pid::InteAntiWindup(true, -5.0f, 5.0f),
    .out_limit = pid::OutLimit(true, -6.0f, 6.0f)};
// /* wheel motor pid params */

const pid::BasicPid::Params kWheelPidsParams = {

    .kp = 0.5f,
    .ki = 0.000f,
    .kd = 20.0f,
    .max_interval_ms = 100,
    .inte_anti_windup = pid::InteAntiWindup(true, -1.0f, 1.0f),
    .out_limit = pid::OutLimit(true, -16.0f, 16.0f)};
// /* wheel motor pid params */

const pid::BasicPid::Params kWheelRecoveryPidsParams[2] = {
    {.kp = 3.0f,
     .ki = 0.05f,
     .kd = 0.0f,
     .max_interval_ms = 10,

     .period_sub = (pid::PeriodSub(true, 2 * M_PI)),
     .inte_anti_windup = pid::InteAntiWindup(true, -5.0f, 5.0f),

     .out_limit = pid::OutLimit(true, -10.0f, 10.0f)},
    {.kp = 4.0f,
     .ki = 0.002f,
     .kd = 0.0f,
     .max_interval_ms = 10,
     .inte_anti_windup = pid::InteAntiWindup(true, -5.0f, 5.0f),

     .out_limit = pid::OutLimit(true, -16.0f, 16.0f)

    }};

// /* virtual leg angle pid params */

const pid::BasicPid::Params kLegAngPidParams = {

    .kp = 25.0f,
    .ki = 0.01f,
    .kd = 1.0f,

    .inte_anti_windup = pid::InteAntiWindup(true, -13.0f, 13.0f),
    .out_limit = pid::OutLimit(true, -30.0f, 30.0f)};

// /* steer yaw additional torque pid params */

const pid::BasicPid::Params kYawTqPidParams[2] = {
    {.kp = 5.5f,
     .ki = 0.0f,
     .kd = 0.0f,

     .max_interval_ms = 100,

     //  .period_sub = pid::PeriodSub(true, 2 * M_PI),
     .inte_anti_windup = pid::InteAntiWindup(true, -1.0f, 1.0f),
     .out_limit = pid::OutLimit(true, -8.0f, 8.0f)},
    {
        .kp = 4.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .max_interval_ms = 100,
        .dead_band = pid::DeadBand(true, -D2R(3.0f), D2R(3.0f)),
        .inte_anti_windup = pid::InteAntiWindup(true, -2.0f, 2.0f),
        .out_limit = pid::OutLimit(true, -9.0f, 9.0f),
    }};

// const pid::BasicPid::Params kYawTqPidParams = {
//         .kp = 5.5f,
//         .ki = 0.0f,
//         .kd = 2.0f,
//         .max_interval_ms = 100,
//         .inte_anti_windup = pid::InteAntiWindup(true, -2.0f, 2.0f),
//         .out_limit = pid::OutLimit(true, -6.0f, 6.0f),
//     };

// /* chassis roll angle pid params */

const pid::BasicPid::Params kRollPidParams = {

    .kp = 400.0f,
    .ki = 0.0f,
    .kd = 200.0f,
    .period_sub = pid::PeriodSub(true, 2 * M_PI),
    .out_limit = pid::OutLimit(true, -250.0f, 250.0f),
    // .inte_anti_windup = pid::InteAntiWindup(true, -3.0f, 3.0f)
};

const pid::BasicPid::Params kYawMotorPidParams[2] = {
    {.kp = 18.5f,
     .ki = 0.0f,
     .kd = 0.0f,
     .max_interval_ms = 10,
     .period_sub = pid::PeriodSub(true, 2 * M_PI),
     .inte_anti_windup = pid::InteAntiWindup(true, -1.0f, 1.0f),
     .out_limit = pid::OutLimit(true, -30.0f, 30.0f)},
    {.kp = 1000.f,
     .ki = 0.0f,
     .kd = 0.0f,
     .max_interval_ms = 10,
     .inte_anti_windup = pid::InteAntiWindup(true, -5000.0f, 5000.0f),
     .out_limit = pid::OutLimit(true, -30000.0f, 30000.0f)}};

// const pid::BasicPid::Params kYawMotorPidParams = {

//     .kp = 1000.0f,
//     .ki = 0.0f,
//     .kd = 80.0f,
//     .period_sub = pid::PeriodSub(true, 2 * M_PI),
//     .out_limit = pid::OutLimit(true, -80.0f, 80.0f),
//     // .inte_anti_windup = pid::InteAntiWindup(true, -3.0f, 3.0f)
// };
