/** 
 *******************************************************************************
 * @file      : chassis_detect_task.cpp
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
/* Includes ------------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
 *******************************************************************************
 * @file      : chassis_detect_task.cpp
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
/* Includes ------------------------------------------------------------------*/
#include "chassis.hpp"
#include "chassis_detect_task.hpp"
#include "chassis_detect_params.hpp"
#include "chassis_sense_task.hpp"
#include "td.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t abnormal_count = 0;
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

namespace TD = hello_world::filter;

void GroundDetect();

void AbnormalDetect(void);

void SlipDetect(void);

void PowerDetect(void);

TD::Td *domg_err_td = nullptr;

/**
 * @brief       init detect task
 * @retval      None
 * @note        None
 */
void DetectInit(void)
{
    robot.detect_states.on_ground=true;
    domg_err_td = new TD::Td(100.0f, 0.001f);
}



/**
 * @brief       task to detect contact state with the ground and abnormal state
 * @arg         None
 * @note        None
 */
void DetectTask(void)
{
    PowerDetect();

    GroundDetect();
    
    AbnormalDetect();

    SlipDetect();

}

/**
 * @brief       detect abnormal state
 * @retval      None
 * @note        use when chassis mode is MATURE
 */
void AbnormalDetect(void)
{
    if (robot.chassis_mode.curr != CHASSIS_MATURE) {
        abnormal_count = 0;
        robot.detect_states.abnormal = false;
        return;
    }

    /* enter chassis mode change to RECOVERY when abnormal counter reaches the threshold */
    if (abnormal_count * kCtrlPeriod > kAbnormalTimeThres) {
        abnormal_count = 0;
        robot.detect_states.abnormal = true;
    } else {
        if (
            fabsf(robot.leg_states[LEFT].curr.state.theta) > kThetaAbnormalThres ||
            fabsf(robot.leg_states[RIGHT].curr.state.theta) > kThetaAbnormalThres ||
            fabsf(robot.leg_states[LEFT].curr.state.dtheta) > kDThetaAbnormalThres ||
            fabsf(robot.leg_states[RIGHT].curr.state.dtheta) > kDThetaAbnormalThres ||
            fabsf(robot.leg_states[LEFT].curr.state.dphi) > kDPhiAbnormalThres ||
            fabsf(robot.leg_states[RIGHT].curr.state.dphi) > kDPhiAbnormalThres ||
            fabsf(robot.leg_states[LEFT].curr.state.phi) > kPhiAbnormalThres ||
            fabsf(robot.leg_states[RIGHT].curr.state.phi) > kPhiAbnormalThres
            // ||fabs(robot.chassis_motor_vel[LWM])>kWheelAbnormalThres
            // ||fabs(robot.chassis_motor_vel[RWM])>kWheelAbnormalThres
            ) {
            abnormal_count++;
        } else {
            abnormal_count /= 2;
        }
    }
}


/** 
 * @brief      离地检测
 * @retval      None
 * @note        None
 */
void GroundDetect()
{
    static uint16_t on_ground_count = 0;
    static uint16_t off_ground_count = 0;
    static float last_uppprt_force = 0;
    float support_force = robot.chassis_states.support_forces[LEFT] + robot.chassis_states.support_forces[RIGHT];

    if (off_ground_count * kCtrlPeriod > kOffGroundTimeThres)
    {
        off_ground_count = 0;
        robot.detect_states.on_ground = false;
    }
    else if (support_force < kBodyMass * kGravAcc * 0.05f)
    {
        on_ground_count = 100;
        off_ground_count++;
    }
    else if (support_force > kBodyMass * kGravAcc * 0.5f)
    {
        off_ground_count /= 2;
    }

    if (support_force > kBodyMass * kGravAcc * 0.8f || on_ground_count <= 0)
    {
        on_ground_count = 0;
        robot.detect_states.on_ground = true;
    }
    else if (support_force > kBodyMass * kGravAcc * 0.5f)
    {
        on_ground_count--;
    }
    last_uppprt_force = support_force;
}


// void SlipDetect(void)
// {
//     static uint8_t slip_flag_omg = 0;
//     static float omg_err = 0;

//     float rot_dpos[2];
//     //车身旋转产生的轮速
//     rot_dpos[1] = robot.imu_datas.gyro_vals[YAW] * kWheelBase / 2;
//     rot_dpos[0] = -rot_dpos[1];
    
//     float psi, dpsi, sPsi, cPsi, trans[2];


//     for (uint8_t i = 0; i < 2; i++) {

//         //计算关节偏角
//         psi = PI / 2 + robot.leg_states[i].curr.state.theta - robot.leg_states[i].curr.state.phi;
//         //计算关节偏角变化率
//         dpsi = robot.leg_states[i].curr.state.dtheta - robot.leg_states[i].curr.state.dphi;
//         arm_sin_cos_f32(R2D(psi), &sPsi, &cPsi);
//         //计算车身旋转产生的前进速度
//         trans[i] = rot_dpos[i] - robot.leg_states[i].curr.state.dheight * sPsi -
//                    robot.leg_states[i].curr.state.height * cPsi * dpsi;
//     }
//     //计算由于打滑导致的角速度误差
//     float omg_pred = ((robot.leg_states[RIGHT].curr.state.dpos - trans[RIGHT]) -
//                       (robot.leg_states[LEFT].curr.state.dpos - trans[LEFT])) /
//                      kWheelBase;

//     //计算差值
//     omg_err = 0.9  * (robot.imu_datas.gyro_vals[YAW] - omg_pred)+0.1* omg_err;

//     //跟踪微分
//     float domg_err = 0;
//     domg_err_td->calc(&omg_err,&domg_err);

//     if (fabsf(robot.imu_datas.gyro_vals[YAW]) < kNormalMaxYawOmg) {
//         if (fabsf(domg_err) > kSlipDOmgErrThres) {
//             slip_flag_omg = 1;
//         } else if (fabsf(omg_err) < kEndSlipOmgErrThres) {
//             slip_flag_omg = 0;
//         }
//     } else {
//         slip_flag_omg = 0;
//     }

//     if (slip_flag_omg) {
//         float pos_bias[2] = {
//             arm_sin_f32(robot.leg_states[LEFT].curr.state.theta) * robot.leg_states[LEFT].curr.state.height +
//                 robot.imu_datas.euler_vals[YAW] * kWheelBase / 2,
//             arm_sin_f32(robot.leg_states[RIGHT].curr.state.theta) * robot.leg_states[RIGHT].curr.state.height -
//                 robot.imu_datas.euler_vals[YAW] * kWheelBase / 2};
//         if (fabsf(robot.leg_states[LEFT].curr.state.dpos) > fabsf(robot.leg_states[RIGHT].curr.state.dpos)) {
//             robot.leg_states[LEFT].curr.state.dpos =
//                 robot.leg_states[RIGHT].curr.state.dpos - robot.imu_datas.gyro_vals[YAW] * kWheelBase;
//         } else {
//             robot.leg_states[RIGHT].curr.state.dpos =
//                 robot.leg_states[LEFT].curr.state.dpos + robot.imu_datas.gyro_vals[YAW] * kWheelBase;
//         }
//     }
// }

void PowerDetect(void){
    if(!robot.Init_finish_flag){
        return;
    }

    if(!robot.referee_ptr->PERFORMANCE->getData().power_management_chassis_output){
        robot.chassis_mode.ref = CHASSIS_DEAD;
    }
    if(robot.cmd.gimbal_enable&&!robot.referee_ptr->PERFORMANCE->getData().power_management_gimbal_output){
        robot.cmd.gimbal_enable = false;
    }

    if(robot.cmd.shooter_enable&&!robot.referee_ptr->PERFORMANCE->getData().power_management_shooter_output){
        robot.cmd.shooter_enable = false;
    }

}


void SlipDetect(void)
{
    if(robot.chassis_mode.curr==CHASSIS_MATURE&&robot.mature_tick>1000)
    {
        robot.detect_states.slip[0] = false;
        robot.detect_states.slip[1] = false;
        if (abs(leg_end_ddpos[0])>kSlipDDPosThres) {
            robot.leg_states[LEFT].curr.state.dpos =
                robot.leg_states[RIGHT].curr.state.dpos - robot.imu_datas.gyro_vals[YAW] * kWheelBase;
                robot.detect_states.slip[0] = true;

        } else if(abs(leg_end_ddpos[1])>kSlipDDPosThres)
        {
            robot.leg_states[RIGHT].curr.state.dpos =
                robot.leg_states[LEFT].curr.state.dpos + robot.imu_datas.gyro_vals[YAW] * kWheelBase;
                robot.detect_states.slip[1] = true;
        }
        // SlipDetect();
    }
    else{
        robot.detect_states.slip[0] = false;
        robot.detect_states.slip[1] = false;
    }
}