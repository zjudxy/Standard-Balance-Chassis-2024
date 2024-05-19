/**
 *******************************************************************************
 * @file      : chassis_switch_task.cpp
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
#include "chassis_switch_task.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
#include "chassis.hpp"
#include "chassis_task.hpp"
#include "chassis_switch_params.hpp"

static void DetectStateHandle(void);

static void ChassisRefModeGen(void);

static void ChassisModeSwitch(void);

static void SteerRefModeGen(void);

static void SteerModeSwitch(void);

void SwitchTask(void)
{
    DetectStateHandle();

    ChassisRefModeGen();

    ChassisModeSwitch();

    SteerRefModeGen();

    SteerModeSwitch();
}

bool abnormal_flag = false;
/**
 * @brief       检测状态处理
 * @retval      None
 * @note        None
 */
static void DetectStateHandle(void)
{
    abnormal_flag = false;
    if ((robot.detect_states.abnormal&&robot.detect_states.abnormal_type == Abnormal_dangerous) || robot.detect_states.low_battery)
    {
        robot.chassis_mode.ref = CHASSIS_RECOVERY;
        abnormal_flag = true;
    }

    else if (robot.chassis_mode.curr == CHASSIS_MATURE)
    {
        if (robot.steer_mode.curr == STEER_GYRO)
        {
            if (!robot.detect_states.on_ground[BODY])
            {
                robot.chassis_mode.ref = CHASSIS_FLY;
                abnormal_flag = true;
            }
        }
        else if (!robot.detect_states.on_ground[LEFT] || !robot.detect_states.on_ground[RIGHT])
        {
            robot.chassis_mode.ref = CHASSIS_FLY;
            abnormal_flag = true;
        }
    }
}
/**
 * @brief       生成底盘期望控制模式
 * @retval      None
 * @note        None
 */
static void ChassisRefModeGen(void)
{
    if (robot.Init_finish_flag == false)
    {
        robot.chassis_mode.ref = CHASSIS_INIT;
        return;
    }

    if (robot.cmd.chassis_mode_ref == robot.chassis_mode.ref||abnormal_flag)
    {
        return;
    }

    switch (robot.chassis_mode.curr)
    {
    case CHASSIS_INIT:
        return;
        break;
    case CHASSIS_DEAD:
        if (robot.cmd.chassis_mode_ref != CHASSIS_MATURE)
        {
            return;
        }
        break;
    case CHASSIS_CONFIRM:
    case CHASSIS_RECOVERY:
    case CHASSIS_FLY:
    case CHASSIS_JUMP:
        if (robot.cmd.chassis_mode_ref != CHASSIS_DEAD)
        {
            return;
        }
        break;
    case CHASSIS_MATURE:
        if (robot.cmd.chassis_mode_ref != CHASSIS_DEAD && robot.cmd.chassis_mode_ref != CHASSIS_JUMP)
        {
            return;
        }
    }
    robot.chassis_mode.ref = robot.cmd.chassis_mode_ref;
}
/**
 * @brief   有关底盘平衡控制的模式切换
 * @param   none
 * @retval  none
 * @note    none
 **/
static void ChassisModeSwitch(void)
{
    static uint16_t power_on_delay = 0;

    if (robot.chassis_mode.curr == CHASSIS_INIT)
    {
        return;
    }

    // if (!js_mains_power_chassis_output) {
    //     power_on_delay = 0;
    // } else if (power_on_delay * kCtrlPeriod <= kPowerOnDelay) {
    //     power_on_delay++;
    //     robot.chassis_mode.ref = robot.chassis_mode.curr = robot.chassis_mode.last = CHASSIS_DEAD;
    // }

    if (robot.chassis_mode.curr == robot.chassis_mode.ref)
    {
        return;
    }

    switch (robot.chassis_mode.ref)
    {
    case CHASSIS_DEAD:
        robot.chassis_mode.last = robot.chassis_mode.curr;
        robot.chassis_mode.curr = CHASSIS_DEAD;
        break;
    case CHASSIS_FLY:
        if (robot.chassis_mode.curr == CHASSIS_MATURE)
        {
            robot.chassis_mode.last = robot.chassis_mode.curr;
            robot.chassis_mode.curr = CHASSIS_FLY;
            
        }
        break;
    case CHASSIS_MATURE:
        if (robot.chassis_mode.curr == CHASSIS_DEAD)
        {
            robot.chassis_mode.last = robot.chassis_mode.curr;
            if (robot.joint_ang_cal_flag)
            {
                robot.chassis_mode.curr = CHASSIS_RECOVERY;
            }
            else
            {
                robot.chassis_mode.curr = CHASSIS_CONFIRM;
            }
        }
        break;
    case CHASSIS_JUMP:
        if (robot.chassis_mode.curr == CHASSIS_MATURE &&
            (robot.steer_mode.curr == STEER_MOVE || robot.steer_mode.curr == STEER_DEPART))
        {
            robot.chassis_mode.last = robot.chassis_mode.curr;
            robot.chassis_mode.curr = CHASSIS_JUMP;
        }
        break;
    case CHASSIS_RECOVERY:
        robot.chassis_mode.last = robot.chassis_mode.curr;
        robot.chassis_mode.curr = CHASSIS_RECOVERY;
        break;
    default:
        break;
    }
}
float debug_dpos = 0;
/**
 * @brief       底盘期望转向模式切换
 * @retval      None
 * @note        None
 */
static void SteerRefModeGen(void)
{
    static uint8_t last_gimbal_reverse_finish_seq;
    static uint8_t gimbal_reverse_flag = false;

    if (robot.chassis_mode.curr != CHASSIS_MATURE)
    {
        robot.steer_mode.ref = STEER_DEPART;
        return;
    }
    if (!robot.comm.gimbal2chassis.gimbal_reset)
    {
        robot.steer_mode.ref = STEER_DEPART;
        return;
    }

    // if(!robot.detect_states.gimbal_online){
    //     robot.steer_mode.ref = STEER_DEPART;
    // }

    if (robot.cmd.gimbal_reverse_seq != robot.comm.gimbal2chassis.reverse_finish_seq)
    {
        if (!gimbal_reverse_flag)
        {
            gimbal_reverse_flag = true;
            if (robot.steer_mode.curr == STEER_MOVE || robot.steer_mode.curr == STEER_DEFENSE)
            {
                robot.steer_mode.ref = STEER_DEPART;
            }

            robot.steer_mode.last = robot.steer_mode.curr;
        }
        return;
    }
    else
    {
        gimbal_reverse_flag = false;

        if (last_gimbal_reverse_finish_seq != robot.comm.gimbal2chassis.reverse_finish_seq)
        {
            robot.steer_mode.ref = robot.steer_mode.last;
            last_gimbal_reverse_finish_seq = robot.comm.gimbal2chassis.reverse_finish_seq;
            return;
        }
    }
    /* 当速度小于一定值时才生成期望转向模式 */
    float dpos = (robot.leg_states[LEFT].curr.state.dpos + robot.leg_states[RIGHT].curr.state.dpos) / 2.0;
    debug_dpos = dpos;
    if (fabs(dpos) > kSteerSwitchSpeedThres)
    {
        return;
    }
    float theta = (robot.leg_states[LEFT].curr.state.theta + robot.leg_states[RIGHT].curr.state.theta) / 2.0;
    if (fabs(theta) > kSteerSwitchLegAngleThres)
    {
        return;
    }

    robot.steer_mode.ref = robot.cmd.steer_mode_ref;
}

int test_flag = 0;
/**
 * @brief   有关底盘转向模式的切换
 * @param   none
 * @retval  none
 * @note    none
 **/
static void SteerModeSwitch(void)
{

    if (robot.chassis_mode.curr != CHASSIS_MATURE)
    {
        test_flag = 1;
        robot.steer_mode.last = robot.steer_mode.curr;
        robot.steer_mode.ref = robot.steer_mode.curr =  STEER_DEPART;
        
        return;
    }

    if (robot.steer_mode.curr == robot.steer_mode.ref)
    {
        test_flag = 0;
        return;
    }

    switch (robot.steer_mode.ref)
    {
    case STEER_DEPART:

        robot.chassis_states.yaw_ang = robot.imu_datas.euler_vals[YAW];
        robot.cmd.yaw_ang = robot.chassis_states.yaw_ang;
        robot.steer_mode.last = robot.steer_mode.curr;
        robot.steer_mode.curr = STEER_DEPART;
        break;
    case STEER_GYRO:
        if (robot.chassis_states.dpos < 0.3f)
        {
            robot.gyro_forward_angle = robot.yaw_motor->angle();
            robot.cmd.yaw_ang = robot.imu_datas.euler_vals[YAW];
            robot.chassis_states.yaw_ang = robot.imu_datas.euler_vals[YAW];
            robot.steer_mode.last = robot.steer_mode.curr;
            robot.steer_mode.curr = STEER_GYRO;
            robot.cmd.gyro_cal_flag = false;
            robot.cmd.gyro_dir = 0;
        }
        break;
    case STEER_MOVE:
        // 底盘临时切换，直接停
        if (robot.steer_mode.curr == STEER_GYRO)
        {
            if (fabsf(robot.chassis_states.yaw_omg) > kGyroSwitchSpeed)
            {
            }
            else if (abs(robot.yaw_motor->angle()) < 0.6f)
            {
                robot.chassis_states.yaw_ang = robot.yaw_motor->angle();
                robot.chassis_states.yaw_omg = robot.imu_datas.euler_vals[YAW];
                robot.cmd.yaw_ang = 0;
                robot.steer_mode.last = robot.steer_mode.curr;
                debug_num = 1;
                robot.steer_mode.curr = STEER_MOVE;
            }
            else if (robot.yaw_motor->angle() > PI - 0.6f)
            {
                robot.chassis_states.yaw_ang = robot.yaw_motor->angle();
                robot.chassis_states.yaw_omg = robot.imu_datas.euler_vals[YAW];
                robot.cmd.yaw_ang = PI;
                robot.steer_mode.last = robot.steer_mode.curr;
                debug_num = 2;
                robot.steer_mode.curr = STEER_MOVE;
            }
            else if (robot.yaw_motor->angle() < -PI + 0.6f)
            {
                robot.chassis_states.yaw_ang = robot.yaw_motor->angle();
                robot.chassis_states.yaw_omg = robot.imu_datas.euler_vals[YAW];
                robot.cmd.yaw_ang = PI;
                robot.steer_mode.last = robot.steer_mode.curr;
                debug_num = 3;
                robot.steer_mode.curr = STEER_MOVE;
            }
            else
            {
                robot.steer_mode.curr = STEER_GYRO;
            }
            break;
        }

        robot.chassis_states.yaw_ang = robot.yaw_motor->angle();
        robot.chassis_states.yaw_omg = robot.imu_datas.euler_vals[YAW];
        robot.cmd.yaw_ang = robot.chassis_states.yaw_ang;
        robot.steer_mode.last = robot.steer_mode.curr;
        debug_num = 4;
        robot.steer_mode.curr = STEER_MOVE;
        break;
    case STEER_DEFENSE:
        if (robot.steer_mode.curr == STEER_GYRO)
        { // 如果是从GYRO切换到DEFENSE模式，则需要等到朝向接近90°/-90°且旋转方向正确的时候，并设置参考角度为90°/-90°
            if (
                (robot.cmd.gyro_dir == kCcwDir &&
                 robot.yaw_motor->angle() > D2R(45) && robot.yaw_motor->angle() < D2R(60)) ||
                (robot.cmd.gyro_dir == kCwDir &&
                 robot.yaw_motor->angle() > D2R(120) && robot.yaw_motor->angle() < D2R(135)) ||
                (robot.cmd.gyro_dir == kCwDir &&
                 robot.yaw_motor->angle() < -D2R(45) && robot.yaw_motor->angle() > -D2R(60)) ||
                (robot.cmd.gyro_dir == kCcwDir &&
                 robot.yaw_motor->angle() < -D2R(120) && robot.yaw_motor->angle() > -D2R(135)))
            {
                robot.steer_mode.last = robot.steer_mode.curr;
                robot.steer_mode.curr = STEER_DEFENSE;
            }
        }
        else if (robot.steer_mode.curr == STEER_DEPART)
        {
            robot.cmd.yaw_ang = robot.yaw_motor->angle();
            robot.chassis_states.yaw_ang = robot.yaw_motor->angle();
            robot.steer_mode.last = robot.steer_mode.curr;
            robot.steer_mode.curr = STEER_DEFENSE;
        }
        else
        { // 如果是从其他切换到DEFENSE模式，直接切换即可，并设置当前角度为初始参考角度
            robot.steer_mode.last = robot.steer_mode.curr;
            robot.steer_mode.curr = STEER_DEFENSE;
        }
        break;
    default:
        break;
    }
}