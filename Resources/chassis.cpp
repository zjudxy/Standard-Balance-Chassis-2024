/**
 *******************************************************************************
 * @file      : chassis.cpp
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
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
#include "buzzer.hpp"
#include "chassis_music.hpp"
#include "tim.h"
BalanceRobot_t robot; // 底盘结构体

/**
 * @brief      机器人结构体初始化
 * @retval      None
 * @note        None
 */
void RobotInit(void)
{   
    robot.chassis_buzzer=new buzzer::Buzzer(&htim12, TIM_CHANNEL_2, buzzer::kPlayConfigSinglePlayback,
      &tune_list_info);
    // 电机初始化
    chassis_motor_Init();
    for (int i = 0; i < 6; i++)
    {
        robot.chassis_motors[i] = chassis_motor_ptr[i];
    }
    for(int i=0;i<4;i++){
        robot.chassis_motor_vel[i]=0;
    }
    robot.yaw_motor=yaw_motor_ptr;
    // 底盘状态初始化
    robot.chassis_mode.curr = robot.chassis_mode.ref = robot.chassis_mode.last = CHASSIS_INIT;
    robot.mode_state.init = INIT_STATE_INIT;
    robot.steer_mode.curr = robot.steer_mode.ref = robot.steer_mode.last = STEER_DEPART;
    // 腿关节初始化
    for (uint8_t i = 0; i < 2; i++)
    {
        memcpy(robot.leg_states[i].ref.data, kRefState, sizeof(LegState_u));
        memcpy(robot.leg_states[i].curr.data, kRefState, sizeof(LegState_u));
    }

    // 五连杆解算函数初始化
    FiveRodsInit(&robot.five_rods_cal, kJointMotorsDis, kThighLen, kLegLen);

    memset(&robot.chassis_states, 0, sizeof(ChassisStates_t));

    memset(&robot.detect_states, 0, sizeof(ChassisDetectStates_t));

    robot.chassis_states.height = kHeightMin;
    robot.chassis_states.dist2obs[0] = robot.chassis_states.dist2obs[1] = 100.0f;

    robot.detect_states.abnormal = false;
    robot.detect_states.on_ground = true;
    robot.detect_states.close2obs = false;
    robot.detect_states.low_battery = false;
    
    memset(&robot.imu_datas, 0, sizeof(ImuDatas_t));

    robot.control_tick = 0;

    ChassisGimbalCommInit(&robot.comm);

    // SupCapInit(&robot.sup_cap, kSupCapMaxVolt, kSupCapMinVolt);

    // TflunaInit(robot.dist_measurement + 0, F_DIST_MEASUREMENT_SCL_PORT, F_DIST_MEASUREMENT_SCL_PIN,
    //            F_DIST_MEASUREMENT_SDA_PORT, F_DIST_MEASUREMENT_SDA_PIN);
    // TflunaInit(robot.dist_measurement + 1, B_DIST_MEASUREMENT_SCL_PORT, B_DIST_MEASUREMENT_SCL_PIN,
    //            B_DIST_MEASUREMENT_SDA_PORT, B_DIST_MEASUREMENT_SDA_PIN);
    memset(&robot.cmd, 0, sizeof(Cmd_t));
    robot.cmd.height = kHeightMin;
    robot.cmd.height = kHeightMin;
    robot.cmd.speed_limit = true;
    robot.cmd.chassis_mode_ref = CHASSIS_DEAD;
    robot.cmd.steer_mode_ref = STEER_DEPART;
    robot.cmd.roll_ang_fix = true;
    robot.cmd_src = REMOTE; // 默认遥控器为控制源
    // robot.cmd.speed_limit=false;
    robot.cmd.ui_redraw = false;
    robot.gyro_forward_angle=0;
    robot.gyro_forward_speed=0;

    robot.referee_ptr = new RobotReferee();

    robot.sup_cap = new super_capacity(kSupCapMaxVolt, kSupCapMinVolt);

    // robot.ui_drawer = new UiDrawer();
    robot.ui_drawer = new Referee(default_ui_init_data);
}
