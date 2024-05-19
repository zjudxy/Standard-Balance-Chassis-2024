/**
 *******************************************************************************
 * @file      : chassis_comm_task.cpp
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

#include "chassis_comm_task.hpp"
#include "chassis.hpp"
#include "HW_fdcan.hpp"
#include "chassis_task.hpp"
#include "chassis_comm_params.hpp"
#include "judge.hpp"
// #include "Ref.h"

uint8_t referee_rx_data[255];
uint8_t referee_tx_data[255];

uint8_t dist_measure_rx_data_Front[1024];
uint8_t dist_measure_rx_data_Back[1024];

static void SendCanMsg(void);

static inline void InfantryTypeAnalysis(void);

static void GetJudge2GimbalInfo(void);

static void GetChassis2GimbalCmd(void);

static void UiTask(void);

void CommInit()
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, referee_rx_data, robot.referee_ptr->get_rx_dma_len());
    HAL_UARTEx_ReceiveToIdle_DMA(&huart7, dist_measure_rx_data_Front,1024);

    HAL_UARTEx_ReceiveToIdle_DMA(&huart10, dist_measure_rx_data_Back,1024);
}
hello_world::referee::RobotPerformancePackage::Data robot_performance_data;
hello_world::referee::RobotShooterPackage::Data robot_shoot_data;
hello_world::referee::RobotPowerHeatPackage::Data robot_power_heat_data;

void CommTask(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, referee_rx_data, robot.referee_ptr->get_rx_dma_len());
    HAL_UARTEx_ReceiveToIdle_DMA(&huart7, dist_measure_rx_data_Front,1024);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart10, dist_measure_rx_data_Back,1024);

    HAL_UART_Receive_IT(&huart7, dist_measure_rx_data_Front,1);
    SendCanMsg();
    robot_performance_data = robot.referee_ptr->PERFORMANCE->getData();
    robot_shoot_data = robot.referee_ptr->SHOOTER->getData();
    robot_power_heat_data = robot.referee_ptr->POWER->getData();
}

int encode_success_flag = 1;

/// @brief fdcan1 关节电机发送 fdcan2 轮电机、云台板通讯
/// @param  None
static void SendCanMsg(void)
{
    static uint8_t tx_msg[8];
    static uint32_t tx_id;
    encode_success_flag = 1;

    if (robot.cmd.start_comm)
    {
        memset(tx_msg, 0, 8);
        GetChassis2GimbalCmd();
        robot.comm.chassisEncode(&robot.comm, tx_msg);
        FDCAN_Send_Msg(&hfdcan2, tx_msg, CHASSIS_TX_ID, 8);

        memset(tx_msg, 0, 8);
        GetJudge2GimbalInfo();
        robot.comm.judgeEncode(&robot.comm, tx_msg);
        FDCAN_Send_Msg(&hfdcan2, tx_msg, JUDGE_TX_ID, 8);
    }
    if(robot.control_tick%10==0)
    {    memset(tx_msg, 0, 8);
    robot.sup_cap->set_sup_cap_on(robot.cmd.sup_cap_on);
    robot.sup_cap->set_power_buffer(robot.referee_ptr->POWER->getData().buffer_energy);
    robot.sup_cap->set_power_limit(robot.referee_ptr->PERFORMANCE->getData().chassis_power_limit);
    robot.sup_cap->Encode(tx_msg);
    FDCAN_Send_Msg(&hfdcan1, tx_msg, SUP_CAP_RX_ID, 8);}

    memset(tx_msg, 0, 8);
    tx_id = robot.chassis_motors[LFM]->tx_id();
    if (robot.chassis_motors[LFM]->encode(tx_msg, tx_id) == 0)
        encode_success_flag = 0;
    if (robot.chassis_motors[LBM]->encode(tx_msg, tx_id) == 0)
        encode_success_flag = 0;
    FDCAN_Send_Msg(&hfdcan1, tx_msg, tx_id, 8);

    memset(tx_msg, 0, 8);
    tx_id = robot.chassis_motors[LWM]->tx_id();
    robot.chassis_motors[LWM]->encode(tx_msg, tx_id);
    robot.chassis_motors[RWM]->encode(tx_msg, tx_id);
    FDCAN_Send_Msg(&hfdcan2, tx_msg, tx_id, 8);

    memset(tx_msg, 0, 8);
    tx_id = robot.chassis_motors[RFM]->tx_id();
    robot.chassis_motors[RFM]->encode(tx_msg, tx_id);
    robot.chassis_motors[RBM]->encode(tx_msg, tx_id);
    FDCAN_Send_Msg(&hfdcan1, tx_msg, tx_id, 8);

    // memset(tx_msg, 0, 8);
    // tx_id = robot.yaw_motor->tx_id();
    // robot.yaw_motor->encode(tx_msg, tx_id);
    // FDCAN_Send_Msg(&hfdcan2, tx_msg, tx_id, 8);
}

static void GetJudge2GimbalInfo(void)
{
    static uint32_t last_shooter_speed_update_tick, last_shooter_heat_update_tick;

    if (!robot.referee_ptr->PERFORMANCE->isHandled())
    {
        robot.comm.judge2gimbal.shooter_cooling_remain =MAX( robot.referee_ptr->PERFORMANCE->getData().shooter_barrel_heat_limit -
                                                         robot.referee_ptr->POWER->getData().shooter_17mm_1_barrel_heat,0);
        robot.comm.judge2gimbal.shooter_speed_update_seq++;
        robot.referee_ptr->PERFORMANCE->setHandled();
    }

    if (robot.referee_ptr->SHOOTER->isUpdated())
    {
        robot.comm.judge2gimbal.bullet_speed = robot.referee_ptr->SHOOTER->getData().bullet_speed;
        robot.referee_ptr->SHOOTER->setHandled();
    }
    else
    {
        robot.comm.judge2gimbal.bullet_speed = -1;
    }

    

    // if (robot.comm.judge2gimbal.bullet_speed == 0 )
    // {
    //     robot.comm.judge2gimbal.bullet_speed = 29.0f;
    // }
    // else if (last_shooter_speed_update_tick != js_datapack_update_ticks.shoot_data_update_tick)
    // {
    //     last_shooter_speed_update_tick = js_datapack_update_ticks.shoot_data_update_tick;
    //     robot.comm.judge2gimbal.bullet_speed = robot.referee_ptr->SHOOTER->getData().bullet_speed;
    //     robot.comm.judge2gimbal.shooter_speed_update_seq++;
    // }

    // if (last_shooter_heat_update_tick != js_datapack_update_ticks.power_heat_data_update_tick)
    // {
    //     last_shooter_heat_update_tick = js_datapack_update_ticks.power_heat_data_update_tick;
    //     robot.comm.judge2gimbal.shooter_cooling_remain =
    //         js_shooter_id1_17mm_cooling_limit - js_shooter_id1_17mm_cooling_heat;
    //     robot.comm.judge2gimbal.shooter_heat_update_seq++;
    // }

    robot.comm.judge2gimbal.shooter_speed_limit = SPEED_30_MPS;

    // robot.comm.judge2gimbal.gimbal_power_on = js_mains_power_gimbal_output;
    // robot.comm.judge2gimbal.shooter_power_on = js_mains_power_shooter_output;

    if (robot.referee_ptr->PERFORMANCE->getData().robot_id)
    {
        if (robot.referee_ptr->PERFORMANCE->getData().robot_id > kColorIDThres)
        {
            robot.comm.judge2gimbal.enemy_color = RED;
        }
        else
        {
            robot.comm.judge2gimbal.enemy_color = BLUE;
        }
    }
    else
    {
        robot.comm.judge2gimbal.enemy_color = UNKNOWN;
    }

    
    

}


static void GetChassis2GimbalCmd(void)
{
    robot.comm.chassis2gimbal.rc_x = robot.cmd.gimbal_x;
    robot.comm.chassis2gimbal.rc_y = robot.cmd.gimbal_y;
    float rc_x_lim = -fabsf(robot.chassis_states.dpos) / kNoGyroMinSpeed + 1;
    LIMIT_MAX(rc_x_lim, 0, 1);
    LIMIT_MAX(robot.comm.chassis2gimbal.rc_x, rc_x_lim, -rc_x_lim);
    if(!robot.Init_finish_flag){
        robot.cmd.gimbal_enable = false;
        robot.cmd.gimbal_ctrl_mode= GIMBAL_MANUAL;
    }


    robot.comm.chassis2gimbal.shoot = robot.cmd.shoot;
    robot.comm.chassis2gimbal.cmd_src = robot.cmd_src;
    robot.comm.chassis2gimbal.gimbal_ctrl_mode = robot.cmd.gimbal_ctrl_mode;
    robot.comm.chassis2gimbal.auto_aim_mode = robot.cmd.auto_aim_mode;
    robot.comm.chassis2gimbal.gimbal_enable = robot.cmd.gimbal_enable;
    robot.comm.chassis2gimbal.ignore_overheated = robot.cmd.ignore_overheated;
    robot.comm.chassis2gimbal.shooter_enable = robot.cmd.shooter_enable;
    
    robot.comm.chassis2gimbal.reverse_seq = robot.cmd.gimbal_reverse_seq;
    robot.comm.chassis2gimbal.auto_shoot = robot.cmd.auto_shoot;

    // if (robot.chassis_mode.curr == CHASSIS_DEAD)
    // {
    //     robot.comm.chassis2gimbal.reverse_seq = robot.comm.gimbal2chassis.reverse_finish_seq;
    // }
}
// static void UiTask(void)
// {
//   UiComponents ui_components = {
//       .AutoShootEdge = {0, 0, 700, 700},
//   };

//   // static size_t tx_len = 0;
//   // if (drawUi(ui_components, tx_data, &tx_len)) {
//   //   HAL_UART_Transmit(&huart6, tx_data, tx_len, 0xF);
//   // }
// }
