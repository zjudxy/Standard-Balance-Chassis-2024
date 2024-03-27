/**
 *******************************************************************************
 * @file      : chassis_task.cpp
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
#include "HW_fdcan.hpp"
#include "chassis_task.hpp"
#include "chassis.hpp"
#include "chassis_sense_task.hpp"
#include "chassis_cmd_task.hpp"
#include "chassis_ctrl_task.hpp"
#include "chassis_switch_task.hpp"
#include "chassis_comm_task.hpp"
#include "chassis_detect_task.hpp"
#include "chassis_ui_task.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/** 
 * @brief      
 *   @arg       None
 * @retval      None
 * @note        None
 */
static void chassis_Task();

/**
 * @brief      底盘程序初始化总入口
 * @retval      None
 * @note        None
 */
void ChassisInit()
{
    RobotInit();

    CmdInit();

    SenseInit();

    UIInit();

    DetectInit();

    CtrlInit();

    CommInit();

    CanInit();

    HAL_TIM_Base_Start_IT(&htim6);

    
}

/**
 * @brief      底盘程序总入口
 * @retval      None
 * @note        None
 */
static void chassis_Task()
{
    robot.control_tick++;
    
    CmdTask();

    SenseTask();

    UITask();

    DetectTask();

    SwitchTask();

    CtrlTask();

    CommTask();



    
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim6)
    {   
        chassis_Task();
    }
}