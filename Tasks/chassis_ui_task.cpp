/**
 *******************************************************************************
 * @file      : chassis_ui_task.cpp
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
#include "chassis_ui_task.hpp"
#include "judge.hpp"
#include "chassis_ui_task.hpp"
#include "chassis.hpp"
#include "referee.hpp"
#include "usart.h"
#include "chassis_comm_task.hpp"
#include "referee_data.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/


namespace rfr = hello_world::referee;
rfr::RfrEncoder* rfr_encoder = nullptr;
rfr::InterGraphic7Package* inter_graphic7_pkg;
rfr::RfrId sender_id;
int last_hero_hp,curr_hero_hp,hero_attacked_tick;
int last_base_hp,curr_base_hp,base_attacked_tick;
int last_outpost_hp,curr_outpost_hp,outpost_attacked_tick;


void AttackEventUpdate(UiComponents &ui_components);


/**
 * @brief      UI初始化总入口
 * @retval      None
 * @note        None
 */
void UIInit(void)
{
  last_hero_hp = curr_hero_hp =hero_attacked_tick = 0;
  last_base_hp = curr_base_hp = base_attacked_tick = 0;
  last_outpost_hp = curr_outpost_hp = outpost_attacked_tick = 0;
}
uint8_t UI_tx_data[referee::kRefereeMaxFrameLength];

/**
 * @brief      UI绘制总入口
 * @retval      None
 * @note        None
 */
void UITask()
{
    static size_t tx_len=0;
    static int ui_send_time =0;

    if(robot.control_tick<500){

      return ;
    }

    bool is_shooted_armor_updated=robot.referee_ptr->HURT->isUpdated();
    if(is_shooted_armor_updated){
        robot.referee_ptr->HURT->setHandled();
    }
    UiComponents ui_components={
        .robot_id=robot.referee_ptr->PERFORMANCE->getData().robot_id,
        .angle=robot.yaw_motor->angle(),
        .super_cap_percentage=(uint16_t)(robot.sup_cap->get_remain_present()*100.0f),
        .auto_shoot_target_id=robot.comm.gimbal2chassis.aimed_enemy_id,
        .auto_shoot_target_list=robot.comm.gimbal2chassis.target_list,
        .shooted_armor=robot.referee_ptr->HURT->getData().module_id,
        .is_shooted_armor_updated=is_shooted_armor_updated,
        .leg_height = {(robot.leg_states[LEFT].curr.state.height-kHeightMin)/(kHeightMax-kAirHeightMin),
        (robot.leg_states[RIGHT].curr.state.height-kHeightMin)/(kHeightMax-kAirHeightMin)},
        .chassis_mode = (uint8_t)robot.chassis_mode.curr,
        .steer_mode = (uint8_t)robot.steer_mode.curr
    };

    AttackEventUpdate(ui_components);

    if(robot.control_tick<1000){
      if(robot.ui_drawer->drawUi(ui_components,UI_tx_data,&tx_len,false)){
        ui_send_time++;
       HAL_UART_Transmit_DMA(&huart1, UI_tx_data, tx_len);
    }
      return ;
    }

    bool ui_draw = true;
    if(robot.cmd.ui_redraw){
        robot.cmd.ui_redraw = false;
        ui_draw = false;
    }

    if(robot.ui_drawer->drawUi(ui_components,UI_tx_data,&tx_len,ui_draw)){
        ui_send_time++;
       HAL_UART_Transmit_DMA(&huart1, UI_tx_data, tx_len);
    }
}


//我方受击打事件更新
void AttackEventUpdate(UiComponents &ui_components){
      if(robot.comm.judge2gimbal.enemy_color==RED){
      curr_hero_hp = robot.referee_ptr->COMP_ROBOTS_HP->getData().blue_1_robot_HP;
      curr_base_hp = robot.referee_ptr->COMP_ROBOTS_HP->getData().blue_base_HP;
      curr_outpost_hp = robot.referee_ptr->COMP_ROBOTS_HP->getData().blue_outpost_HP;
    }
    else{
      curr_hero_hp = robot.referee_ptr->COMP_ROBOTS_HP->getData().red_1_robot_HP;
      curr_base_hp = robot.referee_ptr->COMP_ROBOTS_HP->getData().red_base_HP;
      curr_outpost_hp = robot.referee_ptr->COMP_ROBOTS_HP->getData().red_outpost_HP;
    }
      
      if(last_hero_hp - curr_hero_hp > 4){
        hero_attacked_tick = kAttackRemainTick;
      }
      last_hero_hp = curr_hero_hp;

      if(last_base_hp - curr_base_hp > 4){
        base_attacked_tick = kAttackRemainTick;
      }
      last_base_hp = curr_base_hp;

      if(last_outpost_hp - curr_outpost_hp > 4){
        outpost_attacked_tick = kAttackRemainTick;
      }
      last_outpost_hp = curr_outpost_hp;

      if(hero_attacked_tick>0){
        hero_attacked_tick--;
      }
      if(base_attacked_tick>0){
        base_attacked_tick--;
      }
      if(outpost_attacked_tick>0){
        outpost_attacked_tick--;
      }
      ui_components.is_base_attacked = base_attacked_tick;
      ui_components.is_hero_attacked = hero_attacked_tick;
      ui_components.is_outpost_attacked = outpost_attacked_tick;

}