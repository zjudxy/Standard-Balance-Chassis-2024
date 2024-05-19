/**
 *******************************************************************************
 * @file      : HW_can.c
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2023 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "HW_fdcan.hpp"
#include "fdcan.h"
#include "chassis.hpp"
// #include "chassis_comm_task.hpp"
#include "chassis_gimbal_comm.h"
#include "system.h"
#include "tools.h"
#include "motor_params.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static FDCAN_RxHeaderTypeDef rx_header1, rx_header2;
static uint8_t rx_data1[8], rx_data2[8];

uint32_t pTxMailbox;
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

// can init
void CanInit(void)
{
  FDCAN_FilterTypeDef can_filter1 = {
      .IdType = FDCAN_STANDARD_ID,
      .FilterIndex = 0,
      .FilterType = FDCAN_FILTER_MASK,
      .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
      .FilterID1 = 0x00000000,
      .FilterID2 = 0x00000000,
      // .RxBufferIndex = 0,
      // .IsCalibrationMsg = 0,
  };

  HAL_FDCAN_ConfigFilter(&hfdcan1, &can_filter1);
  HAL_FDCAN_Start(&hfdcan1);
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, DISABLE, ENABLE);
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  FDCAN_FilterTypeDef can_filter2 = {
      .IdType = FDCAN_STANDARD_ID,
      .FilterIndex = 0,
      .FilterType = FDCAN_FILTER_MASK,
      .FilterConfig = FDCAN_FILTER_TO_RXFIFO1,
      .FilterID1 = 0x00000000,
      .FilterID2 = 0x00000000,
      // .RxBufferIndex = 0,
      // .IsCalibrationMsg = 0,
  };

  HAL_FDCAN_ConfigFilter(&hfdcan2, &can_filter2);
  HAL_FDCAN_Start(&hfdcan2);
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, DISABLE, ENABLE);
  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
}
void HAL_FDCAN_RxCpltCallback(FDCAN_HandleTypeDef *hfdcan)
{
}
void FDCAN_Send_Msg(FDCAN_HandleTypeDef *hfdcan, uint8_t *msg, uint32_t id, uint8_t len)
{
  FDCAN_TxHeaderTypeDef tx_header = {
      .Identifier = id,
      .IdType = FDCAN_STANDARD_ID,
      .TxFrameType = FDCAN_DATA_FRAME,
      .DataLength = FDCAN_DLC_BYTES_8,
      .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
      .BitRateSwitch = FDCAN_BRS_OFF,
      .FDFormat = FDCAN_CLASSIC_CAN,
      .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
      .MessageMarker = 0,
  };
  HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, msg);
}
int sup_cap_rx_time = 0;
uint32_t level = 0;
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
  {
    FDCAN_RxHeaderTypeDef rx_header1;
    if (hfdcan == &hfdcan1)
    {
      while(level = HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0)) {
        HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx_header1, &rx_data1[0]);


        yaw_motor_ptr->decode(rx_data1, rx_header1.Identifier);
        for (int i = 0; i < 6; i++)
        {
          robot.chassis_motors[i]->decode(rx_data1, rx_header1.Identifier);
        }
        if(rx_header1.Identifier == SUP_CAP_TX_ID)
        {

          robot.sup_cap->Decode(rx_data1);
          sup_cap_rx_time ++ ;
        }
      }
        
    }
    

    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  }
}

uint8_t LWM_data[8];
uint8_t RWM_data[8];
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
  if (RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE)
  {
    FDCAN_RxHeaderTypeDef rx_header2;
    if (hfdcan == &hfdcan2)
    {
      HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO1, &rx_header2, &rx_data2[0]);
            if (rx_header2.Identifier == GIMBAL_TX_ID)
      {
        robot.comm.chassisDecode(&robot.comm, rx_data2);
        robot.gimbal_comm_tick = robot.control_tick;
      }
      else
      {
        if(rx_header2.Identifier==robot.yaw_motor->rx_id()){
        robot.yaw_motor->decode(rx_data2, rx_header2.Identifier);
        }
      for (int i = 0; i < 6; i++)
      {
        robot.chassis_motors[i]->decode(rx_data2, rx_header2.Identifier);
      }
      }
      if(rx_header2.Identifier==robot.chassis_motors[LWM]->rx_id())
      {
        memcpy(LWM_data,rx_data2,8);
      }
      if(rx_header2.Identifier==robot.chassis_motors[RWM]->rx_id())
      {
        memcpy(RWM_data,rx_data2,8);
      }
      // if(rx_header2.Identifier == SUP_CAP_TX_ID)
      //   {
      //     robot.sup_cap->Decode(rx_data2);
      //   }
    }

    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
  }
}
