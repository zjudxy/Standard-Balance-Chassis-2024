/**
 *******************************************************************************
 * @file      : chassis_commute_task.h
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CHASSIS_COMM_TASK_H_
#define __CHASSIS_COMM_TASK_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "stdbool.h"
#include "stdint.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
extern uint8_t referee_rx_data[255];
extern uint8_t referee_tx_data[255];
extern uint8_t dist_measure_rx_data_Front[1024];
extern uint8_t dist_measure_rx_data_Back[1024];

void CommInit(void);

/**
 * @brief       get msg based on command to gimbal
 * @retval      None
 * @note        None
 */
void CommTask(void);

#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_COMM_TASK_H_ */
