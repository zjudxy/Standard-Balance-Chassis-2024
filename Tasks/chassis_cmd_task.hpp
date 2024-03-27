/** 
 *******************************************************************************
 * @file      : chassis_cmd_task.hpp
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
#ifndef __chassis_cmd_task_H_
#define __chassis_cmd_task_H_
/* Includes ------------------------------------------------------------------*/
#include "DT7.hpp"
#include "system.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/

/* Exported function prototypes ----------------------------------------------*/

extern hello_world::remote_control::DT7 *rc_ptr, *last_rc_ptr, *curr_rc_ptr;


void CmdInit(void);
void CmdTask(void);


#endif /* __FILE_H_ */
