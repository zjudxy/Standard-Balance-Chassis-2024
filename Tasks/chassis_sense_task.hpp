/** 
 *******************************************************************************
 * @file      : chassis_sense_task.hpp
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
#ifndef __chassis_sense_task_H_
#define __chassis_sense_task_H_

/* Includes ------------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

#define IMU_TEMP_HTIM htim10
#define IMU_TEMP_CH TIM_CHANNEL_1


extern float32_t angle_offset;
extern float leg_end_ddpos[2];
void SenseInit(void);

void SenseTask(void);


#endif /* __FILE_H_ */
