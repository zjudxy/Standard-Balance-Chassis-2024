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
extern float StateSpaceModelA_L[2][6][6];
extern float StateSpaceModelB_L[2][6][2];
extern float StateSpaceModelU[2][2];

const int NL = 3;
const float32_t b_ls_20[3] = {
   0.003621681593, 0.007243363187, 0.003621681593
};
const int DL = 3;
const float32_t a_ls_20[3] = {
                1,   -1.822694898,   0.8371816278
};

const float32_t b_ls_30[3] = {
   0.007820207626,  0.01564041525, 0.007820207626
};
const float32_t a_ls_30[3] = {
                1,   -1.734725714,   0.7660065889
};



void SenseInit(void);

void SenseTask(void);

void StateEstimate(void);

#endif /* __FILE_H_ */
