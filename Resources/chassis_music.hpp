/**
 *******************************************************************************
 * @file      : chassis_motor.hpp
 * @brief     : 底盘电机参数
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
#ifndef __chassis_music_HPP_
#define __chassis_music_HPP_
/* Includes ------------------------------------------------------------------*/
#include "buzzer.hpp"
/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

buzzer::TuneListInfo tune_list_info = {
    // 配置乐曲信息
    .intensity_scale = 1.0f,
    .tune_duration = 100,
    // .list = {
    //     buzzer::kTuneC4,
    //     buzzer::kTuneD4,
    //     buzzer::kTuneE4,
    //     buzzer::kTuneF4,
    //     buzzer::kTuneG4,
    //     buzzer::kTuneA5,
    //     buzzer::kTuneB5,
    //     buzzer::kTuneC5,
    //     buzzer::kTuneEnd,
    // },
    .list = {
        buzzer::kTuneC4,
        buzzer::kTuneD4,
        buzzer::kTuneE4,
        buzzer::kTuneF4,
        buzzer::kTuneG4,
        buzzer::kTuneA5,
        buzzer::kTuneB5,
        buzzer::kTuneC5,
        buzzer::kTuneEnd,
    },
};

#endif /* __FILE_H_ */
