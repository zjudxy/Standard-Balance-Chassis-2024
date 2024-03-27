/** 
 *******************************************************************************
 * @file      : chassis_detect_task.hpp
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
#ifndef __JUDGE_HPP__
#define __JUDGE_HPP__
/* Includes ------------------------------------------------------------------*/
#include "referee.hpp"
#include "rfr_official_pkgs.hpp"
#include "rfr_pkg_core.hpp"
/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

// struct UiComponents {
//   uint16_t AutoShootEdge[4];  // x1,y1,x2,y2
// };


namespace referee = hello_world::referee;
class RobotReferee
{
private:
public:
    referee::RfrDecoder *rfr_decoder;
    referee::RobotShooterPackage *SHOOTER;//发射机构
    referee::RobotHurtPackage *HURT;//受击打数据
    referee::RobotBuffPackage *BUFF;//增益数据
    referee::RobotPowerHeatPackage *POWER;//实时底盘功率和枪口热量数
    referee::RobotPerformancePackage *PERFORMANCE;//机器人性能体系数据包
    referee::CompRobotsHpPackage *COMP_ROBOTS_HP;
    int RX_DMA_LEN ;
    int TX_DMA_LEN;
    // referee::refereeEncoder *referee_encoder;



    RobotReferee(){
    rfr_decoder = new referee::RfrDecoder();
    SHOOTER = new referee::RobotShooterPackage();
    HURT = new referee::RobotHurtPackage();
    BUFF = new referee::RobotBuffPackage();
    POWER = new referee::RobotPowerHeatPackage();
    PERFORMANCE = new referee::RobotPerformancePackage();
    COMP_ROBOTS_HP = new referee::CompRobotsHpPackage();

    // referee_encoder = new referee::refereeEncoder();
    rfr_decoder->appendRxPackage(HURT);
    rfr_decoder->appendRxPackage(BUFF);
    rfr_decoder->appendRxPackage(POWER);
    rfr_decoder->appendRxPackage(PERFORMANCE);
    rfr_decoder->appendRxPackage(SHOOTER);
    rfr_decoder->appendRxPackage(COMP_ROBOTS_HP);
    RX_DMA_LEN = 255;
    TX_DMA_LEN = 255;
    }
    ~RobotReferee(){
        delete SHOOTER;
        delete HURT;
        delete BUFF;
        delete POWER;
        delete PERFORMANCE;
        delete COMP_ROBOTS_HP;
        
        delete rfr_decoder;

    }
    bool decode(uint8_t *data, int len)
    {
        for(int i = 0;i<len;i++){
            rfr_decoder->processByte(data[i]);
            // data[i] = 0;
        }
        memset(data,0,len);
        return true;
        // rfr_decoder->decodeFrame(data,len);
    }
    int get_rx_dma_len()
    {
        return RX_DMA_LEN;
    }
    int get_tx_dma_len()
    {
        return TX_DMA_LEN;
    }
    
     
    // bool drawUi(UiComponents ui_components, uint8_t* tx_data, size_t* tx_len);


};


#endif /* __JUDGE_HPP__ */
