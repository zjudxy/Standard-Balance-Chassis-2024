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
#ifndef __SUP_CAPACITY__
#define __SUP_CAPACITY__

/* Includes ------------------------------------------------------------------*/
#include "tools.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

typedef enum _volt_src_e
{
    VOLT_CAP = 0,   // 超电
    VOLT_JUDGE = 1, // 裁判系统
} VoltSrc_e;
class super_capacity
{
private:
    float MaxVolt; // 电容最高电压
    float MinVolt; // 电容最低电压

    bool sup_cap_on;  // 开启超电
    int power_buffer; // 缓冲能量大小
    int power_limit;  // 功率限制大小

    VoltSrc_e volt_src;   // 电源来向
    float volt;           // 电容电压
    float remain_present; // 电容剩余百分比，0-1
public:
    super_capacity(float MaxVolt, float MinVolt)
    {
        this->MaxVolt = MaxVolt;
        this->MinVolt = MinVolt;
        this->sup_cap_on = false;
        this->power_buffer = 0;
        this->power_limit = 0;
        this->volt_src = VOLT_JUDGE;
        this->volt = 0;
        this->remain_present = 0;
    }
    ~super_capacity()
    {
    }
    void Encode(uint8_t tx_data[8])
    {
        tx_data[0] = (uint8_t)this->power_buffer;
        tx_data[1] = (uint8_t)this->sup_cap_on;
        tx_data[2] = (uint8_t)this->power_limit;
    }
    void Decode(uint8_t rx_data[8])
    {
        this->volt = (int16_t)(rx_data[0] << 8 | rx_data[1]) / 1000.0f;

        /* 电容剩余电量计算 */
        float remain_present =
            (this->volt * this->volt - this->MinVolt * this->MinVolt) /
            (this->MaxVolt * this->MaxVolt - this->MinVolt * this->MinVolt);
        LIMIT_MAX(remain_present, 1, 0); // 电量限幅
        this->remain_present = remain_present;

        this->volt_src = (VoltSrc_e)rx_data[3];
    }
    void set_sup_cap_on(bool on)
    {
        this->sup_cap_on = on;
    }
    void set_power_buffer(int power_buffer)
    {
        this->power_buffer = power_buffer;
    }
    void set_power_limit(int power_limit)
    {
        this->power_limit = power_limit;
    }
    float get_remain_present()
    {
        return this->remain_present;
    }
};

#endif /* __SUP_CAPACITY__ */
