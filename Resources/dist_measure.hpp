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
#ifndef __dist_measure_HPP_
#define __dist_measure_HPP_
/* Includes ------------------------------------------------------------------*/
#include "stm32h723xx.h"
#include "main.h"
#include "stdlib.h"
#include "string.h"
#include "system.h"
#include "tick.hpp"
/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

namespace DistMeasure
{

#define USART_REC_LEN 200 // 定义最大接收字节数 200
#define EN_USART1_RX 1	  // 使能（1）/禁止（0）串口1接收

	extern char USART_RX_BUF[USART_REC_LEN]; // 接收缓冲,最大USART_REC_LEN个字节.末字节为换行符
	extern uint16_t USART_RX_STA;			 // 接收状态标记

#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t

#define ANGLE_PER_FRAME 12
#define HEADER 0x54
#define POINT_PER_PACK 12
#define VERLEN 0x2C // 低五位是一帧数据接收到的点数，目前固定是12，高三位固定为1
	typedef struct Point_Data
	{
		u16 distance; // 距离
		u8 intensity; // 置信度
	} LidarPointStructDef;

	typedef struct Pack_Data
	{
		uint8_t header;
		uint8_t ver_len;
		uint16_t temperature;
		uint16_t start_angle;
		LidarPointStructDef point[POINT_PER_PACK];
		uint16_t end_angle;
		uint16_t timestamp;
		uint8_t crc8;
	} LiDARFrameTypeDef;

	class dist_measure
	{
	private:
		float32_t distance; // 测量距离
		LiDARFrameTypeDef Pack_Data;
		uint8_t state = 0; // 状态位
		uint8_t crc = 0;   // 校验和
		uint8_t cnt = 0;   // 用于一帧12个点的计数
		int receive_cnt = 0;
		UART_HandleTypeDef *huart_dist;
		bool _is_updated; // 更新为true
		bool _is_handled; // 已被处理为true
		u8 dist_cnt = 0;
		u16 dist_count = 0;
		u32 dist_sum = 0;
		uint32_t last_update_tick = 0;
		void distance_calculate();

	public:
		dist_measure(UART_HandleTypeDef *huart)
		{
			distance = 0;
			state = 0;
			crc = 0;
			cnt = 0;
			receive_cnt = 0;
			this->huart_dist = huart;
			_is_updated = false;
			_is_handled = false;
			dist_cnt = 0;
			dist_count = 0;
			dist_sum = 0;
			last_update_tick = 0;
		}
		~dist_measure();
		void measure_reset()
		{
			distance = 0;
			state = 0;
			crc = 0;
			cnt = 0;
			receive_cnt = 0;
			_is_updated = false;
			_is_handled = false;
			dist_cnt = 0;
			dist_count = 0;
			dist_sum = 0;
			last_update_tick = 0;
		}
		// 若使用数据，则为true
		float32_t get_distance(bool is_handle = false)
		{
			if (distance < 50||hello_world::tick::GetTickMs() - last_update_tick > 1000)
			{
				distance = 0;
			}
			return distance/1000.0f;
		}

		inline bool is_updated()
		{
			return _is_updated;
		}

		inline bool is_handled()
		{
			return _is_handled;
		}

		inline void set_handle()
		{
			_is_handled = true;
		}
		void decode(uint8_t *data, int len, UART_HandleTypeDef *huart)
		{
			if(huart!=this->huart_dist){
				return ;
			}
			for (int i = 0; i < len; i++)
			{
				data_process(huart, data[i]);
			}
			memset(data, 0, len);
		}
		bool data_process(UART_HandleTypeDef *huart, uint8_t temp_data);
	};

} // namespace name

#endif /* __FILE_H_ */
