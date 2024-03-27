

/**
 * @file      tick.cpp
 * @brief
 * @author    ZhouShichan (zsc19823382069@163.com)
 * @version   1.0.0
 * @date      2023-10-31
 *
 * @copyright Copyright (c) 2023 Hello World Team, Zhejiang University. All Rights Reserved.
 *
 * @attention
 *
 * @par history
 * | Version | Date | Author | Description |
 * | :---: | :---: | :---: | :---: |
 * | 1.0.0 | {now_year} | ZhouShichan | description |
 * @par last editor  ZhouShichan (zsc19823382069@163.com)
 * @par last edit time  2023-10-31
 */
#include "tick.hpp"

#ifdef USE_UNIX_TICK
/* Includes ------------------------------------------------------------------*/
#include <chrono>
#include <ctime>
namespace hello_world
{
namespace tick
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
std::chrono::microseconds start_tick_us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch());
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions -----------------------------------------------*/
uint32_t                  GetTickUs()
{
  std::chrono::system_clock::time_point now  = std::chrono::system_clock::now();
  std::chrono::microseconds             tick = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()) - start_tick_us;
  return (uint32_t)(tick.count());
};

uint32_t GetTickMs() { return GetTickUs() / 1000; };

uint32_t GetTickS() { return GetTickMs() / 1000; };
/* Private function definitions -----------------------------------------------*/
}  // namespace tick

}  // namespace hello_world
#endif

#ifdef USE_HAL_TICK
#include "stm32h7xx_hal.h"
namespace hello_world
{
namespace tick
{
uint32_t start_tick_us = GetTickUs();

uint32_t GetTickUs() { return (uint32_t)((HAL_GetTick() * HAL_GetTickFreq()) * 1000 + SysTick->VAL * 1000 / SysTick->LOAD) - start_tick_us; };

uint32_t GetTickMs() { return GetTickUs() / 1000; };

uint32_t GetTickS() { return GetTickMs() / 1000; };
}  // namespace tick

}  // namespace hello_world
#endif