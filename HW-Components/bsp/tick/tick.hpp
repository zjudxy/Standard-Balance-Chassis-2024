

/**
 * @file      tick.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_CPP_SYSTEM_TICK_H_
#define HW_COMPONENTS_CPP_SYSTEM_TICK_H_

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>

namespace hello_world
{
namespace tick
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
uint32_t GetTickUs();
uint32_t GetTickMs();
uint32_t GetTickS();

}  // namespace tick

}  // namespace hello_world

#endif /* HW_COMPONENTS_CPP_SYSTEM_TICK_H_ */
