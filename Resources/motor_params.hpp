/**
 *******************************************************************************
 * @file      : dev_motor_param.h
 * @brief     : define the param of motor that contorl through CAN
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2023-01-16      ckz             1. finish first demo
 *  V1.0.0      2023-02-06      ckz             1. complete code test
 *  V1.0.2      2023-03-02      ckz             1. modify naming
 *  V1.1.0      2023-04-14      ckz             1. solve the problem of lap counting lag
 *                                              2. corrected A1 motor's reduction ratio
 *  V1.1.2      2023-05-14      ckz             1. add user settings for motor input restrictions
 *                                              2. modify motor parameter description
 *  V1.1.3      2023-07-01      ckz             1. add MF9025v2-16T motor
 *                                              2. add motor power reading
 *******************************************************************************
 * @attention : Some param of motor remain to be determined.
 *******************************************************************************
 *  Copyright (c) 2023 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTOR_PARAMS_H_
#define __MOTOR_PARAMS_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "system.h"

/* Exported macro ------------------------------------------------------------*/
#ifndef PI
#define PI 3.14159265358979f
#endif

/* identifier of speed controller*/
#define GM6020_RX_1_4 0x1FF
#define GM6020_RX_5_7 0x2FF
#define GM6020_TX 0x204
#define M3508_RX_1_4 0x200
#define M3508_RX_5_8 0x1FF
#define M3508_TX 0x200
#define M2006_RX_1_4 0x200
#define M2006_RX_5_8 0x1FF
#define M2006_TX 0x200
#define M8910_RX_1_4 0xFF
#define M8910_RX_5_8 0x100
#define M8910_TX 0x100
#define A1_RX_1_3 0x400
#define A1_RX_4_6 0x3FF
#define A1_TX 0x400
#define DM_J4310_RX 0x00
#define DM_J4310_TX 0x10
#define MF9025V2_RX 0x280
#define MF9025V2_TX 0x140

#define INVALID_VALUE 0
#define UNLIMITED_VALUE 1e10
#define DEFAULT_VALUE 0

#define DM_J4310_ANG_MAX PI
#define DM_J4310_ANG_BITS 16
#define DM_J4310_OMG_MAX 30.0f
#define DM_J4310_OMG_BITS 12
#define DM_J4310_TORGUE_MAX 7.0f
#define DM_J4310_TORGUE_BITS 12

/* Exported constants --------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/


/* Exported variables --------------------------------------------------------*/

/* Exported function prototypes ----------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __DEV_MOTOR_PARAM_H_ */
