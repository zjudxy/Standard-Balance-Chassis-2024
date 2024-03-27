/**
 *******************************************************************************
 * @file      : chassis_gimbal_comm.c
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2023 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "chassis_gimbal_comm.h"

#include "string.h"
#include "tools.h"
/* Private macro -------------------------------------------------------------*/
#ifndef __PACKED_STRUCT
#define __PACKED_STRUCT struct __attribute__((packed, aligned(1)))
#endif

/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/

typedef union _judge2gimbal_u {
    uint8_t msg[8];
    __PACKED_STRUCT
    {
        uint16_t bullet_speed_x1000;

        uint8_t shooter_cooling_remain;

        uint8_t shooter_speed_limit : 2;
        uint8_t enemy_color : 2;
        uint8_t infantry_type_3 : 1;
        uint8_t infantry_type_4 : 1;
        uint8_t infantry_type_5 : 1;
        uint8_t gimbal_power_on : 1;

        uint8_t shooter_speed_updata_seq;

        uint8_t shooter_heat_update_seq;

        uint8_t shooter_power_on : 1;
    }
    bits;
} Judge2Gimbal_u;

typedef union _chassis2gimbal_u {
    uint8_t msg[8];
    __PACKED_STRUCT
    {
        int16_t rc_x_int16;

        int16_t rc_y_int16;

        uint8_t cmd_src : 1;
        uint8_t gimbal_ctrl_mode : 1;
        uint8_t shoot : 1;
        uint8_t shooter_enable : 1;
        uint8_t ignore_overheated : 1;
        uint8_t auto_aim_mode : 3;

        uint8_t gimbal_enable : 1;
        uint8_t auto_shoot : 1;

        uint8_t reverse_seq;
    }
    bits;
} Chassis2Gimbal_u;

typedef union _gimbal2chassis_u {
    uint8_t msg[8];
    __PACKED_STRUCT
    {
        uint8_t target_list;

        uint8_t aimed_enemy_id;

        uint8_t magazine_close : 1;
        uint8_t gimbal_imu_cal : 1;
        uint8_t gimbal_reset : 1;
        uint8_t minipc_ok : 1;

        uint8_t reverse_finish_seq;
    }
    bits;
} Gimbal2Chassis_u;

/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

static inline int16_t FloatToInt16(float f);

static inline float Int16ToFloat(int16_t i);

static void JudgeEncode(const ChassisGimbalComm_t *comm, uint8_t tx_data[8]);

static void JudgeDecode(ChassisGimbalComm_t *comm, const uint8_t rx_data[8]);

static void ChassisEncode(const ChassisGimbalComm_t *comm, uint8_t tx_data[8]);

static void ChassisDecode(ChassisGimbalComm_t *comm, const uint8_t rx_data[8]);

static void GimbalEncode(const ChassisGimbalComm_t *comm, uint8_t tx_data[8]);

static void GimbalDecode(ChassisGimbalComm_t *comm, const uint8_t rx_data[8]);

/**
 * @brief       init communicator
 * @param       *comm: ptr to communicator
 * @retval      None
 * @note        None
 */
void ChassisGimbalCommInit(ChassisGimbalComm_t *comm)
{
    comm->judge2gimbal.bullet_speed = 0;
    comm->judge2gimbal.enemy_color = UNKNOWN;
    comm->judge2gimbal.gimbal_power_on = false;
    comm->judge2gimbal.infantry_types.type_3 = NORMAL_INFANTRY;
    comm->judge2gimbal.infantry_types.type_4 = NORMAL_INFANTRY;
    comm->judge2gimbal.infantry_types.type_5 = NORMAL_INFANTRY;
    comm->judge2gimbal.shooter_power_on = false;
    comm->judge2gimbal.shooter_speed_limit = SPEED_UNKNOWN;
    comm->judge2gimbal.shooter_cooling_remain = 0;
    comm->judge2gimbal.shooter_heat_update_seq = 0;
    comm->judge2gimbal.shooter_speed_update_seq = 0;

    comm->chassis2gimbal.rc_x = 0;
    comm->chassis2gimbal.rc_y = 0;
    comm->chassis2gimbal.cmd_src = REMOTE;
    comm->chassis2gimbal.gimbal_ctrl_mode = GIMBAL_MANUAL;
    comm->chassis2gimbal.shoot = false;
    comm->chassis2gimbal.shooter_enable = false;
    comm->chassis2gimbal.ignore_overheated = false;
    comm->chassis2gimbal.auto_aim_mode = AUTO_AIM_CLOSE;
    comm->chassis2gimbal.gimbal_enable = true;
    comm->chassis2gimbal.reverse_seq = 0;
    comm->chassis2gimbal.auto_shoot = false;

    comm->gimbal2chassis.magazine_close = true;
    comm->gimbal2chassis.gimbal_imu_cal = false;
    comm->gimbal2chassis.gimbal_reset = false;
    comm->gimbal2chassis.reverse_finish_seq = 0;

    comm->judgeEncode = JudgeEncode;
    comm->judgeDecode = JudgeDecode;
    comm->chassisEncode = ChassisEncode;
    comm->chassisDecode = ChassisDecode;
    comm->gimbalEncode = GimbalEncode;
    comm->gimbalDecode = GimbalDecode;
}

/**
 * @brief       get msg of the info from judge on chassis
 * @param       *comm: ptr to communicator
 * @param       tx_data: info of judge to be sent by chassis
 * @retval      None
 * @note        None
 */
static void JudgeEncode(const ChassisGimbalComm_t *comm, uint8_t tx_data[8])
{
    Judge2Gimbal_u j2g_msg;

    j2g_msg.bits.bullet_speed_x1000 = comm->judge2gimbal.bullet_speed * 1000;
    if (comm->judge2gimbal.shooter_cooling_remain > 0xFF) {
        j2g_msg.bits.shooter_cooling_remain = 0xFF;
    } else {
        j2g_msg.bits.shooter_cooling_remain = comm->judge2gimbal.shooter_cooling_remain;
    }
    j2g_msg.bits.shooter_speed_limit = comm->judge2gimbal.shooter_speed_limit;
    j2g_msg.bits.shooter_heat_update_seq = comm->judge2gimbal.shooter_heat_update_seq;
    j2g_msg.bits.shooter_speed_updata_seq = comm->judge2gimbal.shooter_speed_update_seq;
    j2g_msg.bits.gimbal_power_on = comm->judge2gimbal.gimbal_power_on;
    j2g_msg.bits.shooter_power_on = comm->judge2gimbal.shooter_power_on;
    j2g_msg.bits.infantry_type_3 = comm->judge2gimbal.infantry_types.type_3;
    j2g_msg.bits.infantry_type_4 = comm->judge2gimbal.infantry_types.type_4;
    j2g_msg.bits.infantry_type_5 = comm->judge2gimbal.infantry_types.type_5;
    j2g_msg.bits.enemy_color = comm->judge2gimbal.enemy_color;

    memcpy(tx_data + JUDGE_TX_INDEX, j2g_msg.msg, 8 - JUDGE_TX_INDEX);
}

/**
 * @brief       get msg of the info from judge on chassis
 * @param       *comm: ptr to communicator
 * @param       rx_data: info of judge to be received by gimbal
 * @retval      None
 * @note        None
 */
static void JudgeDecode(ChassisGimbalComm_t *comm, const uint8_t rx_data[8])
{
    Judge2Gimbal_u j2g_msg;

    memcpy(j2g_msg.msg, rx_data + JUDGE_TX_INDEX, 8 - JUDGE_TX_INDEX);
    comm->judge2gimbal.bullet_speed = j2g_msg.bits.bullet_speed_x1000 / 1000.0f;
    comm->judge2gimbal.shooter_cooling_remain = j2g_msg.bits.shooter_cooling_remain;
    comm->judge2gimbal.shooter_heat_update_seq = j2g_msg.bits.shooter_heat_update_seq;
    comm->judge2gimbal.shooter_speed_update_seq = j2g_msg.bits.shooter_speed_updata_seq;
    comm->judge2gimbal.shooter_speed_limit = (ShooterSpeedLimit_e)j2g_msg.bits.shooter_speed_limit;
    comm->judge2gimbal.gimbal_power_on = (bool)j2g_msg.bits.gimbal_power_on;
    comm->judge2gimbal.shooter_power_on = (bool)j2g_msg.bits.shooter_power_on;
    comm->judge2gimbal.infantry_types.type_3 = (InfantryType_e)j2g_msg.bits.infantry_type_3;
    comm->judge2gimbal.infantry_types.type_4 = (InfantryType_e)j2g_msg.bits.infantry_type_4;
    comm->judge2gimbal.infantry_types.type_5 = (InfantryType_e)j2g_msg.bits.infantry_type_5;
    comm->judge2gimbal.enemy_color = (EnemyColot_e)j2g_msg.bits.enemy_color;
}

/**
 * @brief       get msg of the cmd that chassis send to gimbal
 * @param       *comm: ptr to communicator
 * @param       tx_data: data to be sent by chassis
 * @retval      None
 * @note        None
 */
static void ChassisEncode(const ChassisGimbalComm_t *comm, uint8_t tx_data[8])
{
    Chassis2Gimbal_u c2g_msg;

    c2g_msg.bits.rc_x_int16 = FloatToInt16(comm->chassis2gimbal.rc_x);
    c2g_msg.bits.rc_y_int16 = FloatToInt16(comm->chassis2gimbal.rc_y);
    c2g_msg.bits.cmd_src = comm->chassis2gimbal.cmd_src;
    c2g_msg.bits.gimbal_ctrl_mode = comm->chassis2gimbal.gimbal_ctrl_mode;
    c2g_msg.bits.shoot = comm->chassis2gimbal.shoot;
    c2g_msg.bits.shooter_enable = comm->chassis2gimbal.shooter_enable;
    c2g_msg.bits.ignore_overheated = comm->chassis2gimbal.ignore_overheated;
    c2g_msg.bits.auto_aim_mode = comm->chassis2gimbal.auto_aim_mode;
    c2g_msg.bits.gimbal_enable = comm->chassis2gimbal.gimbal_enable;
    c2g_msg.bits.reverse_seq = comm->chassis2gimbal.reverse_seq;
    c2g_msg.bits.auto_shoot = comm->chassis2gimbal.auto_shoot;

    memcpy(tx_data + CHASSIS_TX_INDEX, c2g_msg.msg, 8 - CHASSIS_TX_INDEX);
}

/**
 * @brief       get feedback msg from gimbal
 * @param       *comm: ptr to communicator
 * @param       rx_data: data received by chassis
 * @retval      None
 * @note        None
 */
static void ChassisDecode(ChassisGimbalComm_t *comm, const uint8_t rx_data[8])
{
    Gimbal2Chassis_u g2c_msg;

    memcpy(g2c_msg.msg, rx_data + GIMBAL_TX_INDEX, 8 - GIMBAL_TX_INDEX);
    comm->gimbal2chassis.target_list = (Target_e)g2c_msg.bits.target_list;
    comm->gimbal2chassis.aimed_enemy_id = g2c_msg.bits.aimed_enemy_id;
    comm->gimbal2chassis.magazine_close = (bool)g2c_msg.bits.magazine_close;
    comm->gimbal2chassis.gimbal_imu_cal = (bool)g2c_msg.bits.gimbal_imu_cal;
    comm->gimbal2chassis.gimbal_reset = (bool)g2c_msg.bits.gimbal_reset;
    comm->gimbal2chassis.reverse_finish_seq = g2c_msg.bits.reverse_finish_seq;
    comm->gimbal2chassis.minipc_ok = (bool)g2c_msg.bits.minipc_ok;
}

/**
 * @brief       get msg of the feedback that gimbal send to chassis
 * @param       *comm: ptr to communicator
 * @param       tx_data: data to be sent by gimbal
 * @retval      None
 * @note        None
 */
static void GimbalEncode(const ChassisGimbalComm_t *comm, uint8_t tx_data[8])
{
    Gimbal2Chassis_u g2c_msg;

    g2c_msg.bits.target_list = comm->gimbal2chassis.target_list;
    g2c_msg.bits.aimed_enemy_id = comm->gimbal2chassis.aimed_enemy_id;
    g2c_msg.bits.magazine_close = comm->gimbal2chassis.magazine_close;
    g2c_msg.bits.gimbal_imu_cal = comm->gimbal2chassis.gimbal_imu_cal;
    g2c_msg.bits.gimbal_reset = comm->gimbal2chassis.gimbal_reset;
    g2c_msg.bits.reverse_finish_seq = comm->gimbal2chassis.reverse_finish_seq;
    g2c_msg.bits.minipc_ok = comm->gimbal2chassis.minipc_ok;

    memcpy(tx_data + GIMBAL_TX_INDEX, g2c_msg.msg, 8 - GIMBAL_TX_INDEX);
}

/**
 * @brief       get cmd msg from gimbal
 * @param       *comm: ptr to communicator
 * @param       rx_data: data received by gimbal
 * @retval      None
 * @note        None
 */
static void GimbalDecode(ChassisGimbalComm_t *comm, const uint8_t rx_data[8])
{
    Chassis2Gimbal_u c2g_msg;

    memcpy(c2g_msg.msg, rx_data + CHASSIS_TX_INDEX, 8 - CHASSIS_TX_INDEX);
    comm->chassis2gimbal.rc_x = Int16ToFloat(c2g_msg.bits.rc_x_int16);
    comm->chassis2gimbal.rc_y = Int16ToFloat(c2g_msg.bits.rc_y_int16);
    comm->chassis2gimbal.cmd_src = (CmdSrc_e)c2g_msg.bits.cmd_src;
    comm->chassis2gimbal.gimbal_ctrl_mode = (GimbalCtrlMode_e)c2g_msg.bits.gimbal_ctrl_mode;
    comm->chassis2gimbal.shoot = (bool)c2g_msg.bits.shoot;
    comm->chassis2gimbal.shooter_enable = (bool)c2g_msg.bits.shooter_enable;
    comm->chassis2gimbal.ignore_overheated = (bool)c2g_msg.bits.ignore_overheated;
    comm->chassis2gimbal.auto_aim_mode = (AutoAimMode_e)c2g_msg.bits.auto_aim_mode;
    comm->chassis2gimbal.gimbal_enable = (bool)c2g_msg.bits.gimbal_enable;
    comm->chassis2gimbal.reverse_seq = c2g_msg.bits.reverse_seq;
    comm->chassis2gimbal.auto_shoot = (bool)c2g_msg.bits.auto_shoot;
}

static inline int16_t FloatToInt16(float f)
{
    LIMIT_MAX(f, 1, -1);
    return f * 660;
}

static inline float Int16ToFloat(int16_t i)
{
    float f = i / 660.0f;
    LIMIT_MAX(f, 1, -1);
    return f;
}
