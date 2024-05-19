/**
 *******************************************************************************
 * @file      : chassis_gimbal_comm.h
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CHASSIS_GIMBEL_COMM_H_
#define __CHASSIS_GIMBEL_COMM_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

/* Exported macro ------------------------------------------------------------*/
/* CAN ID配置 */

#define JUDGE_TX_ID 0x1FD    // 裁判系统数据帧的ID
#define JUDGE_TX_INDEX 0     // 裁判系统数据在数据中的起始位置
#define CHASSIS_TX_ID 0x1AB  // 底盘控制数据帧的ID
#define CHASSIS_TX_INDEX 0   // 底盘控制数据在数据中的起始位置
#define GIMBAL_TX_ID 0x1AC   // 云台反馈数据帧的ID
#define GIMBAL_TX_INDEX 0    // 云台反馈数据在数据中的起始位置
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/* 控制源 */
typedef enum _cmd_src_e {
    REMOTE,     // 遥控器
    KEY_MOUSE,  // 键鼠
} CmdSrc_e;

/* 云台控制模式 */
typedef enum _gimbal_ctrl_mode_e {
    GIMBAL_MANUAL,  // 手动
    GIMBAL_AUTO,    // 自动
} GimbalCtrlMode_e;

/* 自瞄模式 */
typedef enum _auto_aim_mode_e {
    AUTO_AIM_CLOSE,   // 自瞄关闭
    BIG_BUFF,         // 大符
    SMALL_BUFF,       // 小符
    ANTI_TWIST,       // 反陀螺
    PREDICT,          // 平动预测
    AUTO_JUDGE,       // 模式自判断
    MINIPC_REBOOT,    // minipc重启
    MINIPC_SHUTDOWN,  // minipc关机
} AutoAimMode_e;

/* 步兵类型 */
typedef enum _infantry_type_e {
    NORMAL_INFANTRY = 0,   // 普通步兵
    BALANCE_INFANTRY = 1,  // 平衡步兵
} InfantryType_e;

/* 敌方颜色 */
typedef enum _enemy_color_e {
    UNKNOWN = 0,  // 未知
    BLUE = 1,     // 蓝方
    RED = 2,      // 红方
} EnemyColot_e;

/* 弹速限制 */
typedef enum _shooter_speed_limit_e {
    SPEED_UNKNOWN = 0,  // 未知
    SPEED_15_MPS = 1,   // 15m/s
    SPEED_18_MPS = 2,   // 18m/s
    SPEED_30_MPS = 3,   // 30m/s
} ShooterSpeedLimit_e;

// 识别目标类型
typedef enum _target_e {
    SENTRY = 0x01 << 0,      // 哨兵
    HERO = 0x01 << 1,        // 英雄
    ENGINEER = 0x01 << 2,    // 工程
    INFANTRY_3 = 0x01 << 3,  // 3号步兵
    INFANTRY_4 = 0x01 << 4,  // 4号步兵
    INFANTRY_5 = 0x01 << 5,  // 5号步兵
    OUTPOST = 0x01 << 6,     // 前哨站
    BASE = 0x01 << 7,        // 基地
} Target_e;


typedef enum TargetState{
    kTargetStateLost=0u,
    kTargetStateDetected=1u,
    kTargetStateAimed=2u,
}TargetState_e;

/* 底盘云台通信 */
typedef struct _chassis_gimbal_comm_t ChassisGimbalComm_t;
struct _chassis_gimbal_comm_t {
    struct {
        float bullet_speed;                       // 实际弹速，单位：m
        ShooterSpeedLimit_e shooter_speed_limit;  // 弹速限制
        uint16_t shooter_cooling_remain;          // 剩余枪口热量
        struct {
            InfantryType_e type_3;         // 3号步兵类型
            InfantryType_e type_4;         // 4号步兵类型
            InfantryType_e type_5;         // 5号步兵类型
        } infantry_types;                  // 步兵类型
        bool gimbal_power_on;              // 云台上电
        bool shooter_power_on;             // 发射机构上电
        uint8_t shooter_speed_update_seq;  // 弹速更新次序
        uint8_t shooter_heat_update_seq;   // 枪口热量更新次序
        EnemyColot_e enemy_color;          // 敌方颜色
    } judge2gimbal;                        // 裁判系统数据

    struct {
        float rc_x;                         // 云台水平控制指令，[-1, 1]
        float rc_y;                         // 云台竖直控制指令，[-1, 1]
        CmdSrc_e cmd_src;                   // 控制源
        GimbalCtrlMode_e gimbal_ctrl_mode;  // 云台控制模式
        bool shoot;                         // 发弹
        bool shooter_enable;                 // 云台发射机构
        bool ignore_overheated;             // 无视热量限制
        bool gimbal_enable;                 // 云台使能
        bool auto_shoot;
        AutoAimMode_e auto_aim_mode;  // 自瞄模式
        uint8_t reverse_seq;          // 云台反向次序
    } chassis2gimbal;                 // 底盘控制数据

    struct {
        bool magazine_close;  // 弹仓关闭
        bool gimbal_imu_cal;  // 云台IMU完成校准
        bool gimbal_reset;    // 云台完成复位
        bool minipc_ok;
        uint8_t reverse_finish_seq;  // 云台反向完成次序
        Target_e target_list;        // 识别目标
        uint8_t aimed_enemy_id;      // 自瞄目标序号
        uint8_t target_x;            // 目标x坐标
        uint8_t target_y;            // 目标y坐标
        uint8_t target_state;        // 目标状态

    } gimbal2chassis;                // 云台反馈数据

    /**
     * @brief       get msg of the info from judge on chassis
     * @param       *self: ptr to communicator
     * @param       tx_data: info of judge to be sent by chassis
     * @retval      None
     * @note        None
     */
    void (*judgeEncode)(const ChassisGimbalComm_t *self, uint8_t tx_data[8]);

    /**
     * @brief       get msg to be sent to judge from gimbal
     * @param       *self: ptr to communicator
     * @param       rx_data: data received by judge
     * @retval      None
     * @note        None
     */
    void (*judgeDecode)(ChassisGimbalComm_t *self, const uint8_t rx_data[8]);

    /**
     * @brief       get msg and id of the cmd that chassis send to gimbal
     * @param       *self: ptr to communicator
     * @param       tx_data: data to be sent by chassis
     * @retval      None
     * @note        None
     */
    void (*chassisEncode)(const ChassisGimbalComm_t *self, uint8_t tx_data[8]);

    /**
     * @brief       get feedback msg from gimbal when ID matches
     * @param       *self: ptr to communicator
     * @param       rx_data: data received by chassis
     * @retval      None
     * @note        None
     */
    void (*chassisDecode)(ChassisGimbalComm_t *self, const uint8_t rx_data[8]);

    /**
     * @brief       get msg and id of the feedback that gimbal send to chassis
     * @param       *self: ptr to communicator
     * @param       tx_data: data to be sent by gimbal
     * @retval      None
     * @note        None
     */
    void (*gimbalEncode)(const ChassisGimbalComm_t *self, uint8_t tx_data[8]);

    /**
     * @brief       get cmd msg from gimbal when ID matches
     * @param       *self: ptr to communicator
     * @param       rx_data: data received by gimbal
     * @retval      None
     * @note        None
     */
    void (*gimbalDecode)(ChassisGimbalComm_t *self, const uint8_t rx_data[8]);
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

/**
 * @brief       init communicator
 * @param       *comm: ptr to communicator
 * @retval      None
 * @note        None
 */
void ChassisGimbalCommInit(ChassisGimbalComm_t *comm);

#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_GIMBEL_COMM_H_ */
