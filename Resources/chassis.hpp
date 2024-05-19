/** 
 *******************************************************************************
 * @file      : chassis.hpp
 * @brief     : 机器人结构体定义以及ID等相关参数定义
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
#ifndef __CHASSIS_H_
#define __CHASSIS_H_


// #define WHEEL_MOTOR_8910_150mm
// #define WHEEL_MOTOR_8910_200mm
#define WHEEL_MOTOR_3508_120mm

/* Includes ------------------------------------------------------------------*/
#include "arm_math.h"
#include "system.h"
#include "five_rods_calc.h"
#include "chassis_params.hpp"
#include "chassis_gimbal_comm.h"
#include "motor.hpp"
#include "chassis_motor.hpp"
#include "buzzer.hpp"
#include "judge.hpp"
#include "dist_measure.hpp"
#include "super_capacity.hpp"
#include "chassis_ui_task.hpp"
#include "referee_data.hpp"
namespace buzzer = hello_world::buzzer;
namespace motor = hello_world::motor;
using namespace DistMeasure;
/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/



/* 电机编号 */

#define LFM 0  // 左前关节电机
#define LBM 1  // 左后关节电机
#define RFM 2  // 右前关节电机
#define RBM 3  // 右后关节电机
#define LWM 4  // 左后轮组电机
#define RWM 5  // 右后轮组电机

#define W_INDEX(side) ((side) == LEFT ? LWM : RWM)   // 根据方向获取轮组电机编号
#define FJ_INDEX(side) ((side) == LEFT ? LFM : RFM)  // 根据方向获取前关节电机编号
#define BJ_INDEX(side) ((side) == LEFT ? LBM : RBM)  // 根据方向获取后关节电机编号

#define MIN(x1, x2) ((x1) < (x2) ? (x1) : (x2))  // 返回两个数中的最小值
#define MAX(x1, x2) ((x1) > (x2) ? (x1) : (x2))  // 返回两个数中的最大值

/* 旋转方向编号 */

#define ROLL 0
#define PITCH 1
#define YAW 2

/* 方向轴编号 */

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2

/* 各底盘模式阶段枚举 */

typedef enum _init_state_e {
    INIT_STATE_INIT,
    INIT_STATE_SINGING,
    INIT_STATE_NO_POWER,
    INIT_STATE_RETURN,
} InitState_e;

typedef enum _confirm_state_e {
    CONFIRM_SATTE_INIT,
    CONFIRM_SATTE_RESET,
    CONFIRM_SATTE_CAL_BIAS,
    CONFIRM_STATE_RETURN,
} ConfirmState_e;

typedef enum _recovery_state_e {
    RECOVERY_STATE_INIT,
    RECOVERY_STATE_LOCK,
    RECOVERY_STATE_RECOVERY,
    RECOVERY_STATE_LEG_EXTEND,
    RECOVERY_STATE_TUNE,
    RECOVERY_STATE_STABLE,
    RECOVERY_STATE_RETURN,
} RecoveryState_e;

typedef enum _mature_state_e {
    MATURE_STATE_INIT,
    MATURE_STATE_CTRL,
} MatureState_e;

typedef enum _jump_state_e {
    JUMP_STATE_INIT,
    JUMP_STATE_JUMP,
    JUMP_STATE_RECOVERY,
    JUMP_STATE_AIR,
    JUMP_STATE_RETURN,
} JumpState_e;

typedef enum _fly_state_e {
    FLY_STATE_INIT,
    FLY_STATE_AIR,
    FLY_STATE_RETURN,
} FlyState_e;

typedef enum _abnormal_state_e {
    ALL_STABLE,
    Abnormal_warn,
    Abnormal_dangerous
} AbnormalState_e;

/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

typedef union {
    float32_t data[STATE_NUM];
    struct
    {
        float32_t pos;      // 轮子位置
        float32_t dpos;     // 轮子速度
        float32_t theta;    // 虚拟腿后摆角度
        float32_t dtheta;   // 虚拟后腿摆角速度
        float32_t phi;      // 底盘俯仰角
        float32_t dphi;     // 底盘俯仰角速度
        float32_t height;   // 虚拟腿长度
        float32_t dheight;  // 虚拟腿长度变化率
    } state;
} LegState_u;

typedef struct
{
    LegState_u ref;   // 期望状态
    LegState_u curr;  // 实际状态
} LegState2_t;

/* 底盘模式 */
typedef enum _chassis_mode_e {
    CHASSIS_INIT,
    CHASSIS_DEAD,
    CHASSIS_CONFIRM,
    CHASSIS_RECOVERY,
    CHASSIS_FLY,
    CHASSIS_JUMP,
    CHASSIS_MATURE,
} ChassisMode_e;

typedef struct {
    ChassisMode_e ref;   // 期望状态
    ChassisMode_e curr;  // 实际状态
    ChassisMode_e last;  // 上一次状态
} ChassisMode3_t;

/* 转向模式 */
typedef enum _steer_mode_e {
    STEER_MOVE,
    STEER_DEFENSE,
    STEER_GYRO,
    STEER_DEPART,
} SteerMode_e;

typedef struct {
    SteerMode_e ref;   // 期望状态
    SteerMode_e curr;  // 实际状态
    SteerMode_e last;  // 上一次状态
} SteerMode3_t;


/* 指令结构体 */
typedef struct _cmd_t {
    float32_t height;                // 腿长
    float32_t height_f;              // 腿长（滤波）
    float32_t air_height_diff;               // 腿长
    float32_t dpos;                  // 速度
    float32_t dpos_f;                // 速度（滤波）
    float32_t yaw_ang;               // yaw轴角度
    float32_t pitch_ang;             // pitch轴角度
    float32_t roll_ang;              // roll轴角度
    float32_t gyro_speed;            // 小陀螺转速
    int8_t gyro_dir;                 // 小陀螺转动方向
    bool gyro_cal_flag;                 //是否陀螺补偿计算
    ChassisMode_e chassis_mode_ref;  // 底盘模式
    SteerMode_e steer_mode_ref;      // 转向模式
    
    bool recovery_tune;  // 起身微调
    bool speed_limit;    // 速度限制
    bool start_comm;     // 开始通信
    bool ui_redraw;      // UI界面重画
    bool sup_cap_on;     // 开启超电
    bool roll_ang_fix;   // roll轴角度不可改
    bool auto_jump;      // 等待跳跃

    float32_t gimbal_x;                 // 云台水平方向指令
    float32_t gimbal_y;                 // 云台竖直方向指令
    GimbalCtrlMode_e gimbal_ctrl_mode;  // 云台控制模式
    bool shoot;                         // 发弹
    bool shooter_enable;                 // 云台发射机构启动
    bool ignore_overheated;             // 无视热量显示
    bool gimbal_enable;                 // 云台使能
    bool auto_shoot;                    //依据视觉自动射击
    bool gyro_limit;        
    uint8_t gimbal_reverse_seq;   // 云台反向
    AutoAimMode_e auto_aim_mode;  // 自瞄模式
} Cmd_t;

/* 底盘状态 */
typedef struct _chassis_states_t {
    float32_t dpos;  // 两侧平均速度
    float32_t height;
    float32_t yaw_ang;            // yaw轴角度
    float32_t yaw_omg;            // yaw轴角速度
    float32_t support_forces[2];  // 两侧估计支持力
    float32_t dist2obs[2];        // 距离障碍物的距离
} ChassisStates_t;

/* 底盘检测状态 */
typedef struct _chassis_detect_states_t {
    bool on_ground[3];    // 着地

    bool low_battery;  // 供电源耗尽
    bool abnormal;     // 底盘异常
    AbnormalState_e abnormal_type; 
    bool close2obs;    // 是否接近障碍物
    bool slip[2];         // 是否打滑
    bool gimbal_online;  // 云台在线
} ChassisDetectStates_t;

/* IMU数据 */
typedef struct _imu_datas_t {
    float32_t euler_vals[3];  // 欧拉角度
    float32_t gyro_vals[3];   // 角速度
    float32_t acc_vals[3];    // 加速度
    float32_t yaw_ddphi;        // yaw轴角加速度
} ImuDatas_t;

/* 模式阶段 */
typedef union _mode_state_u {
    InitState_e init;
    ConfirmState_e confirm;
    RecoveryState_e recovery;
    MatureState_e mature;
    JumpState_e jump;
    FlyState_e fly;
} ModeState_u;



/* 底盘结构体 */
typedef struct
{
    LegState2_t leg_states[2];            // 两侧腿状态
    LegState_u leg_states_estimate[2];   //下次估计状态
    float dpos_filter[2];                 // 速度滤波
    ChassisStates_t chassis_states;       // 底盘状态
    ChassisDetectStates_t detect_states;  // 检测状态
    ImuDatas_t imu_datas;                 // IMU数据
    ChassisMode3_t chassis_mode;          // 底盘模式
    SteerMode3_t steer_mode;              // 转向模式
    ModeState_u mode_state;               // 模式阶段
    uint32_t control_tick;  //计时
    Cmd_t cmd;         // 控制指令
    CmdSrc_e cmd_src;  // 控制源
    bool Init_finish_flag; //是否初始化完毕
    bool joint_ang_cal_flag; //是否关节零位计算

    FiveRods_t five_rods_cal;  // 五连杆计算
    motor::Motor* chassis_motors[MOTOR_NUM];              // 底盘电机
    // MotorStates_t motor_states[MOTOR_NUM];  // 底盘电机状态
    float32_t chassis_motor_torques[MOTOR_NUM];     // 底盘电机设定力矩/
    float32_t chassis_motor_vel[6];  //关节电机滤波速度
    motor::Motor* yaw_motor;               // yaw轴电机
    float32_t       yaw_motor_input;
    // MotorStates_t yaw_motor_states;  // yaw轴电机状态
    float32_t gyro_forward_angle;               //小陀螺前进方向
    float32_t gyro_forward_speed;               //小陀螺前进速度
    buzzer::Buzzer* chassis_buzzer;           // 蜂鸣器
    ChassisGimbalComm_t comm;  // 底盘云台通信
    super_capacity* sup_cap;          // 超电
    RobotReferee *referee_ptr;          //裁判系统通讯
    // Tfluna_t dist_measurement[2];  // 测距
    DistMeasure::dist_measure* dist_measurement[2];  // 测距
    uint32_t mature_tick;                 // 正常模式计时
    uint32_t gimbal_comm_tick;                   // 云台通讯时刻

    // UiDrawer* ui_drawer;         // UI界面
    Referee * ui_drawer;         // UI界面
} BalanceRobot_t;


extern BalanceRobot_t robot;


/**
 * @brief       底盘初始化
 * @retval      None
 * @note        None
 */
void RobotInit(void);

/**
 * @brief       限定变化率
 * @param       ref_vel: 期望值
 * @param       curr_vel: 当前值
 * @param       max_diff: 最大的变化速率
 * @retval      符合限定变化率的值
 * @note        None
 */
inline static float32_t LimDiff(float32_t ref_vel, float32_t curr_vel, float32_t max_diff)
{
    float32_t dvel = ref_vel - curr_vel;

    /* 限定变化差值 */
    if (dvel > max_diff * kCtrlPeriod) {
        dvel = max_diff * kCtrlPeriod;
    } else if (dvel < -max_diff * kCtrlPeriod) {
        dvel = -max_diff * kCtrlPeriod;
    }

    return curr_vel + dvel;
}

extern int debug_num;

#endif /* __FILE_H_ */
