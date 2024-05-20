/**
 *******************************************************************************
 * @file      : chassis_ctrl_task.cpp
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
/* Includes ------------------------------------------------------------------*/
#include "system.h"
#include "chassis_ctrl_task.hpp"
#include "chassis_ctrl_params.hpp"
#include "pid.hpp"
#include "chassis.hpp"
#include "tools.h"
#include "HW_fdcan.hpp"
#include "buzzer.hpp"
#include "td.hpp"
#include "chassis_task.hpp"
#include "chassis_cmd_task.hpp"
#include "chassis_sense_task.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
typedef struct _leg_len_ctrl_t
{
    float k1;
    float k3;
    float b;
    float ki;
    float iout;
    float max_iout;
    float max_out;
    float min_out;
} LegLenCtrl_t;
/* Private variables ---------------------------------------------------------*/
namespace TD = hello_world::filter;
pid::MultiNodesPid *jointpid_ptr[4] = {nullptr, nullptr, nullptr, nullptr};     // 关节pid控制
pid::BasicPid *joint_confirm_pid_ptr[4] = {nullptr, nullptr, nullptr, nullptr}; // 关节确定零位pid控制
pid::BasicPid *Wheelpid_ptr[2] = {nullptr, nullptr};                            // 轮电机速度闭环pid控制
pid::BasicPid *WheelSlippid_ptr[2] = {nullptr, nullptr};                        // 轮电机打滑pid控制
pid::MultiNodesPid *WheelRecoverypid_ptr[2] = {nullptr, nullptr};               // 轮电机起身pid控制
pid::BasicPid *leg_ang_pid_ptr = nullptr;                                       // 腿摆角pid控制
pid::MultiNodesPid *yaw_tq_pid_ptr = nullptr;                                   // 转向力矩pid控制


// pid::BasicPid *yaw_tq_pid_ptr = nullptr;                                        // 转向力矩pid控制
pid::BasicPid *roll_pid_ptr = nullptr;           // roll力矩控制
pid::MultiNodesPid *yaw_motor_pid_ptr = nullptr; // 临时yaw轴pid控制
// TD::Td *joint_motor_td[4] = {nullptr};
// pid::MultiNodesPid* leg_ang_pid=nullptr;
static float mature_lqr[2][2][6];
static float recover_lqr[4];
static float out[6];
static LegLenCtrl_t leg_len_ctrls[2];
float test_force[6] = {0};
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

static void ChassisMotorAction(void);

/**
 * @brief      清除所有电机输出
 * @retval      None
 * @note        None
 */
static void ClearAllMotors(void);

static void ChassisModeChangeHandle(void);

static void SteerControl(void);

inline static void YawTqPidReset(void);

inline static void RollPidReset(void);

static float PolyVal(const float *p, float x, uint8_t n);

static void CalLqrMat(void);

static void InitController(void);

static void ConfirmController(void);

inline static void ConfirmPidReset(void);

static void DeadController(void);

static void JointCtrlByPid(float leg_len, float leg_ang, float ffd_force);

inline static void JointPidReset(void);

static void RecoveryController(void);

static float GetLimitedSpeed(float dpos_ref);

static void MatureController(void);

inline static void LegAngPidReset(void);

static void JumpController(void);

static void FlyController(void);

inline static void WheelPidsReset(void);

void JointVelFilter(void);

void YawMotorContorller(void);

float CalTurnTorDiff(void);

/**
 * @brief       init control task
 * @retval      None
 * @note        None
 */
void CtrlInit(void)
{

    for (uint8_t i = 0; i < 4; i++)
    {
        // jointpid_ptr[i] = new pid::MultiNodesPid(pid::MultiNodesPidType::kMultiNodesPidTypeCascade, pid::OutLimit(true,-30.0f, 30.0f), 2, kJointPidsParams);
        jointpid_ptr[i] = new pid::MultiNodesPid(pid::MultiNodesPidType::kMultiNodesPidTypeCascade, pid::OutLimit(true, -30.0f, 30.0f),
                                                 pid::MultiNodesPid::ParamsList(kJointPidsParams, kJointPidsParams + 2));
        // joint_confirm_pid_ptr[i] = new pid::MultiNodesPid(pid::MultiNodesPidType::kMultiNodesPidTypeCascade, pid::OutLimit(true,-30.0f, 30.0f), 1, &kJointConfirmPidsParams);
        joint_confirm_pid_ptr[i] = new pid::BasicPid(kJointConfirmPidsParams);
        // joint_motor_td[i] = new TD::Td(100.0f, 0.001f);
    }

    for (uint8_t i = 0; i < 2; i++)
    {
        Wheelpid_ptr[i] = new pid::BasicPid(kWheelPidsParams);
        WheelRecoverypid_ptr[i] = new pid::MultiNodesPid(pid::MultiNodesPidType::kMultiNodesPidTypeCascade, pid::OutLimit(true, -16.0f, 16.0f),
                                                         pid::MultiNodesPid::ParamsList(kWheelRecoveryPidsParams, kWheelRecoveryPidsParams + 2));
        leg_len_ctrls[i].k1 = kLegLenCtrlK1;
        leg_len_ctrls[i].k3 = kLegLenCtrlK3;
        leg_len_ctrls[i].b = kLegLenCtrlB;
        leg_len_ctrls[i].ki = kLegLenCtrlKi;
        leg_len_ctrls[i].max_iout = kLegLenCtrlMaxIout;
        leg_len_ctrls[i].max_out = kLegLenCtrlMaxOut;
        leg_len_ctrls[i].min_out = kLegLenCtrlMinOut;
    }

    // // PidInit(&leg_ang_pid, 1, &kLegAngPidParams);
    leg_ang_pid_ptr = new pid::BasicPid(kLegAngPidParams);
    // // PidInit(&yaw_tq_pid, 2, kYawTqPidParams);
    yaw_tq_pid_ptr = new pid::MultiNodesPid(pid::MultiNodesPidType::kMultiNodesPidTypeCascade, pid::OutLimit(true, -8.0f, 8.0f),
                                            pid::MultiNodesPid::ParamsList(kYawTqPidParams, kYawTqPidParams + 2));
    // yaw_tq_pid_ptr = new pid::BasicPid(kYawTqPidParams);
    // // PidInit(&roll_pid, 1, &kRollPidParams);
    roll_pid_ptr = new pid::BasicPid(kRollPidParams);

    for (uint8_t i = 0; i < 2; i++)
    {
        for (uint8_t j = 0; j < 6; j++)
        {
            mature_lqr[0][i][j] = mature_lqr[1][i][j] = 0;
        }
    }
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
}

// #define TEST_MODE

float test_angle1, test_angle2 = 0.0f;
float32_t test_tor = 0.0f;
float test_id = 0.0f;
float type = 0;
float test_v1, test_v2 = 0.0f;
float motorangle[6] = {0};
float motorvel[6] = {0};
float yaw_motor_angle = 0.0f;
float32_t input_tor[6] = {0};
/**
 * @brief       task to control chassis
 * @arg         None
 * @retval      None
 * @note        None
 */
void CtrlTask(void)
{
    ClearAllMotors();

    ChassisModeChangeHandle();

    JointVelFilter();

    memset(StateSpaceModelU, 0, 2 * 2 * sizeof(float));

    switch (robot.chassis_mode.curr)
    {
    case CHASSIS_INIT:
        InitController();
        break;
    case CHASSIS_CONFIRM:
        ConfirmController();
        break;
    case CHASSIS_DEAD:
        DeadController();
        break;
    case CHASSIS_RECOVERY:
        RecoveryController();
        break;
    case CHASSIS_MATURE:
        MatureController();
        break;

    case CHASSIS_JUMP:
        JumpController();
        break;
    case CHASSIS_FLY:
        FlyController();
        break;
    default:

        break;
    }

    for (int i = 0; i < MOTOR_NUM; i++)
    {
        motorangle[i] = robot.chassis_motors[i]->angle();
        input_tor[i] = robot.chassis_motor_torques[i];
        motorvel[i] = robot.chassis_motor_vel[i];
        test_angle1 = robot.yaw_motor->angle();
    }

    // robot.chassis_motor_torques[LBM]=robot.chassis_motor_torques[LFM]=0.0f;
    // robot.chassis_motor_torques[LWM]=0.0f;
    // robot.chassis_motor_torques[RWM]=0.3f;

#ifndef TEST_MODE
    ChassisMotorAction();
#endif
    StateEstimate();
}

static void ClearAllMotors()
{
    for (int i = 0; i < MOTOR_NUM; i++)
    {

        robot.chassis_motors[i]->setInput(0);
        robot.chassis_motor_torques[i] = 0;
    }
    // robot.yaw_motor->setInput(0);
    // robot.yaw_motor_input = 0;
    // test_motor_ptr->setInput(0);
}
float32_t wheel_input_f[2] = {0, 0};
/**
 * @brief       set all motors' torque
 * @retval      None
 * @note
 */
static void ChassisMotorAction(void)
{
    for (uint8_t i = 0; i < MOTOR_NUM; i++)
    {
        robot.chassis_motors[i]->setInput(robot.chassis_motor_torques[i]);
    }
    robot.yaw_motor->setInput(robot.yaw_motor_input);
}

static void InitController(void)
{
    if (robot.chassis_buzzer->is_playing())
    {
        robot.chassis_buzzer->play();
    }
    if (robot.Init_finish_flag == true)
    {
        // 关节电机上电
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
        robot.joint_ang_cal_flag = false;
        robot.chassis_mode.ref = robot.chassis_mode.curr = CHASSIS_DEAD;
        robot.chassis_mode.last = CHASSIS_INIT;
        robot.steer_mode.ref = robot.steer_mode.curr = robot.steer_mode.last = STEER_DEPART;
    }
}
static void DeadController(void)
{
    robot.cmd.start_comm = true;
    robot.cmd.sup_cap_on = false;
}
inline static float LegLenCtrlCal(LegLenCtrl_t *ctrl, float h_ref, float h_fdb, float dh_fdb)
{
    float out;
    float err = h_ref - h_fdb;
    ctrl->iout += ctrl->ki * err * kCtrlPeriod;
    LIMIT_MAX(ctrl->iout, ctrl->max_iout, -ctrl->iout);

    out = ctrl->k1 * err + ctrl->k3 * err * err * err - ctrl->b * dh_fdb + ctrl->iout;
    LIMIT_MAX(out, ctrl->max_out, ctrl->min_out);

    return out;
}
/**
 * @brief       模式切换时将模式所处阶段设置为初始化阶段
 * @arg         None
 * @note        None
 */
static void ChassisModeChangeHandle(void)
{
    static ChassisMode_e last_chassis_mode = CHASSIS_DEAD;

    if (last_chassis_mode == robot.chassis_mode.curr)
    {
        return;
    }

    if (robot.chassis_mode.curr != CHASSIS_INIT)
    {
        robot.mature_tick = 0;
    }

    last_chassis_mode = robot.chassis_mode.curr; // 更新上次底盘模式

    switch (robot.chassis_mode.curr)
    {
    case CHASSIS_INIT:
        robot.mode_state.init = INIT_STATE_INIT;
        break;
    case CHASSIS_CONFIRM:
        robot.mode_state.confirm = CONFIRM_SATTE_INIT;
        break;
    case CHASSIS_RECOVERY:
        robot.mode_state.recovery = RECOVERY_STATE_INIT;
        break;
    case CHASSIS_MATURE:
        robot.mode_state.mature = MATURE_STATE_INIT;
        break;
    case CHASSIS_JUMP:
        robot.mode_state.jump = JUMP_STATE_INIT;
        break;
    case CHASSIS_FLY:
        robot.mode_state.fly = FLY_STATE_INIT;
        break;
    default:
        break;
    }
}
// /**
//  * @brief       controller used at INIT mode
//  * @retval      None
//  * @note        None
//  */
// static void InitController(void)
// {
//     switch (robot.mode_state.init) {
//         case INIT_STATE_INIT:
//             robot.mode_state.init = INIT_STATE_SINGING;
//         case INIT_STATE_SINGING:
//             if (robot.buzzer.isPlaying(&robot.buzzer)) {
//                 robot.buzzer.ctrlHook(&robot.buzzer);
//             } else {
//                 robot.cmd.start_comm = true;
//                 robot.cmd.gimbal_enable = true;

//                 robot.mode_state.init = INIT_STATE_NO_POWER;
//             }
//             break;
//         case INIT_STATE_NO_POWER:
//             if (js_mains_power_chassis_output) {
//                 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, SET);
//                 robot.mode_state.init = INIT_STATE_RETURN;
//             }
//             break;
//         case INIT_STATE_RETURN:
//             robot.chassis_mode.ref = robot.chassis_mode.curr = CHASSIS_DEAD;
//             robot.chassis_mode.last = CHASSIS_INIT;
//             robot.steer_mode.ref = robot.steer_mode.curr = robot.steer_mode.last = STEER_DEPART;
//             break;
//         default:
//             break;
//     }
// }
static bool confirm_finish_flag = false;
/**
 * @brief      计算关节电机零位
 * @retval      None
 * @note        None
 */
static void ConfirmController(void)
{
    static uint16_t stop_time[4]; // counters of joint motors reaching mechanical limit
    float ref, output, fdb;

    switch (robot.mode_state.confirm)
    {
    case CONFIRM_SATTE_INIT:
        ConfirmPidReset();
        memset(stop_time, 0, 4 * sizeof(uint16_t));
        if (confirm_finish_flag)
        {
            robot.mode_state.confirm = CONFIRM_STATE_RETURN;
        }
        else
        {
            robot.mode_state.confirm = CONFIRM_SATTE_RESET;
        }
    case CONFIRM_SATTE_RESET:
        robot.mode_state.confirm = CONFIRM_SATTE_CAL_BIAS;
        for (uint8_t i = 0; i < 4; i++)
        {
            /* use pid to turn joint motors in a certain speed*/
            if (i == LFM || i == RFM)
            {
                ref = -kComfirmMotorSpeed;
            }
            else
            {
                ref = kComfirmMotorSpeed;
            }
            fdb = robot.chassis_motor_vel[i];
            joint_confirm_pid_ptr[i]->calc(&ref, &fdb, nullptr, &output);
            robot.chassis_motor_torques[i] = output;

            /* judge whether joint motor reaches the mechanical limit */
            if (fabsf(robot.chassis_motor_vel[i]) < kComfirmStopSpeedThres)
            {
                stop_time[i]++;
            }
            else
            {
                stop_time[i] = 0;
            }

            if (stop_time[i] * kCtrlPeriod < kComfirmStopTimeThres)
            {
                robot.mode_state.confirm = CONFIRM_SATTE_RESET;
            }
        }
        break;

    case CONFIRM_SATTE_CAL_BIAS:
        /* calculate joint motors' bias angle */
        if (robot.mode_state.confirm == CONFIRM_SATTE_CAL_BIAS)
        {
            for (uint8_t i = 0; i < MOTOR_NUM; i++)
            {
                robot.chassis_motors[i]->setAngleValue(kMechanicalLimAng[i]);
            }
            robot.joint_ang_cal_flag = true;
            confirm_finish_flag = true;
            memset(stop_time, 0, 4 * sizeof(uint16_t));
            robot.mode_state.confirm = CONFIRM_STATE_RETURN;
        }
        break;
    case CONFIRM_STATE_RETURN:
        if (robot.cmd.recovery_tune)
        {
            robot.chassis_mode.ref = robot.chassis_mode.curr = CHASSIS_RECOVERY;
            robot.chassis_mode.last = CHASSIS_CONFIRM;
            robot.steer_mode.ref = robot.steer_mode.curr = robot.steer_mode.last = STEER_DEPART;
        }
        else
        {
            robot.cmd.height = kHeightMin;
            robot.cmd.height_f = kHeightMin;
            // robot.cmd.yaw_ang = robot.yaw_motor->angle();
            robot.cmd.yaw_ang = robot.chassis_states.yaw_ang;
            robot.leg_states[LEFT].ref.state.pos = robot.leg_states[RIGHT].ref.state.pos = 0;
            robot.leg_states[LEFT].curr.state.pos = robot.leg_states[RIGHT].curr.state.pos = 0;

            robot.chassis_mode.ref = robot.chassis_mode.curr = CHASSIS_MATURE;
            // robot.chassis_mode.ref = robot.chassis_mode.curr = CHASSIS_RECOVERY;
            debug_num = 5;
            robot.steer_mode.ref = robot.steer_mode.curr = STEER_MOVE;
            robot.steer_mode.last = STEER_DEPART;
            // robot.steer_mode.ref = robot.steer_mode.curr = STEER_DEPART;
        }
        break;
    default:
        break;
    }
}

float roll_df = 0, dtq = 0;
float mature_dpos = 0;
float rotate_pos = 0;
float last_rotate_pos = 0;
float rotate_dpos = 0;
/**
 * @brief       正常运行模式
 * @retval      None
 * @note        None
 */
static void MatureController(void)
{
    float diff_pos = 0;
    static int abnormal_warn_flag = 0;
    float rotate_height = 0.0f; // 旋转降低腿长
    switch (robot.mode_state.mature)
    {
    case MATURE_STATE_INIT:
        robot.mode_state.mature = MATURE_STATE_CTRL;
        // YawTqPidReset();
        // RollPidReset();
        // LegLenPidReset();
        JointPidReset();
        LegAngPidReset();
        YawTqPidReset();
        RollPidReset();
        robot.mature_tick = 0;
        robot.cmd.sup_cap_on = true;
        rotate_dpos = 0;
        rotate_pos = 0;
        last_rotate_pos = 0;
        abnormal_warn_flag = 0;
        if (robot.steer_mode.curr == STEER_MOVE)
        {
            robot.cmd.yaw_ang = robot.yaw_motor->angle();
        }
        break;
    case MATURE_STATE_CTRL:
        robot.mature_tick++;
        // for (int i=0;i<4;i++){
        //     float fdb = robot.chassis_motors[i]->vel();
        //     ref=;
        //     float output=0;
        //     joint_confirm_pid_ptr[i]->calc(&ref, &fdb, nullptr, &output);
        //     robot.chassis_motor_torques[i] = output;
        // }
        if (fabs(robot.chassis_states.yaw_omg) > 3.0f && robot.steer_mode.curr != STEER_GYRO)
        {
            rotate_height = (fabs(robot.chassis_states.yaw_omg * robot.chassis_states.dpos)) * 0.15f;
        }
        robot.cmd.height_f = LimDiff(robot.cmd.height - rotate_height, robot.cmd.height_f, kMaxDH);
        robot.cmd.pitch_ang = LimDiff(0, robot.cmd.pitch_ang, kPitchMaxDAng);
        robot.cmd.air_height_diff = LimDiff(0, robot.cmd.air_height_diff, kMaxDH);
        // robot.cmd.pitch_ang = 0;
        robot.leg_states[LEFT].ref.state.phi = robot.leg_states[RIGHT].ref.state.phi = robot.cmd.pitch_ang;
        // robot.cmd.pitch_ang;

        robot.leg_states[LEFT].ref.state.height = (robot.cmd.height_f) / cos(robot.leg_states[LEFT].curr.state.theta);
        robot.leg_states[RIGHT].ref.state.height = (robot.cmd.height_f) / cos(robot.leg_states[RIGHT].curr.state.theta);
        // robot.leg_states[LEFT].ref.state.height = robot.cmd.height_f;
        // robot.leg_states[RIGHT].ref.state.height = robot.cmd.height_f;
        LIMIT_MAX(robot.leg_states[LEFT].ref.state.height, kHeightMax, kHeightMin);
        LIMIT_MAX(robot.leg_states[RIGHT].ref.state.height, kHeightMax, kHeightMin);

        // robot.cmd.dpos_f = LimDiff(robot.cmd.dpos, robot.cmd.dpos_f, kMaxDDPos);
        robot.cmd.dpos_f = robot.cmd.dpos;
        float dpos = GetLimitedSpeed(robot.cmd.dpos_f);
        mature_dpos = dpos;
        // float dpos = robot.cmd.dpos;

        float diff_pos =
            (robot.leg_states[LEFT].ref.state.pos + robot.leg_states[RIGHT].ref.state.pos) / 2 + dpos * kCtrlPeriod -
            (robot.leg_states[LEFT].curr.state.pos + robot.leg_states[RIGHT].curr.state.pos) / 2;
        if (fabsf(diff_pos) < kPosMaxBias)
        {
            robot.leg_states[LEFT].ref.state.pos += dpos * kCtrlPeriod;
            robot.leg_states[RIGHT].ref.state.pos += dpos * kCtrlPeriod;
        }

        if (robot.detect_states.abnormal && robot.detect_states.abnormal_type == Abnormal_warn && robot.mature_tick > 1000)
        {
            JointCtrlByPid(kHeightMin, PI / 2, kBodyMass * kGravAcc);
            if (!abnormal_warn_flag)
            {
                abnormal_warn_flag = 1;
                for (int i = 0; i < 2; i++)
                {
                    robot.leg_states[i].ref.state.pos = hello_world::Bound(robot.leg_states[i].curr.state.pos,
                                                                           robot.leg_states[i].curr.state.pos + kAbnormalPosBias, robot.leg_states[i].curr.state.pos - kAbnormalPosBias);
                }
            }
        }
        else
        {
            abnormal_warn_flag = 0;
        }

        // 计算旋转导致的位置差

        // robot.leg_states[LEFT].ref.state.pos -= rotate_pos-last_rotate_pos;
        // robot.leg_states[RIGHT].ref.state.pos += rotate_pos-last_rotate_pos;
        // JointCtrlByPid(kHeightMin, PI / 2, kBodyMass * kGravAcc);

        CalLqrMat();
        // CalTurnTorDiff();

        float leg_ang_fdb;
        float leg_push_force[2] = {0, 0};
        float leg_tq[2] = {0, 0};
        leg_ang_fdb = robot.leg_states[RIGHT].curr.state.theta - robot.leg_states[LEFT].curr.state.theta;
        // roll_pid_ptr.calc(&roll_pid, robot.cmd.roll_ang, robot.imu_datas.euler_vals[ROLL], &df);
        roll_pid_ptr->calc(&robot.cmd.roll_ang, &robot.imu_datas.euler_vals[ROLL], nullptr, &roll_df);
        // leg_ang_pid.calcPid(&leg_ang_pid, 0, &fdb, &dtq);
        float ref = 0;
        leg_ang_pid_ptr->calc(&ref, &leg_ang_fdb, nullptr, &dtq);
        float front_tq, behind_tq;
        leg_tq[0] = -dtq;
        leg_tq[1] = dtq;
        leg_push_force[0] = roll_df;
        leg_push_force[1] = -roll_df;
        test_force[0] = roll_df;
        test_force[1] = dtq;

        float rot_dpos = robot.chassis_states.yaw_omg * kWheelBase / 2;
        // if(fabs(-robot.cmd.yaw_ang+robot.chassis_states.yaw_ang)<D2R(5.0f))
        // {
        //     rotate_pos = hello_world::HandleAngleCross0Rad(-robot.cmd.yaw_ang+robot.chassis_states.yaw_ang,0)*kWheelBase/2*3.0f;
        // }
        // else {
        //     rotate_pos = SIGN(-robot.cmd.yaw_ang+robot.chassis_states.yaw_ang)*kRotPosMaxBias;
        // }
        // rotate_pos = hello_world::HandleAngleCross0Rad(-robot.cmd.yaw_ang+robot.chassis_states.yaw_ang,0)*kWheelBase/2*5.0f;
        // rotate_pos = hello_world::Bound(rotate_pos, -kRotPosMaxBias, kRotPosMaxBias);
        // rotate_pos = 0.0f;
        // rot_dpos = 0.0f;
        // robot.leg_states[LEFT].ref.state.pos -= rotate_pos;
        // robot.leg_states[RIGHT].ref.state.pos += rotate_pos;
        // robot.leg_states[LEFT].curr.state.dpos += rot_dpos;
        // robot.leg_states[RIGHT].curr.state.dpos -= rot_dpos;

        /* get motor output torque in virtual leg model*/
        for (uint8_t i = 0; i < 2; i++)
        {
            for (uint8_t j = 0; j < 6; j++)
            {
                leg_tq[i] += mature_lqr[i][0][j] *
                             (robot.leg_states[i].ref.data[j] - robot.leg_states[i].curr.data[j]) / 2;
                robot.chassis_motor_torques[W_INDEX(i)] +=
                    mature_lqr[i][1][j] * (robot.leg_states[i].ref.data[j] - robot.leg_states[i].curr.data[j]) / 2;
            }

            robot.chassis_motor_torques[W_INDEX(i)] += hello_world::Bound((robot.leg_states_estimate[i].state.dtheta - robot.leg_states[i].curr.state.dtheta) * kSlipWheelTorCoff, -kSlipWheelTorLimit, kSlipWheelTorLimit);
            leg_tq[i] -= hello_world::Bound((robot.leg_states_estimate[i].state.dtheta - robot.leg_states[i].curr.state.dtheta) * kSlipWheelTorCoff, -kSlipWheelTorLimit, kSlipWheelTorLimit);
            /* perform virtual leg length control */
            leg_push_force[i] += LegLenCtrlCal(
                leg_len_ctrls + i, robot.leg_states[i].ref.state.height, robot.leg_states[i].curr.state.height,
                robot.leg_states[i].curr.state.dheight);
            if (i == 0)
            {
                test_force[2] = leg_push_force[i] - test_force[0];
            }
            if (!robot.detect_states.abnormal)
            {
                leg_push_force[i] += kBodyMass * kGravAcc * cos(robot.leg_states[i].curr.state.theta) / 2;
            }

            /* convert operation space to joint space */
            robot.five_rods_cal.opSp2JointSp(&robot.five_rods_cal, &front_tq, &behind_tq, leg_push_force[i], leg_tq[i],
                                             robot.chassis_motors[FJ_INDEX(i)]->angle(), robot.chassis_motors[BJ_INDEX(i)]->angle());
            robot.chassis_motor_torques[FJ_INDEX(i)] += front_tq;
            robot.chassis_motor_torques[BJ_INDEX(i)] += behind_tq;
            StateSpaceModelU[i][0] = leg_tq[i];
            StateSpaceModelU[i][1] = robot.chassis_motor_torques[W_INDEX(i)];
            // if (robot.leg_states[i].curr.state.height < 0.11f)
            // {
            //     robot.chassis_motor_torques[FJ_INDEX(i)] += 1.5f;
            //     robot.chassis_motor_torques[BJ_INDEX(i)] -= 1.5f;
            // }
        }
        // robot.leg_states[LEFT].ref.state.pos += rotate_pos;
        // robot.leg_states[RIGHT].ref.state.pos -= rotate_pos;

        // if(robot.detect_states.slip[0]||robot.detect_states.slip[1])
        // {
        // robot.chassis_motor_torques[LWM] += hello_world::Bound((robot.leg_states_estimate[LEFT].state.dtheta-robot.leg_states[LEFT].curr.state.dtheta)*kSlipWheelTorCoff,-kSlipWheelTorLimit,kSlipWheelTorLimit);
        // robot.chassis_motor_torques[RWM] += hello_world::Bound((robot.leg_states_estimate[RIGHT].state.dtheta-robot.leg_states[RIGHT].curr.state.dtheta)*kSlipWheelTorCoff,-kSlipWheelTorLimit,kSlipWheelTorLimit);
        // }
        // if(!robot.detect_states.abnormal&&!robot.detect_states.slip[0]&&!robot.detect_states.slip[1]){
        SteerControl();
        // }

        // robot.chassis_motor_torques[LWM]=robot.chassis_motor_torques[RWM]=3.0f;
        // YawMotorContorller();
        break;
    }
}

/**
 * @brief       controller used at RECOVERY mode
 * @retval      None
 * @note        None
 */
static void RecoveryController(void)
{
    static uint32_t stable_time = 0;
    static uint32_t height_time = 0;
    static uint32_t extend_time = 0;
    static uint32_t lock_time = 0;
    float tq;
    static bool leg_extend_flag = false; // 已伸长置1
    float32_t leg_extend_ang = PI / 2;
    static float32_t extend_ref[2] = {0, 0}; // 0 前轮 1 后轮
    static float32_t ref_ang[4];
    static float32_t fdb_ang = 0.0f;
    static float recover_wheel_ref[2][2] = {{robot.chassis_motors[LWM]->angle(), 0},
                                            {robot.chassis_motors[RWM]->angle(), 0}};
    /* ignore user cmd */
    memcpy(robot.leg_states[LEFT].ref.data, kRefState, STATE_NUM * sizeof(float));
    memcpy(robot.leg_states[RIGHT].ref.data, kRefState, STATE_NUM * sizeof(float));

    switch (robot.mode_state.recovery)
    {
    case RECOVERY_STATE_INIT:
        JointPidReset();
        YawTqPidReset();
        WheelPidsReset();
        stable_time = 0;
        height_time = 0;
        leg_extend_flag = true;
        robot.cmd.sup_cap_on = true;
        robot.mode_state.recovery = RECOVERY_STATE_RECOVERY;
        if (robot.cmd.recovery_tune)
        {
            leg_extend_flag = false;
        }
        if (robot.detect_states.abnormal)
        {
            robot.mode_state.recovery = RECOVERY_STATE_LOCK;
        }

        break;
    case RECOVERY_STATE_LOCK:
        if (fabsf(robot.leg_states[LEFT].curr.state.height - robot.leg_states[LEFT].ref.state.height) <
                kRecoveryHeightThres &&
            fabsf(robot.leg_states[RIGHT].curr.state.height - robot.leg_states[RIGHT].ref.state.height) <
                kRecoveryHeightThres &&
            fabsf(robot.leg_states[LEFT].curr.state.theta - robot.leg_states[LEFT].curr.state.phi) <
                kRecoveryThetaThres &&
            fabsf(robot.leg_states[RIGHT].curr.state.theta - robot.leg_states[RIGHT].curr.state.phi) <
                kRecoveryThetaThres &&
            fabsf(robot.chassis_motor_vel[LWM] * kWheelDiam) < kRecoveryDPosThres &&
            fabsf(robot.chassis_motor_vel[RWM] * kWheelDiam) < kRecoveryDPosThres)
        {
            robot.cmd.yaw_ang = robot.chassis_states.yaw_ang;
            lock_time++;
        }
        JointCtrlByPid(kHeightMin, PI / 2, 0);
        for (uint8_t i = 0; i < 2; i++)
        {
            // wheel_pids[i].calcPid(wheel_pids + i, 0, &robot.leg_states[i].curr.state.dpos, &tq);
            float ref = 0;
            float fdb = robot.chassis_motor_vel[W_INDEX(i)];
            Wheelpid_ptr[i]->calc(&ref, &fdb, nullptr, &tq);
            robot.chassis_motor_torques[W_INDEX(i)] += tq;
        }

        if (lock_time * kCtrlPeriod > kRecoveryLockTimeThres)
        {
            lock_time = 0;
            robot.mode_state.recovery = RECOVERY_STATE_RECOVERY;
            for (int i = 0; i < 2; i++)
            {
                Wheelpid_ptr[i]->reset();
            }
        }

        break;
    case RECOVERY_STATE_RECOVERY:
        /* retract legs to minimum height for a certain time */
        if (fabsf(robot.leg_states[LEFT].curr.state.height - robot.leg_states[LEFT].ref.state.height) <
                kRecoveryHeightThres &&
            fabsf(robot.leg_states[RIGHT].curr.state.height - robot.leg_states[RIGHT].ref.state.height) <
                kRecoveryHeightThres &&
            fabsf(robot.leg_states[LEFT].curr.state.theta - robot.leg_states[LEFT].curr.state.phi) <
                kRecoveryThetaThres &&
            fabsf(robot.leg_states[RIGHT].curr.state.theta - robot.leg_states[RIGHT].curr.state.phi) <
                kRecoveryThetaThres &&
            fabsf(robot.leg_states[LEFT].curr.state.dpos) < kRecoveryDPosThres &&
            fabsf(robot.leg_states[RIGHT].curr.state.dpos) < kRecoveryDPosThres)
        {
            robot.cmd.yaw_ang = robot.chassis_states.yaw_ang;
            height_time++;
        }

        if (height_time * kCtrlPeriod > kRecoveryHeightTimeThres)
        {
            height_time = 0;

            if (!leg_extend_flag && robot.cmd.recovery_tune) // 未伸长，先进入伸长模式
            {
                extend_ref[0] = D2R(-10.0f);
                extend_ref[1] = D2R(190.0f);
                if (robot.imu_datas.euler_vals[PITCH] > 0.0f)
                {
                    extend_ref[0] += D2R(120.0);
                }
                else
                {
                    extend_ref[1] -= D2R(120.0);
                }
                robot.mode_state.recovery = RECOVERY_STATE_LEG_EXTEND;
                for (int i = 0; i < 2; i++)
                {
                    WheelRecoverypid_ptr[i]->reset();
                }
            }
            else
            {
                robot.mode_state.recovery = RECOVERY_STATE_TUNE;
                for (int i = 0; i < 2; i++)
                {
                    Wheelpid_ptr[i]->reset();
                }
            }
        }
        if (leg_extend_flag && robot.cmd.recovery_tune) // 伸腿之后轮电机固定，腿收回将机器人前拉
        {
            for (uint8_t i = 0; i < 2; i++)
            {
                // wheel_pids[i].calcPid(wheel_pids + i, 0, &robot.leg_states[i].curr.state.dpos, &tq);
                float32_t fdb[2] = {robot.chassis_motors[W_INDEX(i)]->angle(), robot.chassis_motor_vel[W_INDEX(i)]};
                WheelRecoverypid_ptr[i]->calc(recover_wheel_ref[i], fdb, nullptr, &tq);
                robot.chassis_motor_torques[W_INDEX(i)] += tq;
            }
            JointCtrlByPid(kHeightMin, PI / 2, kBodyMass * kGravAcc / 2);
            // for (int i = 0; i < 4; i++)
            // {

            //     float32_t out = 0.0f;
            //     float fdb[2] = {robot.chassis_motors[i]->angle(), robot.chassis_motor_vel[i]};
            //     ref_ang[i] = LimDiff(extend_ref[i % 2], ref_ang[i], 1.0f);
            //     // ref = extend_ref[i % 2];
            //     jointpid_ptr[i]->calc(&ref_ang[i], fdb, nullptr, &out);
            //     robot.chassis_motor_torques[i] += out;
            // }
        }
        else
        {
            JointCtrlByPid(kHeightMin, PI / 2, kBodyMass * kGravAcc / 2);

            for (uint8_t i = 0; i < 2; i++)
            {
                // wheel_pids[i].calcPid(wheel_pids + i, 0, &robot.leg_states[i].curr.state.dpos, &tq);
                float ref = 0;
                float fdb = robot.chassis_motor_vel[W_INDEX(i)];
                Wheelpid_ptr[i]->calc(&ref, &fdb, nullptr, &tq);
                robot.chassis_motor_torques[W_INDEX(i)] += tq;
            }
            // for (uint8_t i = 0; i < 2; i++)
            // {
            //     // wheel_pids[i].calcPid(wheel_pids + i, 0, &robot.leg_states[i].curr.state.dpos, &tq);
            //     float32_t fdb[2] = {robot.chassis_motors[W_INDEX(i)]->angle(), robot.chassis_motor_vel[W_INDEX(i)]};
            //     WheelRecoverypid_ptr[i]->calc(recover_wheel_ref[i], fdb, nullptr, &tq);
            //     robot.chassis_motor_torques[W_INDEX(i)] += tq;
            // }
        }
        break;
    case RECOVERY_STATE_LEG_EXTEND:
        // 收腿之后先按倒地方向伸长，再收腿起身

        for (int i = 0; i < 4; i++)
        {

            float32_t out = 0.0f;
            float32_t ref = 0.0f;
            float fdb[2] = {robot.chassis_motors[i]->angle(), robot.chassis_motor_vel[i]};
            // ref = LimDiff(extend_ref[i % 2], robot.chassis_motors[i]->angle(), 1.0f);

            ref = extend_ref[i % 2];
            jointpid_ptr[i]->calc(&ref, fdb, nullptr, &out);
            robot.chassis_motor_torques[i] += out;
        }

        if (
            fabsf(robot.imu_datas.gyro_vals[PITCH]) < kRecoveryStableDPhiThres &&
            fabsf(robot.chassis_motor_vel[0]) < kRecoveryStableDPhiThres &&
            fabsf(robot.chassis_motor_vel[1]) < kRecoveryStableDPhiThres &&
            fabsf(robot.chassis_motor_vel[2]) < kRecoveryStableDPhiThres &&
            fabsf(robot.chassis_motor_vel[3]) < kRecoveryStableDPhiThres)
        {
            extend_time++;
        }
        else
        {
            extend_time /= 2;
        }
        if (extend_time * kCtrlPeriod > kRecoveryStableTimeThres)
        {
            extend_time = 0;
            robot.mode_state.recovery = RECOVERY_STATE_RECOVERY;
            leg_extend_flag = true;
            extend_ref[0] = D2R(-10.0f);
            extend_ref[1] = D2R(190.0f);
            for (int i = 0; i < 4; i++)
            {
                ref_ang[i] = robot.chassis_motors[i]->angle();
            }
        }
        break;
    case RECOVERY_STATE_TUNE:
        // if (robot.cmd.recovery_tune)
        // {
        // float fdb = (robot.leg_states[LEFT].curr.state.dpos + robot.leg_states[RIGHT].curr.state.dpos) / 2;

        // for (uint8_t i = 0; i < 2; i++)
        // {
        //     // wheel_pids[i].calcPid(wheel_pids + i, robot.cmd.dpos, &fdb, &tq);
        //     Wheelpid_ptr[i]->calc(&robot.cmd.dpos, &fdb, nullptr, &tq);
        //     robot.chassis_motor_torques[W_INDEX(i)] += tq;
        // }

        // JointCtrlByPid(kHeightMin, PI / 2, kBodyMass * kGravAcc);

        // SteerControl();

        // break;
        // }
        // else
        // {
        robot.mode_state.recovery = RECOVERY_STATE_STABLE;
        // }
    case RECOVERY_STATE_STABLE:
        for (uint8_t i = 0; i < 2; i++)
        {
            // wheel_pids[i].calcPid(wheel_pids + i, 0, &robot.leg_states[i].curr.state.dpos, &tq);
            float ref = 0;
            float fdb = robot.chassis_motor_vel[W_INDEX(i)];
            Wheelpid_ptr[i]->calc(&ref, &fdb, nullptr, &tq);
            robot.chassis_motor_torques[W_INDEX(i)] += tq;
        }

        if (stable_time * kCtrlPeriod > kRecoveryStableTimeThres)
        {
            stable_time = 0;
            robot.mode_state.recovery = RECOVERY_STATE_RETURN;
        }
        else if (
            fabsf(robot.imu_datas.gyro_vals[PITCH]) < kRecoveryStableDPhiThres &&
            fabsf(robot.leg_states[LEFT].curr.state.dpos) < kRecoveryStableDPosThres &&
            fabsf(robot.leg_states[RIGHT].curr.state.dpos) < kRecoveryStableDPosThres)
        {
            stable_time++;
        }
        else
        {
            stable_time /= 2;
        }
        break;
    case RECOVERY_STATE_RETURN:
        robot.cmd.height = kHeightMid;
        robot.cmd.height_f = kHeightMin;
        // robot.cmd.yaw_ang = robot.yaw_motor->angle();
        robot.cmd.yaw_ang = robot.chassis_states.yaw_ang;
        robot.leg_states[LEFT].ref.state.pos = robot.leg_states[RIGHT].ref.state.pos = 0;
        robot.leg_states[LEFT].curr.state.pos = robot.leg_states[RIGHT].curr.state.pos = 0;

        robot.chassis_mode.ref = robot.chassis_mode.curr = CHASSIS_MATURE;
        robot.chassis_mode.last = CHASSIS_RECOVERY;
        debug_num = 6;
        robot.steer_mode.ref = robot.steer_mode.curr = STEER_MOVE;
        // robot.steer_mode.ref = robot.steer_mode.curr = STEER_DEPART;
        robot.steer_mode.last = STEER_DEPART;
        robot.cmd.recovery_tune = false;
        break;
    default:
        break;
    }
}
int fly_times = 0;
/**
 * @brief       controller used at FLY mode
 * @retval      None
 * @note        None
 */
static void FlyController(void)
{
    float dtq = 0, fdb = 0;
    float front_tq = 0, behind_tq = 0, leg_push_force = 0, leg_tq[2] = {0, 0};
    float diff_pos;
    float air_dpos = 0;
    static float ground_height = 0;
    float leg_ang_ref = 0;
    static SteerMode_e last_steer_mode = STEER_DEPART;
    static float leg_cmd_height_air[2] = {0};
    static float last_height = kHeightMin;
    switch (robot.mode_state.fly)
    {
    case FLY_STATE_INIT:
        for (uint8_t i = 0; i < 2; i++)
        {
            leg_len_ctrls[i].iout = 0;
        }
        robot.cmd.air_height_diff = 0;
        diff_pos =
            (robot.leg_states[LEFT].ref.state.pos + robot.leg_states[RIGHT].ref.state.pos) / 2 -
            (robot.leg_states[LEFT].curr.state.pos + robot.leg_states[RIGHT].curr.state.pos) / 2;
        LIMIT_MAX(diff_pos, kAirPosMaxBias, -kAirPosMaxBias);
        // robot.leg_states[LEFT].ref.state.pos = robot.leg_states[RIGHT].ref.state.pos = robot.cmd.dpos * 0.55f;
        robot.leg_states[LEFT].ref.state.pos = robot.leg_states[RIGHT].ref.state.pos = diff_pos;
        robot.leg_states[LEFT].curr.state.pos = robot.leg_states[RIGHT].curr.state.pos = 0;
        air_dpos = (robot.leg_states[LEFT].curr.state.dpos + robot.leg_states[RIGHT].curr.state.dpos) / 2;
        robot.mode_state.fly = FLY_STATE_AIR;
        fly_times++;
        ground_height = robot.cmd.height_f;
        last_steer_mode = robot.steer_mode.last;
        for (int i = 0; i < 2; i++)
        {
            leg_cmd_height_air[i] = robot.leg_states[i].curr.state.height;
        }
    case FLY_STATE_AIR:
        if (robot.detect_states.on_ground[0] && robot.detect_states.on_ground[1])
        {
            robot.mode_state.fly = FLY_STATE_RETURN;
        }

        robot.cmd.height_f += kAirDH * kCtrlPeriod;
        LIMIT_MAX(robot.cmd.height_f, kAirHeightMin, kAirHeightMax);

        // robot.cmd.air_height_diff = LimDiff(sin(robot.imu_datas.euler_vals[ROLL]) * kWheelBase / 2.0f, robot.cmd.air_height_diff, kAirDH);
        robot.cmd.air_height_diff = 0.0f;
        LIMIT_MAX(robot.cmd.air_height_diff, kAirDiffHeight, -kAirDiffHeight);

        for (int i = 0; i < 2; i++)
        {
            if (robot.detect_states.on_ground[i])
            {
                leg_cmd_height_air[i] = LimDiff(kCmdHeightMin, leg_cmd_height_air[i], kMaxDH);
            }
            else
            {
                leg_cmd_height_air[i] = LimDiff(kAirHeightMax, leg_cmd_height_air[i], kAirDH);
            }
            robot.leg_states[i].ref.state.height = (leg_cmd_height_air[i] + robot.cmd.air_height_diff) / cos(robot.leg_states[i].curr.state.theta);
            LIMIT_MAX(robot.leg_states[i].ref.state.height, kAirHeightMax + kAirDiffHeight, kAirHeightMin);
        }

        // for(uint8_t i=0;i<2;i++){
        //     if(robot.detect_states.on_ground[i]){
        //         robot.leg_states[i].ref.state.height = LimDiff(ground_height/cos(robot.leg_states[i].curr.state.theta), robot.leg_states[i].ref.state.height, kMaxDH);
        //     }

        CalLqrMat();

        fdb = robot.leg_states[RIGHT].curr.state.theta - robot.leg_states[LEFT].curr.state.theta;

        leg_ang_pid_ptr->calc(&leg_ang_ref, &fdb, nullptr, &dtq);

        leg_tq[0] = -dtq;
        leg_tq[1] = dtq;
        /* only control theta and dtheta */
        // 悬空腿只控制腿的摆角
        for (uint8_t i = 0; i < 2; i++)
        {
            if (robot.detect_states.on_ground[i])
            {
                continue;
            }
            leg_tq[i] += mature_lqr[i][0][THETA] *
                             (robot.leg_states[i].ref.state.theta - robot.leg_states[i].curr.state.theta) +
                         mature_lqr[i][0][DOT_THETA] *
                             (robot.leg_states[i].ref.state.dtheta - robot.leg_states[i].curr.state.dtheta);

            // float32_t fdb = robot.chassis_motor_vel[W_INDEX(i)];
            // float32_t wheel_tor = 0;
            // float32_t ref = 0;
            // Wheelpid_ptr[i]->calc(&ref,&fdb,nullptr,&wheel_tor);
            robot.chassis_motor_torques[W_INDEX(i)] = 0;

            /* perform virtual leg length control */
            leg_push_force = LegLenCtrlCal(
                leg_len_ctrls + i, robot.leg_states[i].ref.state.height, robot.leg_states[i].curr.state.height,
                robot.leg_states[i].curr.state.dheight);
            float a = (-robot.leg_states[i].curr.state.height + kAirHeightMax) / (kAirHeightMax - kAirHeightMin);

            leg_push_force += (a * kBodyMass / 2.0f - (1 - a) * (kLegMass + kWheelMass)) * kGravAcc * arm_cos_f32(robot.leg_states[i].curr.state.theta);

            /* convert operation space to joint space */
            robot.five_rods_cal.opSp2JointSp(&robot.five_rods_cal, &front_tq, &behind_tq, leg_push_force, leg_tq[i],
                                             robot.chassis_motors[FJ_INDEX(i)]->angle(), robot.chassis_motors[BJ_INDEX(i)]->angle());
            robot.chassis_motor_torques[FJ_INDEX(i)] += front_tq;
            robot.chassis_motor_torques[BJ_INDEX(i)] += behind_tq;
        }
        // 正常腿计算LQR
        for (uint8_t i = 0; i < 2; i++)
        {
            if (!robot.detect_states.on_ground[i])
            {
                continue;
            }
            for (uint8_t j = 0; j < 6; j++)
            {
                leg_tq[i] += mature_lqr[i][0][j] *
                             (robot.leg_states[i].ref.data[j] - robot.leg_states[i].curr.data[j]) / 2;
                robot.chassis_motor_torques[W_INDEX(i)] +=
                    mature_lqr[i][1][j] * (robot.leg_states[i].ref.data[j] - robot.leg_states[i].curr.data[j]) / 2;
            }

            /* perform virtual leg length control */
            leg_push_force += LegLenCtrlCal(
                leg_len_ctrls + i, robot.leg_states[i].ref.state.height, robot.leg_states[i].curr.state.height,
                robot.leg_states[i].curr.state.dheight);
            if (i == 0)
            {
                test_force[2] = leg_push_force - test_force[0];
            }
            leg_push_force += kBodyMass * kGravAcc * cos(robot.leg_states[i].curr.state.theta) / 2;

            /* convert operation space to joint space */
            robot.five_rods_cal.opSp2JointSp(&robot.five_rods_cal, &front_tq, &behind_tq, leg_push_force, leg_tq[i],
                                             robot.chassis_motors[FJ_INDEX(i)]->angle(), robot.chassis_motors[BJ_INDEX(i)]->angle());
            robot.chassis_motor_torques[FJ_INDEX(i)] += front_tq;
            robot.chassis_motor_torques[BJ_INDEX(i)] += behind_tq;
        }
        break;
    case FLY_STATE_RETURN:
        // robot.cmd.height = (robot.leg_states[LEFT].ref.state.height+robot.leg_states[RIGHT].ref.state.height)/2;
        robot.cmd.height = ground_height;
        robot.chassis_mode.ref = robot.chassis_mode.curr = CHASSIS_MATURE;
        robot.chassis_mode.last = CHASSIS_FLY;
        debug_num = 7;
        robot.steer_mode.ref = robot.steer_mode.curr = last_steer_mode;
        robot.steer_mode.last = STEER_DEPART;
        robot.cmd.pitch_ang = robot.imu_datas.euler_vals[PITCH];
        break;
    default:
        break;
    }
}
/**
 * @brief       control two side joint motors by pid according to given params
 * @param       leg_len: virtual leg expected length
 * @param       leg_ang: virtual leg expected angle
 * @param       ffd_force: feedforward pushing force
 * @retval      None
 * @note        None
 */
static void JointCtrlByPid(float leg_len, float leg_ang, float ffd_force)
{
    float ref[4], ffd[4], out;

    /* get corresponding joint angles */
    robot.five_rods_cal.invKinematics(&robot.five_rods_cal, ref, ref + 1, leg_len, leg_ang);
    ref[2] = ref[0];
    ref[3] = ref[1];

    /* get joint feedforward torques */
    robot.five_rods_cal.opSp2JointSp(&robot.five_rods_cal, ffd, ffd + 1, ffd_force / 2, 0, ref[0], ref[1]);
    ffd[2] = ffd[0];
    ffd[3] = ffd[1];

    for (uint8_t i = 0; i < 4; i++)
    {
        float fdb[2] = {robot.chassis_motors[i]->angle(), robot.chassis_motor_vel[i]};
        // joint_pids[i].calcPid(joint_pids + i, ref[i], fdb, &out);
        jointpid_ptr[i]->calc(ref + i, fdb, nullptr, &out);
        robot.chassis_motor_torques[i] = ffd[i] + out;
    }
}
float rot_torque;
float rot_tq_fdb[2] = {0};
float fdb0 = 0;
hello_world::pid::MultiNodesPid::Datas rot_torque_pid_out[2];
bool slip_flag = false;
float yaw_torque_lim[2] = {0};
/**
 * @brief       control steer yaw angle
 * @retval      None
 * @note        perform LQR calculation before calling this function
 */
static void SteerControl(void)
{

    if (robot.steer_mode.last != robot.steer_mode.curr)
    {
        YawTqPidReset();
    }

    /* calculate torque range   */
    float torque_lim_l[2] = {robot.chassis_motors[LWM]->motor_info().torq_input_lim - robot.chassis_motor_torques[LWM],
                             -robot.chassis_motors[LWM]->motor_info().torq_input_lim - robot.chassis_motor_torques[LWM]};
    float torque_lim_r[2] = {robot.chassis_motors[RWM]->motor_info().torq_input_lim - robot.chassis_motor_torques[RWM],
                             -robot.chassis_motors[RWM]->motor_info().torq_input_lim - robot.chassis_motor_torques[RWM]};
    yaw_torque_lim[0] = MIN(torque_lim_r[0], -torque_lim_l[1]);

    yaw_torque_lim[1] = MIN(torque_lim_r[1], -torque_lim_l[0]);

    if (yaw_torque_lim[0] < yaw_torque_lim[1])
    {
        yaw_torque_lim[0] = yaw_torque_lim[1] = 0;
    }

    /* calculate additional torque to wheel motors according to yaw angle and angular velocity */

    // rot_torque = 0;
    // rot_tq_fdb[0]=robot.chassis_states.yaw_ang;
    // fdb0 = rot_tq_fdb[0];
    // // ANGLE_ACROSS0_HANDLE_RAD(fdb[0], robot.cmd.yaw_ang);
    // if(rot_tq_fdb[0]-robot.cmd.yaw_ang>PI){
    //     rot_tq_fdb[0]-=2*PI;
    // }
    // else if(rot_tq_fdb[0]-robot.cmd.yaw_ang<-PI){
    //     rot_tq_fdb[0]+=2*PI;
    // }

    // static float yaw_tor_err = robot.cmd.yaw_ang - rot_tq_fdb[0];
    // yaw_tq_pid_ptr->calc(&robot.cmd.yaw_ang, rot_tq_fdb, nullptr, &rot_torque);
    // rot_torque_pid_out[0]=yaw_tq_pid_ptr->getDatasAt(0);
    // rot_torque_pid_out[1]=yaw_tq_pid_ptr->getDatasAt(1);
    rot_torque = 0;
    // float fdb[2] = {robot.chassis_states.yaw_ang, robot.chassis_states.yaw_omg};

    rot_tq_fdb[0] = robot.chassis_states.yaw_ang;
    rot_tq_fdb[1] = robot.chassis_states.yaw_omg;
    ANGLE_ACROSS0_HANDLE_RAD(rot_tq_fdb[0], robot.cmd.yaw_ang);

    yaw_tq_pid_ptr->calc(&robot.cmd.yaw_ang, rot_tq_fdb, nullptr, &rot_torque);
    static float rot = rot_torque;
    rot = rot_torque;
    // rot_torque = hello_world::Bound((hello_world::Bound((robot.cmd.yaw_ang - rot_tq_fdb[0])*7.5,-10.0f,10.0f)-robot.chassis_states.yaw_omg)*3.5f,-8.0f,8.0f);

    // if(slip_flag&&!robot.detect_states.slip[0]&&!robot.detect_states.slip[1]){
    //     slip_flag=false;
    //     // robot.cmd.yaw_ang=robot.chassis_states.yaw_ang;
    // }
    // if(robot.steer_mode.curr!=STEER_GYRO&&slip_flag == false&&(robot.detect_states.slip[0]||robot.detect_states.slip[1])){
    //     slip_flag=true;
    //     rot_torque=0;
    // }

    // rot_torque_pid_out[0]=yaw_tq_pid_ptr->getDatasAt(0);
    // rot_torque_pid_out[1]=yaw_tq_pid_ptr->getDatasAt(1);

    LIMIT_MAX(rot_torque, yaw_torque_lim[0], yaw_torque_lim[1]);
    // rot_torque -=robot.yaw_motor->torq()/2;

    robot.chassis_motor_torques[LWM] -= rot_torque;
    robot.chassis_motor_torques[RWM] += rot_torque;

    float front_tq, behind_tq;
    float leg_tq[2] = {-rot_torque * (robot.leg_states[LEFT].curr.state.height) / (kWheelDiam / 2),
                       +rot_torque * (robot.leg_states[RIGHT].curr.state.height) / (kWheelDiam / 2)};
    // float leg_tq[2] = {0.0f,0.0f};
    // if(robot.steer_mode.curr == STEER_GYRO){
    // float32_t gyro_torque=0;
    // gyro_torque =robot.gyro_forward_speed*2.0f*cos(robot.gyro_forward_angle-robot.chassis_states.yaw_ang);
    // robot.chassis_motor_torques[LWM] += gyro_torque;
    // robot.chassis_motor_torques[RWM] += gyro_torque;
    // }
    for (uint8_t i = 0; i < 2; i++)
    {
        robot.five_rods_cal.opSp2JointSp(&robot.five_rods_cal, &front_tq, &behind_tq, 0, leg_tq[i],
                                         robot.chassis_motors[FJ_INDEX(i)]->angle(), robot.chassis_motors[BJ_INDEX(i)]->angle());
        robot.chassis_motor_torques[FJ_INDEX(i)] += front_tq;
        robot.chassis_motor_torques[BJ_INDEX(i)] += behind_tq;
    }
    // if(robot.detect_states.slip[0]){
    //     float32_t fdb= robot.chassis_motor_vel[LWM];
    //     float32_t slip_torque=0;
    //     Wheelpid_ptr[0]->calc(&robot.leg_states[0].curr.state.dpos, &fdb, nullptr, &slip_torque);
    //     robot.chassis_motor_torques[LWM] += slip_torque;
    // }else if(robot.detect_states.slip[1]){
    //     float32_t fdb= robot.chassis_motor_vel[RWM];
    //     float32_t slip_torque=0;
    //     Wheelpid_ptr[1]->calc(&robot.leg_states[1].curr.state.dpos, &fdb, nullptr, &slip_torque);
    //     robot.chassis_motor_torques[RWM] += slip_torque;

    // }
}

/**
 * @brief      跳台阶控制
 * @retval      None
 * @note        None
 */
static void JumpController(void)
{
    float dpos;
    float diff_pos;
    float dtq, fdb;
    float ref;
    float front_tq, behind_tq, leg_push_force, leg_tq[2];
    switch (robot.mode_state.jump)
    {
    case JUMP_STATE_INIT:
        for (uint8_t i = 0; i < 2; i++)
        {
            leg_len_ctrls[i].iout = 0;
        }

        JointPidReset();
        robot.mode_state.jump = JUMP_STATE_JUMP;
    case JUMP_STATE_JUMP:
        /* Make the joint motor output at the maximum torque until the leg reaches the maximum length */
        robot.chassis_motor_torques[LFM] = kJumpTq;
        robot.chassis_motor_torques[LBM] = -kJumpTq;
        robot.chassis_motor_torques[RFM] = kJumpTq;
        robot.chassis_motor_torques[RBM] = -kJumpTq;

        dpos = GetLimitedSpeed(robot.cmd.dpos);

        diff_pos =
            (robot.leg_states[LEFT].ref.state.pos + robot.leg_states[RIGHT].ref.state.pos) / 2 + dpos * kCtrlPeriod -
            (robot.leg_states[LEFT].curr.state.pos + robot.leg_states[RIGHT].curr.state.pos) / 2;
        if (fabsf(diff_pos) < kPosMaxBias)
        {
            robot.leg_states[LEFT].ref.state.pos += dpos * kCtrlPeriod;
            robot.leg_states[RIGHT].ref.state.pos += dpos * kCtrlPeriod;
        }

        CalLqrMat();

        for (uint8_t i = 0; i < 2; i++)
        {
            for (uint8_t j = 0; j < 6; j++)
            {
                robot.chassis_motor_torques[W_INDEX(i)] +=
                    mature_lqr[i][1][j] * (robot.leg_states[i].ref.data[j] - robot.leg_states[i].curr.data[j]) / 2;
            }
        }

        if (
            (robot.leg_states[LEFT].curr.state.height > kJumpHeightMax) ||
            (robot.leg_states[RIGHT].curr.state.height > kJumpHeightMax))
        {
            diff_pos =
                (robot.leg_states[LEFT].ref.state.pos + robot.leg_states[RIGHT].ref.state.pos) / 2 -
                (robot.leg_states[LEFT].curr.state.pos + robot.leg_states[RIGHT].curr.state.pos) / 2;
            LIMIT_MAX(diff_pos, kAirPosMaxBias, -kAirPosMaxBias);
            // robot.leg_states[LEFT].ref.state.pos = robot.leg_states[RIGHT].ref.state.pos = robot.cmd.dpos * 0.55f;
            robot.leg_states[LEFT].ref.state.pos = robot.leg_states[RIGHT].ref.state.pos = diff_pos;
            robot.leg_states[LEFT].curr.state.pos = robot.leg_states[RIGHT].curr.state.pos = 0;

            robot.mode_state.jump = JUMP_STATE_RECOVERY;
        }
        break;
    case JUMP_STATE_RECOVERY:
        JointCtrlByPid(kJumpHeightMin, PI / 2, -(kLegMass + kWheelMass) * kGravAcc);

        if (
            (fabsf(robot.leg_states[LEFT].curr.state.height - kJumpHeightMin) < kJumpHeightThres) ||
            (fabsf(robot.leg_states[RIGHT].curr.state.height - kJumpHeightMin) < kJumpHeightThres))
        {
            robot.cmd.height_f = kJumpHeightMin;

            robot.mode_state.jump = JUMP_STATE_AIR;
        }
        break;
    case JUMP_STATE_AIR:
        if (robot.detect_states.on_ground[0] && robot.detect_states.on_ground[1])
        {
            robot.mode_state.jump = JUMP_STATE_RETURN;
        }

        robot.cmd.height_f += kAirDH * kCtrlPeriod;
        LIMIT_MAX(robot.cmd.height_f, kAirHeightMin, kAirHeightMax);

        robot.leg_states[LEFT].ref.state.height = robot.cmd.height_f;
        robot.leg_states[RIGHT].ref.state.height = robot.cmd.height_f;
        LIMIT_MAX(robot.leg_states[LEFT].ref.state.height, kAirHeightMax, kAirHeightMin);
        LIMIT_MAX(robot.leg_states[RIGHT].ref.state.height, kAirHeightMax, kAirHeightMin);

        CalLqrMat();

        fdb = robot.leg_states[RIGHT].curr.state.theta - robot.leg_states[LEFT].curr.state.theta;
        ref = 0;
        leg_ang_pid_ptr->calc(&ref, &fdb, nullptr, &dtq);
        leg_tq[0] = -dtq;
        leg_tq[1] = dtq;

        /* only control theta and dtheta */
        for (uint8_t i = 0; i < 2; i++)
        {
            leg_tq[i] = mature_lqr[i][0][THETA] *
                            (robot.leg_states[i].ref.state.theta - robot.leg_states[i].curr.state.theta) +
                        mature_lqr[i][0][DOT_THETA] *
                            (robot.leg_states[i].ref.state.dtheta - robot.leg_states[i].curr.state.dtheta);
            robot.chassis_motor_torques[W_INDEX(i)] += 0;

            /* perform virtual leg length control */
            leg_push_force = LegLenCtrlCal(
                leg_len_ctrls + i, robot.leg_states[i].ref.state.height, robot.leg_states[i].curr.state.height,
                robot.leg_states[i].curr.state.dheight);
            leg_push_force += kBodyMass * kGravAcc * arm_cos_f32(robot.leg_states[i].curr.state.theta) / 2;

            /* convert operation space to joint space */
            robot.five_rods_cal.opSp2JointSp(&robot.five_rods_cal, &front_tq, &behind_tq, leg_push_force, leg_tq[i],
                                             robot.chassis_motors[FJ_INDEX(i)]->angle(), robot.chassis_motors[BJ_INDEX(i)]->angle());
            robot.chassis_motor_torques[FJ_INDEX(i)] += front_tq;
            robot.chassis_motor_torques[BJ_INDEX(i)] += behind_tq;
        }

        break;
    case JUMP_STATE_RETURN:
        robot.cmd.height = kHeightMid;

        robot.chassis_mode.ref = robot.chassis_mode.curr = CHASSIS_MATURE;
        robot.chassis_mode.last = CHASSIS_JUMP;
        debug_num = 8;
        robot.steer_mode.ref = robot.steer_mode.curr = STEER_MOVE;
        robot.steer_mode.last = STEER_DEPART;
        robot.cmd.pitch_ang = robot.imu_datas.euler_vals[PITCH];
        break;
    default:
        break;
    }
}

// /**
//  * @brief       controller used at FLY mode
//  * @retval      None
//  * @note        None
//  */
// static void FlyController(void)
// {
//     switch (robot.mode_state.fly) {
//         case FLY_STATE_INIT:
//             for (uint8_t i = 0; i < 2; i++) {
//                 leg_len_ctrls[i].iout = 0;
//             }

//             float diff_pos =
//                 (robot.leg_states[LEFT].ref.state.pos + robot.leg_states[RIGHT].ref.state.pos) / 2 -
//                 (robot.leg_states[LEFT].curr.state.pos + robot.leg_states[RIGHT].curr.state.pos) / 2;
//             LIMIT_MAX(diff_pos, kAirPosMaxBias, -kAirPosMaxBias);
//             // robot.leg_states[LEFT].ref.state.pos = robot.leg_states[RIGHT].ref.state.pos = robot.cmd.dpos * 0.55f;
//             robot.leg_states[LEFT].ref.state.pos = robot.leg_states[RIGHT].ref.state.pos = diff_pos;
//             robot.leg_states[LEFT].curr.state.pos = robot.leg_states[RIGHT].curr.state.pos = 0;

//             robot.mode_state.fly = FLY_STATE_AIR;
//         case FLY_STATE_AIR:
//             if (robot.detect_states.on_ground) {
//                 robot.mode_state.fly = FLY_STATE_RETURN;
//             }

//             robot.cmd.height_f += kAirDH * kCtrlPeriod;
//             LIMIT_MAX(robot.cmd.height_f, kAirHeightMin, kAirHeightMax);

//             robot.leg_states[LEFT].ref.state.height = robot.cmd.height_f;
//             robot.leg_states[RIGHT].ref.state.height = robot.cmd.height_f;
//             LIMIT_MAX(robot.leg_states[LEFT].ref.state.height, kAirHeightMax, kAirHeightMin);
//             LIMIT_MAX(robot.leg_states[RIGHT].ref.state.height, kAirHeightMax, kAirHeightMin);

//             CalLqrMat();

//             float dtq, fdb = robot.leg_states[RIGHT].curr.state.theta - robot.leg_states[LEFT].curr.state.theta;
//             leg_ang_pid_ptr->calc(nullptr, &fdb,nullptr, &dtq);
//             float front_tq, behind_tq, leg_push_force, leg_tq[2] = {-dtq, dtq};

//             /* only control theta and dtheta */
//             for (uint8_t i = 0; i < 2; i++) {
//                 leg_tq[i] = mature_lqr[i][0][THETA] *
//                                 (robot.leg_states[i].ref.state.theta - robot.leg_states[i].curr.state.theta) +
//                             mature_lqr[i][0][DOT_THETA] *
//                                 (robot.leg_states[i].ref.state.dtheta - robot.leg_states[i].curr.state.dtheta);
//                 robot.chassis_motor_torques[W_INDEX(i)] += 0;

//                 /* perform virtual leg length control */
//                 leg_push_force = LegLenCtrlCal(
//                     leg_len_ctrls + i, robot.leg_states[i].ref.state.height, robot.leg_states[i].curr.state.height,
//                     robot.leg_states[i].curr.state.dheight);
//                 leg_push_force += kBodyMass * kGravAcc * arm_cos_f32(robot.leg_states[i].curr.state.theta) / 2;

//                 /* convert operation space to joint space */
//                 robot.five_rods_cal.opSp2JointSp(&robot.five_rods_cal, &front_tq, &behind_tq, leg_push_force, leg_tq[i],
//                                                  robot.chassis_motors[FJ_INDEX(i)]->angle(), robot.chassis_motors[BJ_INDEX(i)]->angle());
//                 robot.chassis_motor_torques[FJ_INDEX(i)] += front_tq;
//                 robot.chassis_motor_torques[BJ_INDEX(i)] += behind_tq;
//             }
//             break;
//         case FLY_STATE_RETURN:
//             robot.cmd.height = kHeightMid;

//             robot.chassis_mode.ref = robot.chassis_mode.curr = CHASSIS_MATURE;
//             robot.chassis_mode.last = CHASSIS_FLY;
//             robot.steer_mode.ref = robot.steer_mode.curr = STEER_MOVE;
//             robot.steer_mode.last = STEER_DEPART;
//             robot.cmd.pitch_ang = robot.imu_datas.euler_vals[PITCH];
//             break;
//         default:
//             break;
//     }
// }

/**
 * @brief      yaw轴电机控制
 * @retval      None
 * @note        None
 */
void YawMotorContorller(void)
{
    // float fdb[2] = {robot.yaw_motor->angle(), robot.yaw_motor->vel()};
    // float ref = 0;
    // switch (robot.steer_mode.curr)
    // {
    // case STEER_GYRO:
    //     ref = -robot.chassis_states.yaw_ang + robot.gyro_forward_angle;
    //     break;

    // default:
    //     ref = 0;
    //     break;
    // }
    // float output = 0;
    // yaw_motor_pid_ptr->calc(&ref, fdb, nullptr, &output);
    // robot.yaw_motor_input = output;
}

/**
 * @brief       reset roll pid
 * @retval      None
 * @note        None
 */
inline static void RollPidReset(void)
{
    // roll_pid.resetData(&roll_pid);
    roll_pid_ptr->reset();
}
/**
 * @brief       reset yaw torque pid
 * @retval      None
 * @note        None
 */
inline static void YawTqPidReset(void)
{
    // yaw_tq_pid.resetData(&yaw_tq_pid);
    yaw_tq_pid_ptr->reset();
}

/**
 * @brief       reset joint pids
 * @arg         None
 * @note        None
 */
inline static void JointPidReset(void)
{
    for (uint8_t i = 0; i < 4; i++)
    {
        // joint_pids[i].resetData(joint_pids + i);
        jointpid_ptr[i]->reset();
    }
}

inline static void LegAngPidReset(void)
{
    leg_ang_pid_ptr->reset();
}

/**
 * @brief       calculate LQR control matrix
 * @retval      None
 * @note        set the expected leg length of the left and right sides before calling this function
 */
static void CalLqrMat(void)
{
    for (uint8_t i = 0; i < 2; i++)
    {
        for (uint8_t j = 0; j < 6; j++)
        {
            mature_lqr[LEFT][i][j] = PolyVal(kMatureLqrPoly[i][j], robot.leg_states[LEFT].curr.state.height, 3);
            mature_lqr[RIGHT][i][j] = PolyVal(kMatureLqrPoly[i][j], robot.leg_states[RIGHT].curr.state.height, 3);
        }
    }
}

inline static void ConfirmPidReset(void)
{
    for (int i = 0; i < 4; i++)
    {
        joint_confirm_pid_ptr[i]->reset();
    }
}

inline static void WheelPidsReset(void)
{
    for (int i = 0; i < 2; i++)
    {
        Wheelpid_ptr[i]->reset();
    }
}
// 关节滤波速度计算
void JointVelFilter(void)
{
    if (confirm_finish_flag == false)
    {
        for (int i = 0; i < 6; i++)
        {
            robot.chassis_motor_vel[i] = robot.chassis_motors[i]->vel();
        }
        return;
    }
    for (int i = 0; i < 6; i++)
    {
        // float32_t ang = robot.chassis_motors[i]->angle();
        // joint_motor_td[i]->calc(&ang, robot.chassis_motor_vel + i);
        robot.chassis_motor_vel[i] = robot.chassis_motors[i]->vel() * 0.9 + robot.chassis_motor_vel[i] * 0.1;
    }
}
/**
 * @brief       获取约束后的期望速度
 * @param       dpos_ref: 未进行约束的期望速度
 * @retval      约束后的期望速度
 * @note        None
 */
static float GetLimitedSpeed(float dpos_ref)
{
    if (!robot.cmd.speed_limit)
    {

        // float32_t dpos_lim = kMaxMovingSpeed;
        // float32_t dpos_curr = (robot.leg_states[LEFT].curr.state.dpos + robot.leg_states[RIGHT].curr.state.dpos) / 2;
        // dpos_ref = MIN(dpos_ref, dpos_lim - 0.7*(dpos_curr - dpos_lim));

        return dpos_ref;
    }

    float diff_dpos[2] = {100, -100};
    float dpos_lim[2];
    float joint_angs[2];

    /* 根据超电剩余量进行限速 */
    float sup_cap_dpos_lim = robot.sup_cap->get_remain_present() * kMaxLimSpeed;

    if (robot.cmd.auto_jump)
    {
        sup_cap_dpos_lim = robot.sup_cap->get_remain_present() * kAutoJumpSpeed;
    }

    LIMIT_MAX(dpos_ref, sup_cap_dpos_lim, -sup_cap_dpos_lim);

    return dpos_ref;

    if (dpos_ref > 0)
    {
        dpos_lim[0] = dpos_ref;
        dpos_lim[1] = 0;
    }
    else
    {
        dpos_lim[0] = 0;
        dpos_lim[1] = dpos_ref;
    }

    for (uint8_t i = 0; i < 2; i++)
    {
        joint_angs[0] = robot.chassis_motors[FJ_INDEX(i)]->angle();
        joint_angs[1] =
            robot.chassis_motors[BJ_INDEX(i)]->angle() > 0 ? robot.chassis_motors[BJ_INDEX(i)]->angle() : robot.chassis_motors[BJ_INDEX(i)]->angle() + 2 * PI;

        /* 根据与大腿与机械限位的角度差进行限速 */
        for (uint8_t j = 0; j < 2; j++)
        {
            diff_dpos[0] =
                MIN(diff_dpos[0], kSpeedDiffLimitSlope * fabsf(joint_angs[j] - kAbsMechanicalLimAng[j][0]));
            diff_dpos[1] =
                MAX(diff_dpos[1], -kSpeedDiffLimitSlope * fabsf(joint_angs[j] - kAbsMechanicalLimAng[j][1]));
        }
    }

    float dpos = (robot.leg_states[LEFT].curr.state.dpos + robot.leg_states[RIGHT].curr.state.dpos) / 2;
    LIMIT_MAX(dpos_ref, diff_dpos[0] + dpos, diff_dpos[1] + dpos);
    LIMIT_MAX(dpos_ref, dpos_lim[0], dpos_lim[1]);

    return dpos_ref;
}

float leg_force[2] = {0};
float debug_x[4] = {0};
// 计算因转向导致的两腿补偿力矩
float CalTurnTorDiff(void)
{
    float a, b, c, d;
    float mass_height = (robot.leg_states[LEFT].curr.state.height + robot.leg_states[LEFT].curr.state.height) / 2 + kBarycenterHeight;
    float theta = robot.imu_datas.euler_vals[ROLL];
    c = kGravAcc * kBodyMass;
    a = kWheelBase / (2 * cos(theta)) - mass_height * sin(theta);
    b = -kWheelBase / (2 * cos(theta)) - mass_height * sin(theta);
    d = -kBodyMass * robot.chassis_states.dpos * robot.imu_datas.gyro_vals[YAW];
    debug_x[0] = a;
    debug_x[1] = b;
    debug_x[2] = c;
    debug_x[3] = d;

    leg_force[0] = (d - b * c) / (a - b);
    leg_force[1] = (d - a * c) / (b - a);
    return (leg_force[0] - leg_force[1]) / 2;
}