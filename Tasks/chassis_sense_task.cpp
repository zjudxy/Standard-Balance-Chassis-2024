/**
 *******************************************************************************
 * @file      : chassis_sense_task.cpp
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
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

#include "imu.hpp"
#include "ahrs.hpp"
#include "tim.h"
#include "pid.hpp"
#include "arm_math.h"
#include "spi.h"
#include "chassis.hpp"
#include "td.hpp"
#include "filter.h"
#include "observer.hpp"
#include "tools.h"
#include "chassis_sense_params.hpp"
#include "chassis_cmd_task.hpp"
#include "chassis_params.hpp"
#include "chassis_sense_task.hpp"
#include "chassis_ctrl_params.hpp"
#include "iir.hpp"
namespace filter = hello_world::filter;
namespace observer = hello_world::observer;
static observer::Kalman *observer_ptr[2] = {nullptr};

// filter::Iir gyro_data_iir[3]={
//     filter::Iir(b_ls_20,a_ls_20,NL,DL),
//     filter::Iir(b_ls_20,a_ls_20,NL,DL),
//     filter::Iir(b_ls_20,a_ls_20,NL,DL)
// }; 
// filter::Iir acc_data_iir[3]={
//     filter::Iir(b_ls_20,a_ls_20,NL,DL),
//     filter::Iir(b_ls_20,a_ls_20,NL,DL),
//     filter::Iir(b_ls_20,a_ls_20,NL,DL)
// };

static Td_t dtheta_td[2], dheight_td[2], domg_err_td;
static Td_t ddtheta_td[2], ddphi_td[2], ddheight_td[2], dpos_td[2];
// filter::Td *ddp_td[2] = {nullptr};
float32_t body_dpos = 0.0f;
float StateSpaceModelU[2][2] = {0};
float StateSpaceModelA_L[2][6][6] = {0};
float StateSpaceModelB_L[2][6][2] = {0};

float ddpos[2] = {0, 0};

static void UpdataLegState(void);

static void EstiSupportForce(void);

static void GetSteerState(void);

static void GetIumData(void);

static void GetStateSpcaeModel(void);

static void CalBodyDpos_f(void);

void Td_init();

void gyro_time_offset_cal();



namespace imu = hello_world::imu;
static imu::BMI088 *imu_ptr = nullptr;

static float gyro_data[3], acc_data[3], temp;
static float gyro_data_f[3], acc_data_f[3];
uint32_t fail_count;
imu::BMI088ErrState err_fail;

namespace ahrs = hello_world::ahrs;

ahrs::Ahrs *ahrs_ptr = nullptr;
ahrs::Ahrs *ahrs_ptr_f = nullptr;

float Q[4] = {0.001, 0,
              0, 0.0001};
float R[4] = {0.9, 0,
              0, 0.9};
float P[4] = {1, 0,
              0, 1};

float H[4] = {1, 0,
              0, 1};
float F[4] = {1, kCtrlPeriod,
              0, 1};

float x0[2] = {0, 0};

/**
 * @brief      传感器初始化，包括IMU及其相关参数
 * @retval      None
 * @note        None
 */
void SenseInit(void)
{
    Td_init();
    imu::BMI088HWConfig hw_config = {
        .hspi = &hspi2,
        .acc_cs_port = GPIOC,
        .acc_cs_pin = GPIO_PIN_0,
        .gyro_cs_port = GPIOC,
        .gyro_cs_pin = GPIO_PIN_3,
    };
    const float rot_mat_flatten[9] = {0.0, 1.0, 0.0,
                                      1.0, 0.0, 0.0,
                                      0.0, 0.0, -1.0};
    // imu_ptr = new imu::BMI088(hw_config, rot_mat_flatten);

    // static imu::BMI088ErrState err_state;
    // do
    // {
    //     fail_count++;
    //     err_state = imu_ptr->init(true);
    // } while (err_state != imu::kBMI088ErrStateNoErr);
    // float samp_freq = 1000.0f, kp = 1.0f, ki = 0.0f;
    // ahrs_ptr = new ahrs::Mahony(samp_freq, kp, ki);

    imu_ptr = new imu::BMI088(hw_config, rot_mat_flatten);

    ahrs_ptr = new ahrs::Mahony(1000.0f, 1.0f, 0.0f);
    ahrs_ptr_f = new ahrs::Mahony(1000.0f, 1.0f, 0.0f);

    static imu::BMI088ErrState err_state;
    while (imu_ptr->init() != imu::kBMI088ErrStateNoErr)
    {
    }
    do
    {
        fail_count++;
        err_state = imu_ptr->init(true);
    } while (err_state != imu::kBMI088ErrStateNoErr);
    // ddp_td[0] = new filter::Td(100.0f, 0.001f);
    // ddp_td[1] = new filter::Td(100.0f, 0.001f);

    observer::Kalman::Config kf_config = {
        .x_dim = 2,
        .z_dim = 2,
        .u_dim = 0,

        .F = F,
        .B = nullptr,
        .H = H,
        .Q = Q,
        .R = R,
        .P = P,
        .x0 = x0};
    observer_ptr[LEFT] = new observer::Kalman(kf_config);
    observer_ptr[RIGHT] = new observer::Kalman(kf_config);
}

/**
 * @brief      传感器处理函数
 * @retval      None
 * @note        None
 */
void SenseTask(void)
{

    GetIumData();

    UpdataLegState();

    // CalBodyDpos_f();

    EstiSupportForce();

    GetSteerState();
    // if (robot.chassis_mode.curr == CHASSIS_MATURE)
    // {
    //     if (robot.detect_states.slip[LEFT])
    //     {
    //         robot.leg_states[LEFT].curr.state.dpos = robot.dpos_filter[LEFT];
    //     }
    //     if (robot.detect_states.slip[RIGHT])
    //     {
    //         robot.leg_states[RIGHT].curr.state.dpos = robot.dpos_filter[RIGHT];
    //     }
    // }
    // if (robot.chassis_mode.curr == CHASSIS_MATURE){
    // robot.leg_states[LEFT].curr.state.dpos = robot.dpos_filter[LEFT];
    // robot.leg_states[RIGHT].curr.state.dpos = robot.dpos_filter[RIGHT];
    // }
    float dpos = 0;
    if(robot.detect_states.on_ground[LEFT]&&robot.detect_states.on_ground[RIGHT]){
        dpos = (robot.leg_states[LEFT].curr.state.dpos + robot.leg_states[RIGHT].curr.state.dpos) / 2;
    }
    else if (robot.detect_states.on_ground[LEFT])
    {
        dpos = robot.leg_states[LEFT].curr.state.dpos;
    }
    else if (robot.detect_states.on_ground[RIGHT])
    {
        dpos = robot.leg_states[RIGHT].curr.state.dpos;
    }

    robot.leg_states[LEFT].curr.state.dpos = robot.leg_states[RIGHT].curr.state.dpos = dpos;
    robot.chassis_states.dpos = dpos;

    float pos = (robot.leg_states[LEFT].curr.state.pos + robot.leg_states[RIGHT].curr.state.pos) / 2;
    pos += (robot.leg_states[LEFT].curr.state.dpos + robot.leg_states[RIGHT].curr.state.dpos) *
           kCtrlPeriod / 2;
    robot.leg_states[LEFT].curr.state.pos = robot.leg_states[RIGHT].curr.state.pos = pos;

    // dpos = (robot.leg_states[LEFT].curr.state.dpos + robot.leg_states[RIGHT].curr.state.dpos) / 2;
    // // robot.leg_states[LEFT].curr.state.dpos = robot.leg_states[RIGHT].curr.state.dpos = dpos;
    // robot.chassis_states.dpos = dpos;

    // robot.leg_states[LEFT].curr.state.pos += robot.leg_states[LEFT].curr.state.dpos * kCtrlPeriod;
    // robot.leg_states[RIGHT].curr.state.pos += robot.leg_states[RIGHT].curr.state.dpos * kCtrlPeriod;

    gyro_time_offset_cal();

    GetStateSpcaeModel();
}


float32_t euler_angle_iir[3] = {0};

static void GetIumData(void)
{
    static float gyro_offet[3] = {0};
    static float gyro_yaw_last = 0.0f;
    static float acc_data_limit[3] = {0};
    static float acc_abs = 0.0f;
    imu_ptr->getData(acc_data, gyro_data, &temp);
    static bool acc_out_limit_flag = false;
    /* acc_data: 加速度数据 gyro_data: 角速度数据 */

    // gyro_data_iir[ROLL].calc(&gyro_data[ROLL], &gyro_data_f[ROLL]);
    // gyro_data_iir[PITCH].calc(&gyro_data[PITCH], &gyro_data_f[PITCH]);
    // gyro_data_iir[YAW].calc(&gyro_data[YAW], &gyro_data_f[YAW]);

    // acc_data_iir[X_AXIS].calc(&acc_data[X_AXIS], &acc_data_f[X_AXIS]);
    // acc_data_iir[Y_AXIS].calc(&acc_data[Y_AXIS], &acc_data_f[Y_AXIS]);
    // acc_data_iir[Z_AXIS].calc(&acc_data[Z_AXIS], &acc_data_f[Z_AXIS]);


    if (robot.control_tick < 1500 && robot.Init_finish_flag == false)
    {
        gyro_offet[ROLL] += gyro_data[ROLL] / 1500.0f;
        gyro_offet[PITCH] += gyro_data[PITCH] / 1500.0f;
        gyro_offet[YAW] += gyro_data[YAW] / 1500.0f;
    }
    else
    {
        robot.Init_finish_flag = true;
    }
    if (robot.Init_finish_flag == true)
    {
        gyro_data[ROLL] -= gyro_offet[ROLL];
        gyro_data[PITCH] -= gyro_offet[PITCH];
        gyro_data[YAW] -= gyro_offet[YAW];
        acc_abs = sqrt(acc_data[X_AXIS] * acc_data[X_AXIS] + acc_data[Y_AXIS] * acc_data[Y_AXIS] + acc_data[Z_AXIS] * acc_data[Z_AXIS]);
        if(fabs(acc_abs-kGravAcc)<1.0f){
            memcpy(acc_data_limit, acc_data, sizeof(acc_data));
            acc_out_limit_flag = false;
        }
        else{
            memset(acc_data_limit, 0, sizeof(acc_data));
            acc_out_limit_flag = true;
        }
        ahrs_ptr->update(acc_data_limit, gyro_data);
        // ahrs_ptr_f->update(acc_data_f, gyro_data_f);
    }
    float euler_angle[3]; // [roll pitch yaw]



    ahrs_ptr->getEulerAngle(robot.imu_datas.euler_vals);
    // ahrs_ptr_f->getEulerAngle(euler_angle_iir);

    robot.imu_datas.gyro_vals[ROLL] = gyro_data[ROLL] * 0.95 + robot.imu_datas.gyro_vals[ROLL] * 0.05;
    robot.imu_datas.gyro_vals[PITCH] = gyro_data[PITCH] * 0.95 + robot.imu_datas.gyro_vals[PITCH] * 0.05;
    robot.imu_datas.gyro_vals[YAW] = gyro_data[YAW] * 0.95 + robot.imu_datas.gyro_vals[YAW] * 0.05;

    robot.imu_datas.acc_vals[X_AXIS] = acc_data[X_AXIS];
    robot.imu_datas.acc_vals[Y_AXIS] = acc_data[Y_AXIS];
    robot.imu_datas.acc_vals[Z_AXIS] = acc_data[Z_AXIS];

    robot.imu_datas.yaw_ddphi = (robot.imu_datas.gyro_vals[YAW] - gyro_yaw_last) / kCtrlPeriod;
    gyro_yaw_last = robot.imu_datas.gyro_vals[YAW];
}

float leg_end_ddpos[2] = {0, 0};
float body_acc_x = 0.0f;
float dpos_origin[2] = {0, 0};
static void UpdataLegState(void)
{
    float leg_len, leg_ang, dpsi, alpha3, alpha4;
    static float leg_end_dpos[2] = {0, 0};
    float32_t leg_end_dpos_last[2] = {0, 0};
    static float dpos_last[2] = {0};
    // MotorStates_t states;
    float J[2][2];

    for (uint8_t i = 0; i < 2; i++)
    {
        dpos_last[i] = robot.leg_states[i].curr.state.dpos;

        robot.leg_states[i].curr.state.phi = robot.imu_datas.euler_vals[PITCH];
        robot.leg_states[i].curr.state.dphi = robot.imu_datas.gyro_vals[PITCH];

        robot.five_rods_cal.kinematicsMore(&robot.five_rods_cal, &leg_len, &leg_ang, &alpha3, &alpha4,
                                           robot.chassis_motors[FJ_INDEX(i)]->angle(), robot.chassis_motors[BJ_INDEX(i)]->angle());
        robot.five_rods_cal.calJacobian(
            &robot.five_rods_cal, J, robot.chassis_motors[FJ_INDEX(i)]->angle(), robot.chassis_motors[BJ_INDEX(i)]->angle());

        robot.leg_states[i].curr.state.height = leg_len;
        // robot.leg_states[i].curr.state.dheight =
        //     robot.chassis_motors[FJ_INDEX(i)]->vel() * J[0][0] + robot.chassis_motors[BJ_INDEX(i)]->vel() * J[0][1];

        // robot.leg_states[i].curr.state.theta = -PI / 2 + leg_ang + robot.leg_states[i].curr.state.phi;
        // dpsi = robot.chassis_motors[FJ_INDEX(i)]->vel() * J[1][0] + robot.chassis_motors[BJ_INDEX(i)]->vel() * J[1][1];
        robot.leg_states[i].curr.state.dheight =
            robot.chassis_motor_vel[FJ_INDEX(i)] * J[0][0] + robot.chassis_motor_vel[BJ_INDEX(i)] * J[0][1];

        robot.leg_states[i].curr.state.theta = -PI / 2 + leg_ang + robot.leg_states[i].curr.state.phi;
        dpsi = robot.chassis_motor_vel[FJ_INDEX(i)] * J[1][0] + robot.chassis_motor_vel[BJ_INDEX(i)] * J[1][1];
        robot.leg_states[i].curr.state.dtheta = dpsi + robot.leg_states[i].curr.state.dphi;

        float32_t dalpha, tmp_ang, sTmpAng, cTmpAng;

        if (kWheelEncoderSide == FRONT)
        {
            tmp_ang = robot.chassis_motors[FJ_INDEX(i)]->angle() - robot.leg_states[i].curr.state.theta +
                      robot.leg_states[i].curr.state.phi;
            arm_sin_cos_f32(R2D(tmp_ang), &sTmpAng, &cTmpAng);

            dalpha = (sTmpAng * robot.leg_states[i].curr.state.dheight +
                      cTmpAng * robot.leg_states[i].curr.state.height *
                          (robot.leg_states[i].curr.state.dphi - robot.leg_states[i].curr.state.dtheta)) /
                     (robot.five_rods_cal.leg_len * arm_sin_f32(robot.chassis_motors[FJ_INDEX(i)]->angle() - alpha3));
        }
        else
        {
            tmp_ang = robot.chassis_motors[BJ_INDEX(i)]->angle() - robot.leg_states[i].curr.state.theta +
                      robot.leg_states[i].curr.state.phi;
            arm_sin_cos_f32(R2D(tmp_ang), &sTmpAng, &cTmpAng);

            dalpha = (sTmpAng * robot.leg_states[i].curr.state.dheight +
                      cTmpAng * robot.leg_states[i].curr.state.height *
                          (robot.leg_states[i].curr.state.dphi - robot.leg_states[i].curr.state.dtheta)) /
                     (robot.five_rods_cal.leg_len * arm_sin_f32(robot.chassis_motors[BJ_INDEX(i)]->angle() - alpha4));
        }

        float32_t delta_omg = dalpha + robot.leg_states[i].curr.state.dphi;
        robot.leg_states[i].curr.state.dpos =
            (robot.chassis_motor_vel[W_INDEX(i)] + delta_omg) * (kWheelDiam / 2.0f);

        leg_end_dpos[i] = robot.leg_states[i].curr.state.dpos -
                          robot.leg_states[i].curr.state.dheight * sin(robot.leg_states[i].curr.state.theta) - robot.leg_states[i].curr.state.height * cos(robot.leg_states[i].curr.state.theta) * robot.leg_states[i].curr.state.dtheta - robot.leg_states[i].curr.state.dphi * cos(robot.leg_states[i].curr.state.phi) * kBarycenterHeight;
        // ddp_td[i]->calc(&leg_end_dpos[i],&leg_end_ddpos[i]);
        leg_end_ddpos[i] = (leg_end_dpos[i] - leg_end_dpos_last[i]) / kCtrlPeriod - robot.imu_datas.yaw_ddphi * kWheelBase / 2.0f * (i == 0 ? -1 : 1);
        leg_end_dpos_last[i] = leg_end_dpos[i];
        ddpos[i] = (robot.leg_states[i].curr.state.dpos - dpos_last[i]) / kCtrlPeriod;
        dpos_origin[i] = robot.leg_states[i].curr.state.dpos;
    }
    float acc_x = -robot.imu_datas.acc_vals[X_AXIS] * cos(robot.imu_datas.euler_vals[PITCH]) - robot.imu_datas.acc_vals[Z_AXIS] * sin(robot.imu_datas.euler_vals[PITCH]);
    float acc_z = robot.imu_datas.acc_vals[X_AXIS] * sin(robot.imu_datas.euler_vals[PITCH]) - robot.imu_datas.acc_vals[Z_AXIS] * cos(robot.imu_datas.euler_vals[PITCH])+kGravAcc;
    body_acc_x =  robot.imu_datas.acc_vals[X_AXIS] * cos(robot.imu_datas.euler_vals[PITCH]) + robot.imu_datas.acc_vals[Z_AXIS] * sin(robot.imu_datas.euler_vals[PITCH]);
    // arm_sqrt_f32(acc_x * acc_x + acc_z * acc_z,&body_acc_x);
    // body_acc_x *=SIGN(acc_x);
    robot.chassis_states.height = robot.leg_states[LEFT].curr.state.height + robot.leg_states[RIGHT].curr.state.height;
    // robot.yaw_motor.getStates(&robot.yaw_motor, &states);
    // robot.yaw_chassis_motors->angle() = states->angle();
    // robot.yaw_chassis_motors->vel() = Lpf1Order(0.9, states->vel(), robot.yaw_chassis_motors->vel());
    body_dpos = (leg_end_dpos[0] + leg_end_dpos[1]) / 2;
}

static void EstiSupportForce(void)
{
    float cTheta, sTheta, cPhi, sPhi;
    float ddz_b, ddz_w;
    float ddtheta, ddphi, ddheight;
    LegState_u leg_state;
    float leg_len, leg_ang, leg_push_force, leg_tq;

    float J[2][2], detA;

    for (uint8_t i = 0; i < 2; i++)
    {
        leg_state = robot.leg_states[i].curr;
        ddtheta = ddtheta_td[i].calcLnTd(ddtheta_td + i, leg_state.state.dtheta);
        ddphi = ddphi_td[i].calcLnTd(ddphi_td + i, leg_state.state.dphi);
        ddheight = ddheight_td[i].calcLnTd(ddheight_td + i, leg_state.state.dheight);

        arm_sin_cos_f32(R2D(leg_state.state.theta), &sTheta, &cTheta);
        arm_sin_cos_f32(R2D(leg_state.state.phi), &sPhi, &cPhi);

        ddz_b = robot.imu_datas.acc_vals[Z_AXIS] * cPhi - robot.imu_datas.acc_vals[X_AXIS] * sPhi - kGravAcc;
        ddz_w = ddz_b - ddheight * cTheta + 2 * leg_state.state.dheight * leg_state.state.dtheta * sTheta +
                leg_state.state.height * ddtheta * sTheta +
                leg_state.state.height * leg_state.state.dtheta * leg_state.state.dtheta * cTheta +
                kBarycenterHeight * ddphi * sPhi + kBarycenterHeight * leg_state.state.dphi * leg_state.state.dphi * cPhi;

        robot.five_rods_cal.kinematics(&robot.five_rods_cal, &leg_len, &leg_ang, robot.chassis_motors[FJ_INDEX(i)]->angle(),
                                       robot.chassis_motors[BJ_INDEX(i)]->angle());

        robot.five_rods_cal.jointSp2OpSp(&robot.five_rods_cal, &leg_push_force, &leg_tq,
                                         robot.chassis_motors[FJ_INDEX(i)]->torq(), robot.chassis_motors[BJ_INDEX(i)]->torq(),
                                         robot.chassis_motors[FJ_INDEX(i)]->angle(), robot.chassis_motors[BJ_INDEX(i)]->angle());

        robot.chassis_states.support_forces[i] = kWheelMass * (ddz_w + kGravAcc) + leg_push_force * cTheta - leg_tq * sTheta / leg_len;
    }
}

static void GetSteerState(void)
{
    if (robot.steer_mode.curr == STEER_DEPART || robot.steer_mode.curr == STEER_GYRO)
    {
        robot.chassis_states.yaw_ang = robot.imu_datas.euler_vals[YAW];
        robot.chassis_states.yaw_omg = robot.imu_datas.gyro_vals[YAW];
    }
    else
    {
        robot.chassis_states.yaw_ang = robot.yaw_motor->angle();
        robot.chassis_states.yaw_omg = robot.imu_datas.gyro_vals[YAW];
    }
    // robot.chassis_states.yaw_ang = robot.imu_datas.euler_vals[YAW];
    // robot.chassis_states.yaw_omg = robot.imu_datas.gyro_vals[YAW];
}

void Td_init()
{
    for (uint8_t i = 0; i < 2; i++)
    {
        // dtheta_td[i]=new filter::Td()
        TdInit(dtheta_td + i, kDThetaTdR, 0, kCtrlPeriod);
        TdInit(dheight_td + i, kDHeightTdR, 0, kCtrlPeriod);

        TdInit(ddtheta_td + i, kDDThetaTdR, 0, kCtrlPeriod);
        TdInit(ddphi_td + i, kDDPhiTdR, 0, kCtrlPeriod);
        TdInit(ddheight_td + i, kDDHeightTdR, 0, kCtrlPeriod);
        TdInit(dpos_td + i, kDPosTdR, 0, kCtrlPeriod);
    }

    TdInit(&domg_err_td, kDOmgErrTdR, 0, kCtrlPeriod);

    // PidInit(&imu_temp_pid, 1, &kImuTempPidParams);
}

int Across0(float32_t temp)
{
    if (abs(temp) < 0.01f)
    {
        return true;
    }
    return false;
}

static int cal_start_tick = 0;
static int Across0_ang_ref = 0;
static int Across0_ang_fdb = 0;
static bool ref_update_flag = 0;
static bool fdb_update_flag = 0;
static bool start_flag = false;
static int Across0_num = 0;
static float32_t time_total = 0.0f;
static float32_t time_offset = 0.0f;
float32_t angle_offset = -D2R(120.0f);
float ref_max_ang = 0.0f;
float32_t fdb_min_ang = 0.0f, fdb_max_ang = 0.0f;
float32_t fdb_min_vel = 0.0f, fdb_max_vel = 0.0f;
bool max_update_flag = false, min_update_flag = false;
int fdb_min_tick_curr = 0, fdb_max_tick_curr = 0;

void gyro_time_offset_cal()
{
    static float32_t maxdpos = 0.0f;
    static int maxdpos_tick = 0.0f;
    static float32_t maxdpos_ang = 0.0f;
    if (robot.steer_mode.curr != STEER_GYRO)
    {
        robot.cmd.gyro_cal_flag = false;
        ref_update_flag = fdb_update_flag = false;
        Across0_ang_ref = Across0_ang_fdb = 0;
        start_flag = false;
    }

    if (robot.cmd.gyro_cal_flag && !start_flag)
    {
        start_flag = true;
        ref_max_ang = robot.gyro_forward_angle;
        cal_start_tick = robot.control_tick;
        ref_update_flag = false;
        fdb_max_ang = robot.gyro_forward_angle;
        fdb_max_vel = body_dpos;
        fdb_max_tick_curr = robot.control_tick;
        fdb_min_tick_curr = robot.control_tick;
        max_update_flag = false;
        min_update_flag = true;
    }

    if (!start_flag)
        return;

    cal_start_tick++;
    if (!max_update_flag)
    {
        if (body_dpos > fdb_max_vel)
        {
            fdb_max_ang = robot.yaw_motor->angle();
            fdb_max_vel = body_dpos;
            fdb_max_tick_curr = robot.control_tick;
        }
        if (robot.control_tick - fdb_max_tick_curr > 150)
        {
            fdb_update_flag = true;
            max_update_flag = true;
            min_update_flag = false;
            fdb_min_vel = body_dpos;
            fdb_min_tick_curr = robot.control_tick;
        }
    }
    if (!min_update_flag)
    {
        if (body_dpos < fdb_min_vel)
        {
            fdb_min_ang = robot.yaw_motor->angle();
            fdb_min_vel = body_dpos;
            fdb_min_tick_curr = robot.control_tick;
        }
        else if (robot.control_tick - fdb_min_tick_curr > 150)
        {
            min_update_flag = true;
            max_update_flag = false;
            fdb_max_vel = body_dpos;
            fdb_max_tick_curr = robot.control_tick;
        }
    }

    if (fdb_update_flag)
    {
        ref_max_ang = robot.gyro_forward_angle;
        angle_offset += fdb_max_ang - ref_max_ang;
        ANGLE_ACROSS0_HANDLE_RAD(angle_offset, 0);
        fdb_update_flag = false;
    }

    // if(!ref_update_flag){
    //     if(robot.cmd.dpos>ref_max_val){
    //         ref_max_val=robot.cmd.dpos;
    //         ref_max_ang=robot.chassis_states.yaw_ang;
    //         ref_max_tick=robot.control_tick;
    //     }
    //     if(robot.control_tick-ref_max_tick>150){
    //         ref_update_flag=true;
    //         fdb_max_ang=robot.chassis_states.yaw_ang;
    //         fdb_max_val=robot.chassis_states.dpos;
    //     }
    // }
    // if (ref_update_flag)
    // {
    //     if (body_dpos > fdb_max_val)
    //     {
    //         fdb_max_ang = robot.chassis_states.yaw_ang;
    //         fdb_max_val = body_dpos;
    //         fdb_max_tick = robot.control_tick;
    //     }
    //     if (robot.control_tick - fdb_max_tick > 150)
    //     {
    //         ref_update_flag = false;
    //         fdb_update_flag = true;
    //         angle_offset += fdb_max_ang - ref_max_ang;
    //         ANGLE_ACROSS0_HANDLE_RAD(angle_offset, 0);
    //         fdb_max_tick = robot.control_tick;
    //     }
    // }

    // if (fdb_update_flag&&(robot.control_tick-fdb_max_tick>1000))
    // {
    //     float temp = robot.chassis_states.yaw_ang - ref_max_ang;
    //     ANGLE_ACROSS0_HANDLE_RAD(temp, 0);
    //     if (Across0(temp))
    //     {
    //         ref_update_flag = true;
    //         fdb_update_flag = false;
    //         ref_max_tick = robot.control_tick;
    //         fdb_max_tick = robot.control_tick;
    //         fdb_max_val = body_dpos;
    //         fdb_max_ang = robot.chassis_states.yaw_ang;

    //     }
    // }
    // if (Across0(robot.cmd.dpos))
    // {
    //     if (fdb_update_flag == false)
    //         return;
    //     Across0_ang_ref = robot.cmd.yaw_ang;
    //     ref_update_flag = true;
    //     fdb_update_flag = false;
    // }

    // if (Across0(robot.chassis_states.dpos))
    // {
    //     Across0_ang_fdb = robot.chassis_states.yaw_ang;
    //     fdb_update_flag = true;
    //     if (ref_update_flag)
    //     {
    //         Across0_num++;
    //         time_total += Across0_ang_fdb - Across0_ang_ref;
    //         time_offset = time_total / Across0_num;
    //         ref_update_flag == false;
    //     }
    // }
}

static void GetStateSpcaeModel(void)
{
    for (int i = 0; i < 2; i++)
    {
        float height = robot.leg_states[i].curr.state.height;
        for (int j = 0; j < 6; j++)
        {
            for (int k = 0; k < 6; k++)
            {
                StateSpaceModelA_L[i][j][k] = PolyVal(kStateSpaceModelPolyA_L[j][k], height, 3);
            }
        }
        for (int j = 0; j < 6; j++)
        {
            for (int k = 0; k < 2; k++)
            {
                StateSpaceModelB_L[i][j][k] = PolyVal(kStateSpaceModelPolyB_L[j][k], height, 3);
            }
        }
    }
}

// 下一刻状态预测
void StateEstimate(void)
{
    // X_estimate=X+AX*dt+BU*dt
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            robot.leg_states_estimate[i].data[j] = robot.leg_states[i].curr.data[j];
            for (int k = 0; k < 6; k++)
            {
                robot.leg_states_estimate[i].data[j] += StateSpaceModelA_L[i][j][k] * robot.leg_states[i].curr.data[j] * kCtrlPeriod;
            }
            for (int k = 0; k < 2; k++)
            {
                robot.leg_states_estimate[i].data[j] += StateSpaceModelB_L[i][j][k] * StateSpaceModelU[i][k] * kCtrlPeriod;
            }
        }
    }
}

static void CalBodyDpos_f(void)
{
    static float x_hat[2][2] = {0, 0, 0, 0};
    float z[2][2] = {{dpos_origin[LEFT], body_acc_x},
                     {dpos_origin[RIGHT], body_acc_x}};
    for (int i = 0; i < 2; i++)
    {
        observer_ptr[i]->calc(nullptr, z[i]);
        observer_ptr[i]->getX(x_hat[i]);
    }
    robot.dpos_filter[LEFT] = x_hat[LEFT][0];
    robot.dpos_filter[RIGHT] = x_hat[RIGHT][0];
}

static float InvSqrt(float x)
{
    if (x < 0)
    {
        return 0;
    }

    /* http://en.wikipedia.org/wiki/Fast_inverse_square_root */
    float x_2 = 0.5f * x;
    float y = x;
    int32_t i = *reinterpret_cast<int32_t *>(&y);
    i = 0x5f3759df - (i >> 1);
    y = *reinterpret_cast<float *>(&i);
    y = y * (1.5f - (x_2 * y * y));

    return y;
}