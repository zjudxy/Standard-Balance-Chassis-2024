/**
 *******************************************************************************
 * @file      : five_rods_calc.c
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention : See the R&D document for the detailed derivation process
 *******************************************************************************
 *  Copyright (c) 2023 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "five_rods_calc.h"

#include "system.h"
#include "tools.h"

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
enum Pos {  // number of axis
    X = 0,
    Z
};
enum Point {  // number of joint
    A = 0,
    B,
    C,
    D,
    E
};

/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

static void GetFiveRodState(
    const FiveRods_t *five_rods, float32_t pnt[5][2], float32_t alpha1, float32_t alpha2);

static void Kinematics(
    const FiveRods_t *five_rods, float32_t *leg_len, float32_t *leg_ang, float32_t front_ang, float32_t behind_ang);

static void KinematicsMore(
    const FiveRods_t *five_rods, float32_t *leg_len, float32_t *leg_ang, float32_t *alpha3, float32_t *alpha4, float32_t front_ang, float32_t behind_ang);

static void InvKinematics(
    const FiveRods_t *five_rods, float32_t *front_ang, float32_t *behind_ang, float32_t leg_len, float32_t leg_ang);

static void CalJacobian(const FiveRods_t *five_rods, float32_t J[2][2], float32_t front_ang, float32_t behind_ang);

static void JointSp2OpSp(
    const FiveRods_t *five_rods, float32_t *leg_push_force, float32_t *leg_tq, float32_t front_tq, float32_t behind_tq,
    float32_t front_ang, float32_t behind_ang);

static void OpSp2JointSp(
    const FiveRods_t *five_rods, float32_t *front_tq, float32_t *behind_tq, float32_t leg_push_force, float32_t leg_tq,
    float32_t front_ang, float32_t behind_ang);

/**
 * @brief       init the five rods mechanism with given param
 * @param       *five_rods: ptr to five rods mechanism
 * @param       joint_dis: distance between two joints with motors
 * @param       thigh_len: length of thigh
 * @param       leg_len: length of leg
 * @retval      None
 * @note        None
 */
void FiveRodsInit(FiveRods_t *five_rods, float32_t joint_dis, float32_t thigh_len, float32_t leg_len)
{
    ALG_ASSERT(five_rods);
    ALG_ASSERT(joint_dis > 0);
    ALG_ASSERT(thigh_len > 0);
    ALG_ASSERT(leg_len > 0);

    five_rods->joint_dis = joint_dis;
    five_rods->thigh_len = thigh_len;
    five_rods->leg_len = leg_len;

    five_rods->getFiveRodState = GetFiveRodState;
    five_rods->kinematics = Kinematics;
    five_rods->kinematicsMore = KinematicsMore;
    five_rods->invKinematics = InvKinematics;
    five_rods->calJacobian = CalJacobian;
    five_rods->jointSp2OpSp = JointSp2OpSp;
    five_rods->opSp2JointSp = OpSp2JointSp;
}

/**
 * @brief       calculate the coordinates of five points on the xOz plane
 * @param       *five_rods: ptr to five rods mechanism
 * @param       pnt: array of five points coordinates
 * @param       alpha1: unit: rad
 * @param       alpha2: unit: rad
 * @retval      None
 * @note        None
 */
static void GetFiveRodState(const FiveRods_t *five_rods, float32_t pnt[5][2], float32_t alpha1, float32_t alpha2)
{
    ALG_ASSERT(five_rods);
    ALG_ASSERT(pnt);

    float32_t sAlpha1, cAlpha1, sAlpha2, cAlpha2;
    arm_sin_cos_f32(R2D(alpha1), &sAlpha1, &cAlpha1);
    arm_sin_cos_f32(R2D(alpha2), &sAlpha2, &cAlpha2);

    pnt[A][X] = -five_rods->joint_dis / 2, pnt[A][Z] = 0;
    pnt[B][X] = five_rods->joint_dis / 2, pnt[B][Z] = 0;
    pnt[C][X] = -five_rods->joint_dis / 2 + five_rods->thigh_len * cAlpha2, pnt[C][Z] = -five_rods->thigh_len * sAlpha2;
    pnt[D][X] = five_rods->joint_dis / 2 + five_rods->thigh_len * cAlpha1, pnt[D][Z] = -five_rods->thigh_len * sAlpha1;

    float32_t pnt_F[2] = {(pnt[C][X] + pnt[D][X]) / 2, (pnt[C][Z] + pnt[D][Z]) / 2};

    float32_t CD_vec[2] = {pnt[D][X] - pnt[C][X], pnt[D][Z] - pnt[C][Z]};
    float32_t CD, EF;
    arm_sqrt_f32(CD_vec[X] * CD_vec[X] + CD_vec[Z] * CD_vec[Z], &CD);
    arm_sqrt_f32(five_rods->leg_len * five_rods->leg_len - CD * CD / 4, &EF);

    pnt[E][X] = EF * CD_vec[Z] / CD + pnt_F[X];
    pnt[E][Z] = -EF * CD_vec[X] / CD + pnt_F[Z];
}

/**
 * @brief       kinematics of five rods mechanism
 * @param       *five_rods: ptr to five rods mechanism
 * @param       *leg_len: corresponding virtual leg length, unit: m
 * @param       *leg_ang: corresponding virtual leg angle, unit: rad
 * @param       front_ang: corresponding angle of front joint, unit: rad
 * @param       behind_ang: corresponding angle of behind joint, unit: rad

 * @retval      None
 * @note        see the R&D document for the detailed derivation process
 */
static void Kinematics(
    const FiveRods_t *five_rods, float32_t *leg_len, float32_t *leg_ang, float32_t front_ang, float32_t behind_ang)
{
    ALG_ASSERT(five_rods);
    ALG_ASSERT(leg_len);
    ALG_ASSERT(leg_ang);

    float32_t pnt[5][2];
    GetFiveRodState(five_rods, pnt, front_ang, behind_ang);

    arm_sqrt_f32(pnt[E][X] * pnt[E][X] + pnt[E][Z] * pnt[E][Z], leg_len);
    arm_atan2_f32(-pnt[E][Z], pnt[E][X], leg_ang);
}

/**
 * @brief       kinematics of five rods mechanism with more result
 * @param       *five_rods: ptr to five rods mechanism
 * @param       *leg_len: corresponding virtual leg length, unit: m
 * @param       *leg_ang: corresponding virtual leg angle, unit: rad
 * @param       *alpha3: unit: rad
 * @param       *alpha4: unit: rad
 * @param       front_ang: corresponding angle of front joint, unit: rad
 * @param       behind_ang: corresponding angle of behind joint, unit: rad

 * @retval      None
 * @note        see the R&D document for the detailed derivation process
 */
static void KinematicsMore(
    const FiveRods_t *five_rods, float32_t *leg_len, float32_t *leg_ang, float32_t *alpha3, float32_t *alpha4, float32_t front_ang, float32_t behind_ang)
{
    ALG_ASSERT(five_rods);
    ALG_ASSERT(leg_len);
    ALG_ASSERT(leg_ang);

    float32_t pnt[5][2];
    GetFiveRodState(five_rods, pnt, front_ang, behind_ang);

    arm_sqrt_f32(pnt[E][X] * pnt[E][X] + pnt[E][Z] * pnt[E][Z], leg_len);
    arm_atan2_f32(-pnt[E][Z], pnt[E][X], leg_ang);

    float32_t DE_vec[2] = {pnt[E][X] - pnt[D][X],
                           pnt[E][Z] - pnt[D][Z]};
    float32_t CE_vec[2] = {pnt[E][X] - pnt[C][X],
                           pnt[E][Z] - pnt[C][Z]};

    arm_atan2_f32(-DE_vec[Z], DE_vec[X], alpha3);
    arm_atan2_f32(-CE_vec[Z], CE_vec[X], alpha4);
}

/**
 * @brief       inverse kinematics of five rods mechanism
 * @param       *five_rods: ptr to five rods mechanism
 * @param       *front_ang: corresponding angle of front joint, unit: rad
 * @param       *behind_ang: corresponding angle of behind joint, unit: rad
 * @param       leg_len: virtual leg length, unit: m
 * @param       leg_ang: virtual leg angle, unit: rad
 * @retval      None
 * @note        see the R&D document for the detailed derivation process
 */
static void InvKinematics(
    const FiveRods_t *five_rods, float32_t *front_ang, float32_t *behind_ang, float32_t leg_len, float32_t leg_ang)
{
    ALG_ASSERT(five_rods);
    ALG_ASSERT(front_ang);
    ALG_ASSERT(behind_ang);
    ALG_ASSERT(leg_len > 0);
    ALG_ASSERT(leg_ang >= 0 && leg_ang <= PI);

    float32_t sPsi, cPsi;
    arm_sin_cos_f32(R2D(leg_ang), &sPsi, &cPsi);
    float32_t AH = five_rods->joint_dis / 2 + leg_len * cPsi;
    float32_t BH = five_rods->joint_dis / 2 - leg_len * cPsi;
    float32_t EH = leg_len * sPsi;

    float32_t AE, BE;
    arm_sqrt_f32(EH * EH + AH * AH, &AE);
    arm_sqrt_f32(EH * EH + BH * BH, &BE);

    float32_t alpha22, alpha12;
    arm_atan2_f32(EH, BH, &alpha12);
    arm_atan2_f32(EH, AH, &alpha22);

    float32_t tmp;
    float32_t alpha11, alpha21;
    tmp = (five_rods->thigh_len * five_rods->thigh_len + BE * BE - five_rods->leg_len * five_rods->leg_len) /
          (2 * five_rods->thigh_len * BE);
    LIMIT_MAX(tmp, 1, -1);  // ensure the rationality of the solution
    alpha11 = acosf(tmp);
    tmp = (five_rods->thigh_len * five_rods->thigh_len + AE * AE - five_rods->leg_len * five_rods->leg_len) /
          (2 * five_rods->thigh_len * AE);
    LIMIT_MAX(tmp, 1, -1);  // ensure the rationality of the solution
    alpha21 = acosf(tmp);

    *front_ang = (PI - alpha11 - alpha12);
    *behind_ang = alpha21 + alpha22;

    /* normalize the angle to [-π, π) */
    ANGLE_RANGE_HANDLE_RAD(*front_ang);
    ANGLE_RANGE_HANDLE_RAD(*behind_ang);
}

/**
 * @brief       calculate Jacobian matrix
 * @param       *five_rods: ptr to five rods mechanism
 * @param       J: Jacobian matrix
 * @param       front_ang: angle of front joint, unit: rad
 * @param       behind_ang: angle of behind joint, unit: rad
 * @retval      None
 * @note        None
 */
static void CalJacobian(const FiveRods_t *five_rods, float32_t J[2][2], float32_t front_ang, float32_t behind_ang)
{
    ALG_ASSERT(five_rods);
    ALG_ASSERT(J);

    float32_t pnt[5][2];
    GetFiveRodState(five_rods, pnt, front_ang, behind_ang);

    float32_t DE_vec[2] = {pnt[E][X] - pnt[D][X],
                           pnt[E][Z] - pnt[D][Z]};
    float32_t CE_vec[2] = {pnt[E][X] - pnt[C][X],
                           pnt[E][Z] - pnt[C][Z]};

    float32_t psi, l, alpha3, alpha4;
    arm_sqrt_f32(pnt[E][X] * pnt[E][X] + pnt[E][Z] * pnt[E][Z], &l);
    arm_atan2_f32(-pnt[E][Z], pnt[E][X], &psi);
    arm_atan2_f32(-DE_vec[Z], DE_vec[X], &alpha3);
    arm_atan2_f32(-CE_vec[Z], CE_vec[X], &alpha4);

    float32_t sAlpha1_3 = arm_sin_f32(front_ang - alpha3), sAlpha2_4 = arm_sin_f32(behind_ang - alpha4);
    float32_t sAlpha3_4 = arm_sin_f32(alpha3 - alpha4);
    float32_t sPsi_alpha3, cPsi_alpha3, sPsi_alpha4, cPsi_alpha4;
    arm_sin_cos_f32(R2D(psi - alpha3), &sPsi_alpha3, &cPsi_alpha3);
    arm_sin_cos_f32(R2D(psi - alpha4), &sPsi_alpha4, &cPsi_alpha4);
    J[0][0] = -five_rods->thigh_len * sAlpha1_3 * sPsi_alpha4 / sAlpha3_4;
    J[0][1] = five_rods->thigh_len * sAlpha2_4 * sPsi_alpha3 / sAlpha3_4;
    J[1][0] = -five_rods->thigh_len * sAlpha1_3 * cPsi_alpha4 / (l * sAlpha3_4);
    J[1][1] = five_rods->thigh_len * sAlpha2_4 * cPsi_alpha3 / (l * sAlpha3_4);
}

/**
 * @brief       convert torques in joint space to force(moment) in operation space
 * @param       *five_rods: ptr to five rods mechanism
 * @param       *leg_push_force: corresponding pushing force of virtual leg, unit: N
 * @param       *leg_tq: corresponding torque of virtual leg, unit: N·m
 * @param       front_tq: torque of front joint, unit: N·m
 * @param       behind_tq: torque of behind joint, unit: N·m
 * @param       front_ang: angle of front joint, unit: rad
 * @param       behind_ang: angle of behind joint, unit: rad
 * @retval      None
 * @note        see the R&D document for the detailed derivation process
 */
static void JointSp2OpSp(
    const FiveRods_t *five_rods, float32_t *leg_push_force, float32_t *leg_tq, float32_t front_tq, float32_t behind_tq,
    float32_t front_ang, float32_t behind_ang)
{
    ALG_ASSERT(five_rods);
    ALG_ASSERT(leg_push_force);
    ALG_ASSERT(leg_tq);

    float32_t J[2][2];
    CalJacobian(five_rods, J, front_ang, behind_ang);
    float32_t det_J = J[0][0] * J[1][1] - J[0][1] * J[1][0];

    *leg_push_force = (front_tq * J[1][1] - behind_tq * J[1][0]) / det_J;
    *leg_tq = (-front_tq * J[0][1] + behind_tq * J[0][0]) / det_J;
}

/**
 * @brief       convert force(moment) in operation space to torques in joint space
 * @param       *five_rods: ptr to five rods mechanism
 * @param       *front_tq: corresponding torque of front joint, unit: N·m
 * @param       *behind_tq: corresponding torque of behind joint, unit: N·m
 * @param       leg_push_force: pushing force of virtual leg, unit: N
 * @param       leg_tq: torque of virtual leg, unit: N·m
 * @param       front_ang: angle of front joint, unit: rad
 * @param       behind_ang: angle of behind joint, unit: rad
 * @retval      None
 * @note        see the R&D document for the detailed derivation process
 */
static void OpSp2JointSp(
    const FiveRods_t *five_rods, float32_t *front_tq, float32_t *behind_tq, float32_t leg_push_force, float32_t leg_tq,
    float32_t front_ang, float32_t behind_ang)
{
    ALG_ASSERT(five_rods);
    ALG_ASSERT(front_ang);
    ALG_ASSERT(behind_ang);

    float32_t J[2][2];
    CalJacobian(five_rods, J, front_ang, behind_ang);
    *front_tq = J[0][0] * leg_push_force + J[1][0] * leg_tq;
    *behind_tq = J[0][1] * leg_push_force + J[1][1] * leg_tq;
}
