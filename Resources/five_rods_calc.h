/**
 *******************************************************************************
 * @file      : five_rods_calc.h
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FIVE_RODS_CALC_H_
#define __FIVE_RODS_CALC_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "arm_math.h"

/* Exported macro ------------------------------------------------------------*/
#ifndef R2D
// radian to degree
#define R2D(x) ((x)*180 / PI)
#endif
#ifndef D2R
// degree to redian
#define D2R(x) ((x)*PI / 180)
#endif

/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

typedef struct _five_rodss_t FiveRods_t;
struct _five_rodss_t {
    float32_t joint_dis;  // distance between two joints with motors, unit: m
    float32_t thigh_len;  // length of thigh, unit: m
    float32_t leg_len;    // length of leg, unit: m

    /**
     * @brief       calculate the coordinates of five points on the xOz plane
     * @param       *five_rods: ptr to five rods mechanism
     * @param       pnt: array of five points coordinates
     * @param       alpha1: unit: rad
     * @param       alpha2: unit: rad
     * @retval      None
     * @note        None
     */
    void (*getFiveRodState)(const FiveRods_t *five_rods, float32_t pnt[5][2], float32_t alpha1, float32_t alpha2);

    /**
     * @brief       kinematics of five rods mechanism
     * @param       *five_rods: ptr to five rods mechanism
     * @param       *leg_len: corresponding virtual leg length, unit: m
     * @param       *leg_ang: corresponding virtual leg angle, unit: rad
     * @param       front_ang: angle of front joint, unit: rad
     * @param       behind_ang: angle of behind joint, unit: rad
     * @retval      None
     * @note        see the R&D document for the detailed derivation process
     */
    void (*kinematics)(
        const FiveRods_t *five_rods, float32_t *leg_len, float32_t *leg_ang, float32_t front_ang, float32_t behind_ang);

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
    void (*kinematicsMore)(
        const FiveRods_t *five_rods, float32_t *leg_len, float32_t *leg_ang, float32_t *alpha3, float32_t* alpha4, float32_t front_ang, float32_t behind_ang);

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
    void (*invKinematics)(
        const FiveRods_t *five_rods, float32_t *front_ang, float32_t *behind_ang, float32_t leg_len, float32_t leg_ang);

    /**
     * @brief       calculate Jacobian matrix
     * @param       *five_rods: ptr to five rods mechanism
     * @param       J: Jacobian matrix
     * @param       front_ang: angle of front joint, unit: rad
     * @param       behind_ang: angle of behind joint, unit: rad
     * @retval      None
     * @note        None
     */
    void (*calJacobian)(const FiveRods_t *five_rods, float32_t J[2][2], float32_t front_ang, float32_t behind_ang);

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
    void (*jointSp2OpSp)(
        const FiveRods_t *five_rods, float32_t *leg_push_force, float32_t *leg_tq, float32_t front_tq, float32_t behind_tq,
        float32_t front_ang, float32_t behind_ang);

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
    void (*opSp2JointSp)(
        const FiveRods_t *five_rods, float32_t *front_tq, float32_t *behind_tq, float32_t leg_push_force, float32_t leg_tq,
        float32_t front_ang, float32_t behind_ang);
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

/**
 * @brief       init the five rods mechanism with given param
 * @param       *five_rods: ptr to five rods mechanism
 * @param       joint_dis: distance between two joints with motors
 * @param       thigh_len: length of thigh
 * @param       leg_len: length of leg
 * @retval      None
 * @note        None
 */
void FiveRodsInit(FiveRods_t *five_rods, float32_t joint_dis, float32_t thigh_len, float32_t leg_len);

#ifdef __cplusplus
}
#endif

#endif /* __FIVE_RODS_CALC_H_ */
