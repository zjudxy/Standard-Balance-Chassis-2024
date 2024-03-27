/**
 *******************************************************************************
 * @file      : filter.c
 * @brief     : filters
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.1      2022-12-03      dungloi         1. revision TD from Jiayuan
 *  V0.9.2      2023-01-09      dungloi         1. add kalman filter
 *  V0.9.3      2023-02-03      jiayuan         1. add IIR filter
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2022 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FILTER_H_
#define __FILTER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "arm_math.h"
#include "system.h"
#include "tools.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef struct Td Td_t;
typedef struct Kf Kf_t;
typedef struct Iir Iir_t;

struct Td {
    /* parameters */
    float h;   ///< step length
    float h0;  ///< Filter factor
    float r;   ///< fast speed factor
    /* data */
    float v;       ///< input
    float x1, x2;  ///< state variable
    float u;       ///< result of optimal synthesis function of fast control
    /* methods */
    /**
     * @brief       calculate linear TRACKING-DIFFERENTIATOR
     * @param       td: ptr to td
     * @param       val: refresh target value
     * @retval      x2, type: float
     * @note        None
     */
    float (*calcLnTd)(Td_t* td, float val);

    /**
     * @brief       calculate nonlinear TRACKING-DIFFERENTIATOR
     * @param       td: ptr to td
     * @param       val: refresh target value
     * @retval      x2, type: float
     * @note        None
     */
    float (*calcNlnTd)(Td_t* td, float val);
};

typedef enum {
    KF_MODEL_BASED,
    KF_SENSOR_FUSION,
} KfType_t;

struct Kf {
    KfType_t type;
    uint8_t xd;  ///< state dimension
    uint8_t zd;  ///< observation dimension

    arm_matrix_instance_f32 x_x1d;  ///< the state matrix
    arm_matrix_instance_f32 P_xxd;  ///< the state covariance matrix
    arm_matrix_instance_f32 F_xxd;  ///< the state-transition model matrix
    arm_matrix_instance_f32 Q_xxd;  ///< the covariance of the process noise matrix
    arm_matrix_instance_f32 H_zxd;  ///< the observation model matrix
    arm_matrix_instance_f32 R_zzd;  ///< the covariance of the observation noise matrix

    /**
     * @brief       predict x
     * @param       kf: ptr to a kf instance
     * @param       (optional va_arg, KF_SENSOR_FUSION only) x_measure: sensor measurement as prediction
     * @retval      None
     * @note        None
     */
    void (*predict)(Kf_t* kf, ...);

    /**
     * @brief       calculate optimal kalman gain, update the result and some matrix
     * @param       kf: ptr to a kf instance
     * @param       z: ptr to the array to contain z measurement
     * @retval      None
     * @note        None
     */
    void (*update)(Kf_t* kf, float* z);

    /**
     * @brief       get estimation result x
     * @param       kf: ptr to a kf instance
     * @param       x: ptr to array containing x estimation result
     * @retval      None
     * @note        None
     */
    void (*getX)(Kf_t* kf, float* x);
};

typedef struct{
    float* base;
    int head;
    int rear;
} IirQueue_t;

struct Iir{
    uint8_t order;  ///< filter order
    uint8_t n;  ///> number of variables filtered by this filter   
    const float* num;  ///< ptr to the numerator factor of the filter transform function, list in a float array
    const float* den;  ///< ptr to the denominator factor of the filter transform function, list in a float array
    IirQueue_t* xQlist;   ///< queue storing the past input
    IirQueue_t* yQlist;   ///< queue storing the past output 
    /* methods */
    /**
     * @brief       calculate IIR filter
     * @param       iir: ptr to an iir instance
     * @param       no: number of the filtered variable
     * @param       val: refresh target value
     * @retval      filtered value, type: float
     * @note        None
     */
    float (*calcIir)(Iir_t* iir, uint8_t no, float val);

    /**
     * @brief       calculate IIR filter
     * @param       iir: ptr to an iir instance
     * @param       n: number of the filtered variable
     * @param       vals: ptr to refresh target values
     * @param       out: ptr to store the updated data
     * @retval      none
     * @note        None
     */
    void (*calcMultIir)(Iir_t* iir, uint8_t n, float* vals, float* out);

    /**
     * @brief       delete IIR filter
     * @param       iir: ptr to an iir instance
     * @retval      None
     * @note        None
     */
    void (*delIir)(Iir_t* iir); 
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

/**
 * @brief       Init a tracking-differentiator
 * @param       td: ptr to a td instance
 * @param       r: fast speed factor
 * @param       h0: Filter factor
 * @param       h: step length
 * @retval      None
 * @note        None
 */
void TdInit(Td_t* td, float r, float h0, float h);

/**
 * @brief
 * @brief       Init a kalman filter
 * @param       kf: ptr to a kalman filter instance
 * @param       dimension: dimension of the state matrix
 * @param       measure_dimension: dimension of the measurement matrix
 * @param       type: type of filter
 *   @arg       KF_MODEL_BASED
 *   @arg       KF_SENSOR_FUSION
 * @param       x_init_x1d: init value of the state matrix, list in a float array, x * 1 dimension
 * @param       P_init_xxd: init value of state cov matrix, list in a float array, x * x dimension
 * @param       F_xxd: state transistion coef matrix, list in a float array, x * x dimension
 * @param       Q_xxd: covariance coef of the process noise matrix, list in a float array, x  * x dimension
 * @param       H_zxd: observation model coef matrix, list in a float array, z * x dimension
 * @param       R_zzd: covariance coef of the observation noise matrix, list in a float array, z * z dimension
 * @retval      None
 * @note        the larger Q is, trust measurement more; the larger R is, trust estimation more
 */
void KfInit(Kf_t* kf, uint8_t state_dimension, uint8_t measure_dimension, KfType_t type,
            float* x_init_x1d, float* P_init_xxd, float* F_xxd, float* Q_xxd, float* H_zxd, float* R_zzd);

/**
 * @brief       Init an IIR filter
 * @param       iir: ptr to a IIR instance
 * @param       order: filter order
 * @param       n: variable number 
 * @param       num[]: ptr to the numerator factor of the filter transform function, list in a float array, descending power
 * @param       den[]: ptr to the denominator factor of the filter transform function, list in a float array, descending power
 * @retval      None
 * @note                Y(z)    num[0]+num[1]*z^(-1)+num[2]*z^(-2)+...+num[order]*z^(-order)
 * @note        tf type ---- = --------------------------------------------------------------
 * @note                X(z)    den[0]+den[1]*z^(-1)+den[2]*z^(-2)+...+den[order]*z^(-order)
 */
void IirInit(Iir_t* iir, uint8_t n, uint8_t order, const float num[], const float den[]);

/**
 * @brief       1 order low pass filter
 * @param       coef: coefficient
 * @param       a: value a
 * @param       b: value b
 * @retval      result: float
 * @note        None
 */
static inline float Lpf1Order(float coef, float a, float b)
{
    if (!(coef <= 1 && coef >= 0))
        return 0;
    return coef * a + (1.0f - coef) * b;
}

#ifdef __cplusplus
}
#endif

#endif /* __FILTER_H_ */
