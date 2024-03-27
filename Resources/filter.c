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
/* Includes ------------------------------------------------------------------*/
#include "filter.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

static float CalcNlnTd(Td_t* td, float val);

static float CalcLnTd(Td_t* td, float val);

static float Fst(float x1, float x2, float r, float h0);

static void KfGetX(Kf_t* kf, float* x);

static void KfPredict(Kf_t* kf, ...);

static void KfUpdate(Kf_t* kf, float* z);

static float CalcIir(Iir_t* iir, uint8_t no, float val);

static void CalcMultIir(Iir_t* iir, uint8_t n, float* vals, float* out);

static void DelIir(Iir_t* iir);

/* ------------------------ tracking-differentiator ------------------------- */
/**
 * @brief       Init a tracking-differentiator
 * @param       td: ptr to td
 * @param       r: fast speed factor
 * @param       h0: Filter factor
 * @param       h: step length
 * @retval      None
 * @note        None
 */
void TdInit(Td_t* td, float r, float h0, float h)
{
    ALG_ASSERT(td);

    td->r = r;
    td->h0 = h0;
    td->h = h;

    td->calcNlnTd = CalcNlnTd;
    td->calcLnTd = CalcLnTd;
}

/**
 * @brief       calculate nonlinear tracking-differentiator
 * @param       td: ptr to td
 * @param       val: refresh target value
 * @retval      x2, type: float
 * @note        None
 */
static float CalcNlnTd(Td_t* td, float val)
{
    ALG_ASSERT(td);
    td->v = val;
    td->u = Fst(td->x1 - td->v, td->x2, td->r, td->h0);
    td->x1 += td->h * td->x2;
    td->x2 += td->h * td->u;
    return td->x2;
}

/**
 * @brief       calculate linear tracking-differentiator
 * @param       td: ptr to td
 * @param       val: refresh target value
 * @retval      x2, type: float
 * @note        None
 */
static float CalcLnTd(Td_t* td, float val)
{
    ALG_ASSERT(td);
    td->v = val;
    td->x1 = td->x1 + td->h * td->x2;
    td->x2 = td->x2 -
             td->h * (td->r * td->r * td->x1 + 2 * td->r * td->x2 - td->r * td->r * td->v);
    return td->x2;
}

/**
 * @brief       Optimal synthesis function of fast control
 * @param       x1: state variable
 * @param       x2: state variable
 * @param       r: fast speed factor
 * @param       h0: Filter factor
 * @retval      result, type: float
 * @note        refer to paper by Han Jingqing
 */
static float Fst(float x1, float x2, float r, float h0)
{
    float d, d0, y, a0, a;
    d = r * h0;
    d0 = d * h0;
    y = x1 + h0 * x2;
    a0 = sqrt(d * d + 8 * r * fabs(y));
    if (fabs(y) > d0)
        a = x2 + (a0 - d) / 2.0f * GetSign(y);
    else
        a = x2 + y / h0;
    if (fabs(a) <= d)
        return -(r * a / d);
    else
        return -r * (GetSign(a));
}

/* ------------------------------ Kalman Filter ------------------------------*/
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
            float* x_init_x1d, float* P_init_xxd, float* F_xxd, float* Q_xxd, float* H_zxd, float* R_zzd)
{
    ALG_ASSERT(kf);

    kf->xd = state_dimension;
    kf->zd = measure_dimension;
    kf->type = type;

    arm_mat_init_f32(&kf->x_x1d, kf->xd, 1, (float32_t*)x_init_x1d);
    arm_mat_init_f32(&kf->P_xxd, kf->xd, kf->xd, (float32_t*)P_init_xxd);
    arm_mat_init_f32(&kf->F_xxd, kf->xd, kf->xd, (float32_t*)F_xxd);
    arm_mat_init_f32(&kf->Q_xxd, kf->xd, kf->xd, (float32_t*)Q_xxd);
    arm_mat_init_f32(&kf->H_zxd, kf->zd, kf->xd, (float32_t*)H_zxd);
    arm_mat_init_f32(&kf->R_zzd, kf->zd, kf->zd, (float32_t*)R_zzd);

    kf->getX = KfGetX;
    kf->predict = KfPredict;
    kf->update = KfUpdate;
}

/**
 * @brief       predict x
 * @param       kf: ptr to a kf instance
 * @param       (optional va_arg, KF_SENSOR_FUSION only) x_measure: sensor measurement as prediction
 * @retval      None
 * @note        None
 */
static void KfPredict(Kf_t* kf, ...)
{
    ALG_ASSERT(kf);

    arm_matrix_instance_f32 Ft_xxd, tmp_xxd;
    float* Ft_xxd_data = (float*)MALLOC(kf->xd * kf->xd * sizeof(float));
    float* tmp_xxd_data = (float*)MALLOC(kf->xd * kf->xd * sizeof(float));
    arm_mat_init_f32(&Ft_xxd, kf->xd, kf->xd, (float32_t*)Ft_xxd_data);
    arm_mat_init_f32(&tmp_xxd, kf->xd, kf->xd, (float32_t*)tmp_xxd_data);

    switch (kf->type) {
        case KF_MODEL_BASED: {
            /* x' = F * x */
            arm_mat_mult_f32(&kf->F_xxd, &kf->x_x1d, &kf->x_x1d);

            /* P' = FPF^T + Q */
            arm_mat_trans_f32(&kf->F_xxd, &Ft_xxd);
            arm_mat_mult_f32(&kf->F_xxd, &kf->P_xxd, &tmp_xxd);
            arm_mat_mult_f32(&tmp_xxd, &Ft_xxd, &tmp_xxd);
            arm_mat_add_f32(&tmp_xxd, &kf->Q_xxd, &kf->P_xxd);

            break;
        }
        case KF_SENSOR_FUSION: {
            va_list valist;
            va_start(valist, kf);
            float* x_measure = va_arg(valist, float*);
            ALG_ASSERT(x_measure);
            va_end(valist);

            /* set sensor measurement as prediction */
            arm_mat_init_f32(&kf->x_x1d, kf->xd, 1, (float32_t*)x_measure);

            /* P' = FPF^T + Q */
            arm_mat_trans_f32(&kf->F_xxd, &Ft_xxd);
            arm_mat_mult_f32(&kf->F_xxd, &kf->P_xxd, &tmp_xxd);
            arm_mat_mult_f32(&tmp_xxd, &Ft_xxd, &tmp_xxd);
            arm_mat_add_f32(&tmp_xxd, &kf->Q_xxd, &kf->P_xxd);

            break;
        }
        default:
            break;
    }

    FREE(Ft_xxd_data);
    FREE(tmp_xxd_data);
}

/**
 * @brief       calculate optimal kalman gain, update the result and some matrix
 * @param       kf: ptr to a kf instance
 * @param       z: ptr to the array to contain z measurement
 * @retval      None
 * @note        None
 */
static void KfUpdate(Kf_t* kf, float* z)
{
    arm_matrix_instance_f32 z_z1d;
    arm_mat_init_f32(&z_z1d, kf->zd, 1, (float32_t*)z);

    arm_matrix_instance_f32 y_z1d, Ht_xzd, S_zzd, Sinv_zzd, K_xzd, I_xxd;
    arm_matrix_instance_f32 tmp_z1d, tmp_x1d, tmp_zxd, tmp_zzd, tmp_xzd, tmp_xxd;

    float* y_data = (float*)MALLOC(kf->zd * 1 * sizeof(float));
    float* Ht_data = (float*)MALLOC(kf->xd * kf->zd * sizeof(float));
    float* S_data = (float*)MALLOC(kf->zd * kf->zd * sizeof(float));
    float* Sinv_data = (float*)MALLOC(kf->zd * kf->zd * sizeof(float));
    float* K_data = (float*)MALLOC(kf->xd * kf->zd * sizeof(float));
    float* I_data = (float*)MALLOC(kf->xd * kf->xd * sizeof(float));

    for (uint8_t i = 0; i < kf->xd; i++) {
        for (uint8_t j = 0; j < kf->xd; j++) {
            if (i == j) {
                I_data[i * kf->xd + j] = 1;
            } else {
                I_data[i * kf->xd + j] = 0;
            }
        }
    }

    float* tmp_z1d_data = (float*)MALLOC(kf->zd * 1 * sizeof(float));
    float* tmp_x1d_data = (float*)MALLOC(kf->xd * 1 * sizeof(float));
    float* tmp_zxd_data = (float*)MALLOC(kf->zd * kf->xd * sizeof(float));
    float* tmp_zzd_data = (float*)MALLOC(kf->zd * kf->zd * sizeof(float));
    float* tmp_xzd_data = (float*)MALLOC(kf->xd * kf->zd * sizeof(float));
    float* tmp_xxd_data = (float*)MALLOC(kf->xd * kf->xd * sizeof(float));

    arm_mat_init_f32(&y_z1d, kf->zd, 1, (float32_t*)y_data);
    arm_mat_init_f32(&Ht_xzd, kf->xd, kf->zd, (float32_t*)Ht_data);
    arm_mat_init_f32(&S_zzd, kf->zd, kf->zd, (float32_t*)S_data);
    arm_mat_init_f32(&Sinv_zzd, kf->zd, kf->zd, (float32_t*)Sinv_data);
    arm_mat_init_f32(&K_xzd, kf->xd, kf->zd, (float32_t*)K_data);
    arm_mat_init_f32(&I_xxd, kf->xd, kf->xd, I_data);

    arm_mat_init_f32(&tmp_z1d, kf->zd, 1, (float32_t*)tmp_z1d_data);
    arm_mat_init_f32(&tmp_x1d, kf->xd, 1, (float32_t*)tmp_x1d_data);
    arm_mat_init_f32(&tmp_zxd, kf->zd, kf->xd, (float32_t*)tmp_zxd_data);
    arm_mat_init_f32(&tmp_zzd, kf->zd, kf->zd, (float32_t*)tmp_zzd_data);
    arm_mat_init_f32(&tmp_xzd, kf->xd, kf->zd, (float32_t*)tmp_xzd_data);
    arm_mat_init_f32(&tmp_xxd, kf->xd, kf->xd, tmp_xxd_data);

    /* S = HP'H^T + R */
    arm_mat_trans_f32(&kf->H_zxd, &Ht_xzd);
    arm_mat_mult_f32(&kf->H_zxd, &kf->P_xxd, &tmp_zxd);
    arm_mat_mult_f32(&tmp_zxd, &Ht_xzd, &tmp_zzd);
    arm_mat_add_f32(&tmp_zzd, &kf->R_zzd, &S_zzd);

    /* K = P' H^T S^-1 */
    arm_mat_inverse_f32(&S_zzd, &Sinv_zzd);
    arm_mat_mult_f32(&kf->P_xxd, &Ht_xzd, &tmp_xzd);
    arm_mat_mult_f32(&tmp_xzd, &Sinv_zzd, &K_xzd);

    /* y = z - Hx' */
    arm_mat_mult_f32(&kf->H_zxd, &kf->x_x1d, &tmp_z1d);
    arm_mat_sub_f32(&z_z1d, &tmp_z1d, &y_z1d);

    /* x = x' + Ky */
    arm_mat_mult_f32(&K_xzd, &y_z1d, &tmp_x1d);
    arm_mat_add_f32(&kf->x_x1d, &tmp_x1d, &kf->x_x1d);

    /* P = (I - KH) P' */
    arm_mat_mult_f32(&K_xzd, &kf->H_zxd, &tmp_xxd);
    arm_mat_sub_f32(&I_xxd, &tmp_xxd, &tmp_xxd);
    arm_mat_mult_f32(&tmp_xxd, &kf->P_xxd, &kf->P_xxd);

    FREE(y_data);
    FREE(Ht_data);
    FREE(S_data);
    FREE(Sinv_data);
    FREE(K_data);
    FREE(I_data);
    FREE(tmp_z1d_data);
    FREE(tmp_x1d_data);
    FREE(tmp_zxd_data);
    FREE(tmp_zzd_data);
    FREE(tmp_xzd_data);
    FREE(tmp_xxd_data);
}

/**
 * @brief       get estimation result x
 * @param       kf: ptr to a kf instance
 * @param       x: ptr to array containing x estimation result
 * @retval      None
 * @note        None
 */
static void KfGetX(Kf_t* kf, float* x)
{
    ALG_ASSERT(kf);
    ALG_ASSERT(x);
    memcpy(x, kf->x_x1d.pData, kf->xd * sizeof(float));
}

/* ------------------------ IIR Filter ------------------------- */
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
void IirInit(Iir_t* iir, uint8_t n, uint8_t order, const float num[], const float den[])
{
    ALG_ASSERT(iir);
    ALG_ASSERT(order);
    ALG_ASSERT(n);

    iir->order = order;
    iir->n = n;
    /* init queues */
    iir->xQlist = (IirQueue_t*)MALLOC(sizeof(IirQueue_t) * iir->n);
    iir->yQlist = (IirQueue_t*)MALLOC(sizeof(IirQueue_t) * iir->n);
    for (int i = 0; i < n; i++) {
        iir->xQlist[i].base = (float*)MALLOC(sizeof(float) * (iir->order + 1));
        iir->yQlist[i].base = (float*)MALLOC(sizeof(float) * (iir->order + 1));
        memset(iir->xQlist[i].base, 0, sizeof(float) * (iir->order + 1));
        memset(iir->yQlist[i].base, 0, sizeof(float) * (iir->order + 1));               
        ALG_ASSERT(iir->xQlist[i].base != NULL);
        ALG_ASSERT(iir->yQlist[i].base != NULL);
        iir->xQlist[i].head = 0;
        iir->xQlist[i].rear = iir->order;
        iir->yQlist[i].head = 0;
        iir->yQlist[i].rear = iir->order;
    }
    iir->num = num;
    iir->den = den;

    iir->calcIir = CalcIir;
    iir->calcMultIir = CalcMultIir;
    iir->delIir = DelIir;
}

/**
 * @brief       calculate IIR filter
 * @param       iir: ptr to an iir instance
 * @param       no: number of the filtered variable
 * @param       val: refresh target value
 * @retval      filtered value, type: float
 * @note        None
 */
static float CalcIir(Iir_t* iir, uint8_t no, float val)
{
    /* if iir has been deleted */
    if (iir->xQlist == NULL)
        return -0.000000114514;

    float temp = 0;
    int i;
    /* enQueue and deQueue */

    iir->xQlist[no - 1].head = (iir->xQlist[no - 1].head + 1) % (iir->order + 1);
    iir->xQlist[no - 1].rear = (iir->xQlist[no - 1].rear + 1) % (iir->order + 1);
    iir->xQlist[no - 1].base[iir->xQlist[no - 1].rear] = val;
    iir->yQlist[no - 1].head = (iir->yQlist[no - 1].head + 1) % (iir->order + 1);
    iir->yQlist[no - 1].rear = (iir->yQlist[no - 1].rear + 1) % (iir->order + 1);

    /* calculate filter */
    for (i = 0; i <= iir->order; i++) {
        temp += iir->xQlist[no - 1].base[(iir->xQlist[no - 1].head + i) % (iir->order + 1)] * iir->num[iir->order - i];
        if (i != iir->order)
            temp -= iir->yQlist[no - 1].base[(iir->yQlist[no - 1].head + i) % (iir->order + 1)] * iir->den[iir->order - i];
    }

    iir->yQlist[no - 1].base[iir->yQlist[no - 1].rear] = temp / iir->den[0];

    return iir->yQlist[no - 1].base[iir->yQlist[no - 1].rear];
}

/**
 * @brief       calculate IIR filter
 * @param       iir: ptr to an iir instance
 * @param       n: number of the filtered variable
 * @param       vals: ptr to refresh target values
 * @param       out: ptr to store the updated data
 * @retval      none
 * @note        None
 */
static void CalcMultIir(Iir_t* iir, uint8_t n, float* vals, float* out)
{
    for (int i = 1; i <= n; i++)
        out[i - 1] = iir->calcIir(iir, i, vals[i - 1]);
}

/**
 * @brief       delete IIR filter
 * @param       iir: ptr to an iir instance
 * @retval      None
 * @note        None
 */
static void DelIir(Iir_t* iir)
{
    for (int i = 0; i < iir->n; i++) {
        FREE(iir->xQlist[i].base);
        FREE(iir->yQlist[i].base);
        iir->xQlist[i].base = NULL;
        iir->yQlist[i].base = NULL;
    }
    FREE(iir->xQlist);
    FREE(iir->yQlist);
    iir->xQlist = NULL;
    iir->yQlist = NULL;
}
