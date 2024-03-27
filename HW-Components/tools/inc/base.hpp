/**
 *******************************************************************************
 * @file      base.hpp
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
#ifndef HW_COMPONENTS_TOOLS_BASE_HPP_
#define HW_COMPONENTS_TOOLS_BASE_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cmath>
#include <cstdint>

#include "allocator.hpp"
#include "assert.hpp"
namespace hello_world
{
/* Exported macro ------------------------------------------------------------*/
#ifndef PI
#define PI 3.14159265358979f
#endif
/* Exported constants --------------------------------------------------------*/

static const float kGravAcc = 9.7936f;  ///* 重力加速度（杭州）
static const float kRad2DegCoff = 180 / PI;
static const float kDeg2RadCoff = PI / 180;
/* Exported types ------------------------------------------------------------*/

/* 周期数据转连续数据 */
class PeriodData2ContData : public MemMang
{
 public:
  /**
   * @brief       周期数据转连续数据初始化
   * @param        period: 周期
   * @param        init_data: 初始值
   * @retval       None
   * @note        None
   */
  explicit PeriodData2ContData(float period, float init_data = 0)
      : kPeriod_(period), last_data_(init_data)
  {
    /* 变量检查 */
#pragma region
    HW_ASSERT(period > 0, "period <= 0");
#pragma endregion
  };

  ~PeriodData2ContData() = default;

  /**
   * @brief       周期数据转连续数据
   * @param        data: 待转换的周期数据
   * @retval       转换后的连续数据
   * @note        None
   */
  float operator()(float data)
  {
    float delta_k = roundf((data - last_data_) / kPeriod_);
    last_data_ = data - delta_k * kPeriod_;
    return last_data_;
  }

 private:
  const float kPeriod_;  ///< 周期
  float last_data_;      ///< 上一次的数据
};

/* 周期角度转连续角度，单位：rad */
class PeriodAngle2ContAngleRad : public PeriodData2ContData
{
 public:
  /**
   * @brief       周期角度转连续角度初始化
   * @param        period: 周期
   * @param        init_angle: 初始角度
   * @retval       None
   * @note        None
   */
  explicit PeriodAngle2ContAngleRad(float init_angle = 0)
      : PeriodData2ContData(2 * PI, init_angle){};

  ~PeriodAngle2ContAngleRad() = default;
};

/* 周期角度转连续角度，单位：deg */
class PeriodAngle2ContAngleDeg : public PeriodData2ContData
{
 public:
  /**
   * @brief       周期角度转连续角度初始化
   * @param        period: 周期
   * @param        init_angle: 初始角度
   * @retval       None
   * @note        None
   */
  explicit PeriodAngle2ContAngleDeg(float init_angle = 0)
      : PeriodData2ContData(360, init_angle){};

  ~PeriodAngle2ContAngleDeg() = default;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

/**
 * @brief       获取变量的符号
 * @param        x: 输入变量
 * @retval       输入变量的符号
 * @note        只有支持大小比较的变量才可使用该函数
 */
template <typename T>
inline uint8_t GetSign(T x)
{
  if (x > 0) {
    return 1;
  } else if (x < 0) {
    return -1;
  } else {
    return 0;
  }
}

/**
 * @brief       获取两个变量间的最大值
 * @param        x1: 比较变量1
 * @param        x2: 比较变量1
 * @retval       二者间的最大值
 * @note        只有支持大小比较的变量才可使用该函数
 */
template <typename T>
inline T Max(T x1, T x2)
{
  return x1 > x2 ? x1 : x2;
}

/**
 * @brief       获取两个变量间的最小值
 * @param        x1: 比较变量1
 * @param        x2: 比较变量1
 * @retval       二者间的最小值
 * @note        只有支持大小比较的变量才可使用该函数
 */
template <typename T>
inline T Min(T x1, T x2)
{
  return x1 < x2 ? x1 : x2;
}

/**
 * @brief       将输入值限制在一个范围内
 * @param        x: 待范围限制变量
 * @param        lim1: 范围边界1
 * @param        lim2: 范围边界2
 * @retval       范围限制后的值
 * @note        无需考虑两个范围边界的相对大小关系，变量需要支持大小比较
 */
template <typename T1, typename T2, typename T3>
inline T1 Bound(T1 x, T2 lim1, T3 lim2)
{
  float max_lim, min_lim;
  if (lim1 >= lim2) {
    max_lim = lim1;
    min_lim = lim2;
  } else {
    max_lim = lim2;
    min_lim = lim1;
  }

  if (x > max_lim) {
    return max_lim;
  } else if (x < min_lim) {
    return min_lim;
  } else {
    return x;
  }
}

/**
 * @brief       判断变量是否位于给定的范围内
 * @param        bound1: 范围边界1（含）
 * @param        bound2: 范围边界2（含）
 * @retval       变量是否位于给定的范围内
 * @note        无需考虑两个范围边界的相对大小关系，变量需要支持大小比较
 */
template <typename T>
inline bool IsInRange(T x, T bound1, T bound2)
{
  if (bound1 <= bound2) {
    return x >= bound1 && x <= bound2;
  } else {
    return x <= bound1 && x >= bound2;
  }
}

/**
 * @brief       将周期性数据归一化到指定周期区间
 * @param        period_lb: 周期区间下界（含）
 * @param        period_ub: 周期区间上界
 * @param        data: 待归一化数据
 * @retval       归一化后的数据
 * @note        period_lb 必须小于 period_ub
 */
inline float NormPeriodData(float period_lb, float period_ub, float data)
{
  HW_ASSERT(period_ub > period_lb, "lower bound is larger than upper bound");

  float period = period_ub - period_lb;
  float tmp = fmodf(data - period_lb, period);  // (-period, period)

  if (tmp < 0) {
    return tmp + period + period_lb;
  } else {
    return tmp + period_lb;
  }
}

/**
 * @brief       将待处理数据平移到参考数据的所在周期中
 * @param        period_lb: 周期区间下界（含）
 * @param        period_ub: 周期区间上界
 * @param        ref_data: 参考数据（无需再周期区间之内）
 * @param        data: 待处理数据
 * @retval       平移后的数据
 * @note        period_lb 必须小于 period_ub
 */
inline float PeriodData2SameRegion(float period_lb, float period_ub,
                                   float ref_data, float data)
{
  HW_ASSERT(period_ub > period_lb, "lower bound is larger than upper bound");

  float period = period_ub - period_lb;

  float delta_k = floorf((ref_data - period_lb) / period) -
                  floorf((data - period_lb) / period);
  return data + delta_k * period;
}

/**
 * @brief       将待处理数据平移到距离参考数据最近的值
 * @param        period: 周期
 * @param        ref_data: 参考数据
 * @param        data: 待处理数据
 * @retval       平移后的数据
 * @note        None
 */
inline float PeriodData2NearestDist(float period, float ref_data, float data)
{
  HW_ASSERT(period > 0, "period <= 0");

  float delta_k = roundf((ref_data - data) / period);
  return data + delta_k * period;
}

/**
 * @brief       将角度归一化到[-π, π)
 * @param        angle: 待归一化角度，单位：rad
 * @retval       归一化后的角度，单位：rad
 * @note        None
 */
inline float AngleNormRad(float angle)
{
  return NormPeriodData(-PI, PI, angle);
}

/**
 * @brief       将角度归一化到[-180°, 180°)
 * @param        angle: 待归一化角度，单位：deg
 * @retval       归一化后的角度，单位：deg
 * @note        None
 */
inline float AngleNormDeg(float angle)
{
  return NormPeriodData(-180, 180, angle);
}

/**
 * @brief 处理周期性数据的差值
 * 
 * @param minuend 被减数
 * @param subtrahend 减数
 * @param period 周期
 * @return 返回处理后的差值，差值的范围在 -period/2 到 period/2 之间
 * 
 * @details 该函数用于处理周期性数据的差值，例如角度值。首先计算 minuend 和 subtrahend 的差值，
 * 然后将差值除以周期 period 并四舍五入得到倍数 times，最后返回差值减去 times 乘以 period 的结果。
 */
inline float PeriodDataSub(float minuend, float subtrahend, float period)
{
  HW_ASSERT(period > 0, "val must be larger than 0");
  period = period < 0 ? -period : period;

  float diff = minuend - subtrahend;
  float times = roundf(diff / period);
  float res = diff - times * period;

  return res;
};

/**
 * @brief 处理角度跨越 0 的情况，单位为弧度
 * 
 * @param minuend 被减数，也就是目标角度
 * @param subtrahend 减数，也就是实际反馈的角度值
 * @return 返回处理后的角度值，角度值的范围在 -PI 到 PI 之间
 * 
 * @details 该函数用于处理角度跨越 0 的情况，单位为弧度。首先调用 PeriodDataSub 函数处理 minuend 和 subtrahend 的差值，
 * 然后将处理后的差值加到 subtrahend 上，返回结果。
 */
inline float HandleAngleCross0Rad(float minuend, float subtrahend)
{
  return subtrahend + PeriodDataSub(minuend, subtrahend, PI * 2);
};

/**
 * @brief 处理角度跨越 0 的情况，单位为度
 * 
 * @param minuend 被减数，也就是目标角度
 * @param subtrahend 减数，也就是实际反馈的角度值
 * @return 返回处理后的角度值，角度值的范围在 -180 到 180 之间
 * 
 * @details 该函数用于处理角度跨越 0 的情况，单位为度。首先调用 PeriodDataSub 函数处理 minuend 和 subtrahend 的差值，
 * 然后将处理后的差值加到 subtrahend 上，返回结果。
 */
inline float HandleAngleCross0Deg(float minuend, float subtrahend)
{
  return subtrahend + PeriodDataSub(minuend, subtrahend, 360.0f);
};

/**
 * @brief       将弧度制角度转换为角度制
 * @param        angle: 待转换角度，单位：rad
 * @retval       输入角度的角度制表示
 * @note        None
 */
inline float Rad2Deg(float angle) { return angle * kRad2DegCoff; }

/**
 * @brief       将角度制角度转换为弧度制
 * @param        angle: 待转换角度，单位：deg
 * @retval       输入角度的弧度制表示
 * @note        None
 */
inline float Deg2Rad(float angle) { return angle * kDeg2RadCoff; }
}  // namespace hello_world

#endif /* HW_COMPONENTS_TOOLS_BASE_HPP_ */
