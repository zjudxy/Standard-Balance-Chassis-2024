/** 
 * @file      pid_opt_period_sub.hpp
 * @author    ZhouShichan (zsc19823382069@163.com)
 * @date      2024-01-28
 * @brief     周期最小差值优化器的实现
 * @par last editor  ZhouShichan (zsc19823382069@163.com)
 * @version   1.0.0
 * 
 * @copyright Copyright (c) 2024 Hello World Team, Zhejiang University. All Rights Reserved.
 * 
 * @attention 
 * 
 * @par history
 * | Version | Date | Author | Description |
 * | :---: | :---: | :---: | :---: |
 * | 1.0.0 | 2024-MM-DD | ZhouShichan | description |
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FILE_HPP_
#define __FILE_HPP_

/* Includes ------------------------------------------------------------------*/
#include "math.h"
#include "pid_opt_core.hpp"
/* Exported macro ------------------------------------------------------------*/
namespace hello_world
{
namespace pid
{
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

class __PID_PACKED_PARAMS PeriodSub : public PidOptimizer
{
 private:
  float period_ = 0.0f;  ///< 周期值

 public:
  explicit PeriodSub() = default;

  /** 构造函数 @see setParams */
  explicit PeriodSub(bool is_enabled, float period) { setParams(is_enabled, period); };

  /** 设置周期值 @see setParams */
  State setPeriod(float period) { return setParams(enabled_, period); };

  /** 获取周期值 */
  float getPeriod() const { return period_; };

  /** 当且仅当周期值大于0.001时，参数合法 */
  bool isLegal() { return period_ > 0.001f; };

  /**
   * @brief 设置周期最小差值优化参数
   * @param is_enabled 是否启用周期最小差值优化
   * @param period 周期值
   * @retval State::kPidOptStateOk 参数设置成功
   * @retval State::kPidOptStateIllegalParams 参数非法，已经自动处理
   * @see handleIllegalParams
   */
  State setParams(bool is_enabled, float period)
  {
    enabled_ = is_enabled;
    period_  = period;
    return handleIllegalParams() ? State::kPidOptStateIllegalParams : State::kPidOptStateOk;
  };
  /**
   * @brief 检查和处理非法参数
   * 
   * 若周期值小于0.001，则将周期值置为0.001。
   * @retval true 参数非法，已经处理
   * @retval false 参数合法
   */
  bool handleIllegalParams()
  {
    if (period_ < 0.001f) {
      period_ = 0.001f;
      return true;
    } else {
      return false;
    }
  };
  /**
   * @brief 计算周期最小差值优化后的误差值
   * 
   * @param ref 参考值
   * @param fdb 反馈值
   * @return 循环周期数据中ref和fdb最小的误差值
   */
  float calc(float ref, float fdb)
  {
    if (isEnabled() && isLegal()) {
      float delta = ref - fdb;
      float times = roundf(delta / period_);
      return delta - times * period_;
    } else {
      return ref - fdb;
    }
  };
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace pid
}  // namespace hello_world

#endif /* __FILE_HPP_ */
