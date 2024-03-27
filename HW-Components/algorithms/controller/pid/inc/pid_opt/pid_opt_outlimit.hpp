/** 
 * @file      pid_opt_outlimit.hpp
 * @author    ZhouShichan (zsc19823382069@163.com)
 * @date      2024-01-28
 * @brief     输出限幅优化器的实现
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
 * | 1.0.0 | 2024-MM-DD | ZhouShichan | 创建文件 |
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HWCOMPONENTS_ALGORITHMS_CONTROLLER_PID_OPT_OUT_LIMIT_HPP_
#define HWCOMPONENTS_ALGORITHMS_CONTROLLER_PID_OPT_OUT_LIMIT_HPP_

/* Includes ------------------------------------------------------------------*/
#include "pid_opt_core.hpp"
/* Exported macro ------------------------------------------------------------*/
namespace hello_world
{
namespace pid
{
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/**
 * @class OutLimit
 * @brief 输出限幅优化器
 * 
 * 为避免输出过大，可采用输出限幅的方法。
 * - 具体计算公式见`calc`。
 * - 参数合法性检查见`isLegal`。
 */
class __PID_PACKED_PARAMS OutLimit : public PidOptimizer
{
 private:
  float lower_ = 0.0f;  ///< 输出限幅下限
  float upper_ = 0.0f;  ///< 输出限幅上限
 public:
  explicit OutLimit() = default;
  /** 构造函数 @see setParams */
  explicit OutLimit(bool is_enabled, float lower, float upper) { setParams(is_enabled, lower, upper); };

  /** 设置输出限幅下限 @see setParams */
  State setLower(float lower) { return setParams(enabled_, lower, upper_); };

  /** 设置输出限幅上限 @see setParams */
  State setUpper(float upper) { return setParams(enabled_, lower_, upper); };

  /** 获取输出限幅下限 */
  float getLower() const { return lower_; };

  /** 获取输出限幅上限 */
  float getUpper() const { return upper_; };

  /** 当且仅当输出限幅下限小于等于上限时，参数合法 */
  bool isLegal() const { return lower_ <= upper_; };

  /**
   * @brief 设置输出限幅优化的参数
   * 
   * @param is_enabled 是否启用
   * @param lower 输出限幅下限
   * @param upper 输出限幅上限
   * @retval State::kPidOptStateOk 参数设置成功
   * @retval State::kPidOptStateIllegalParams 参数非法，已经自动处理
   * @see handleIllegalParams
   */
  State setParams(bool is_enabled, float lower, float upper)
  {
    enabled_ = is_enabled;
    lower_   = lower;
    upper_   = upper;
    return handleIllegalParams() ? State::kPidOptStateIllegalParams : State::kPidOptStateOk;
  };
  /**
   * @brief 检查和处理非法参数
   * 
   * 若下限大于上限，则交换上下限，确保上限大于等于下限。
   * @retval true 参数非法，已经处理
   * @retval false 参数合法
   */
  bool handleIllegalParams()
  {
    if (lower_ > upper_) {
      float tmp = upper_;
      upper_    = lower_;
      lower_    = tmp;
      return true;
    }
    return false;
  };

  /**
   * @brief 计算限幅优化后的输出值
   * @param out 未经过限幅优化的输出值
   * @return 如果优化器启用，若参数合法将会返回限幅区间内的值，若参数非法将返回 0；若优化器未启用，将直接返回输入值。
   */
  float calc(float out)
  {
    if (isEnabled()) {
      if (isLegal()) {
        if (out > upper_) {
          return upper_;
        } else if (out < lower_) {
          return lower_;
        } else {
          return out;
        }
      } else {
        return 0;
      }
    }
    return out;
  };
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace pid
}  // namespace hello_world

#endif /* HWCOMPONENTS_ALGORITHMS_CONTROLLER_PID_OPT_OUT_LIMIT_HPP_  */
