/** 
 * @file      basic_pid.cpp
 * @author    ZhouShichan (zsc19823382069@163.com)
 * @date      2024-01-28
 * @brief     
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
/* Includes ------------------------------------------------------------------*/
#include "basic_pid.hpp"

#include "tick.hpp"
/* Private macro -------------------------------------------------------------*/

namespace hello_world
{
namespace pid
{
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

BasicPid::State BasicPid::calc(const float *ref_ls, const float *fdb_ls, const float *ffd_ls, float *out_ls)
{
  // 重置计算状态
  datas_.calc_state = CalcState::kPidCalcStateNone;

  // 获取时间间隔 单位：秒
  // 如果间隔超过阈值将重置节点
  uint32_t tick     = tick::GetTickMs();
  uint32_t interval = tick - datas_.last_tick;
  datas_.last_tick  = tick;
  if (interval <= 1) {
    interval = 1;
  }
  if (params_.max_interval_ms && interval > params_.max_interval_ms) {
    interval           = 1;
    datas_.calc_state |= CalcState::kPidClacStateOverWait;
    if (params_.auto_reset) {
      reset();
    }
  }
  datas_.interval_ms = interval;

  // 记录上次PID控制器的输出
  datas_.last_ref = datas_.ref;
  datas_.last_fdb = datas_.fdb;
  datas_.last_out = datas_.out;
  datas_.last_err = datas_.err;

  // 更新本次PID控制器的输入
  float ref = (ref_ls == nullptr) ? 0 : ref_ls[0];
  if (ref_ls == nullptr) {
    datas_.calc_state |= CalcState::kPidCalcStateLackRef;
  }

  float fdb = (fdb_ls == nullptr) ? 0 : fdb_ls[0];
  if (fdb_ls == nullptr) {
    datas_.calc_state |= CalcState::kPidCalcStateLackFdb;
  }

  datas_.ref = params_.setpoint_ramping.calc(ref, datas_.last_ref);
  datas_.fdb = fdb;
  datas_.err = params_.period_sub.calc(datas_.ref, datas_.fdb);
  datas_.err = params_.dead_band.calc(datas_.err);
  datas_.ffd = (ffd_ls != nullptr) ? ffd_ls[0] : 0;

  float out = datas_.ffd;

  // 比例项计算
  datas_.prop = datas_.err * params_.kp;
  out        += datas_.prop;

  // 积分项计算
  if (params_.ki) {
    float inte_err = params_.inte_trapezoidal.calc(datas_.err, datas_.last_err);
    float k_icr    = params_.inte_changing_rate.calc(inte_err);
    float k_is     = params_.inte_separation.calc(inte_err);
    float k_iaw    = params_.inte_anti_windup.calc(inte_err, datas_.inte);
    datas_.inte   += (k_icr * k_iaw) * params_.ki * datas_.interval_ms * inte_err;
    out           += k_is * datas_.inte;
  } else {
    datas_.inte = 0;
  }

  // 微分项计算
  if (params_.kd) {
    float diff  = params_.diff_previous.calc(datas_.fdb, datas_.last_fdb, datas_.err, datas_.last_err) * (params_.kd / datas_.interval_ms);
    datas_.diff = params_.diff_filter.calc(diff, datas_.diff);
    out        += datas_.diff;
  } else {
    datas_.diff = 0;
  }

  // 输出限幅
  datas_.out = params_.out_limit.calc(out);

  // 输出
  if (out_ls != nullptr) {
    out_ls[0] = datas_.out;
  } else {
    datas_.calc_state |= CalcState::kPidCalcStateFailedOut;
  }
  datas_.calc_state |= CalcState::kPidCalcStateOk;
  return datas_.calc_state == CalcState::kPidCalcStateOk ? State::kControllerStateOk : State::kControllerStateError;
};

/* Private function definitions ----------------------------------------------*/
}  // namespace pid
}  // namespace hello_world
