/**
 *******************************************************************************
 * @file      : buzzer.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2023-11-28      Caikunzhen      1. 未测试版本
 *  V1.0.0      2023-12-05      Caikunzhen      1. 完成测试
 *  V1.0.1      2024-01-22      Caikunzhen      1. 修复频率计算错误问题
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2023 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "buzzer.hpp"

#include <cmath>

#include "assert.hpp"
#include "base.hpp"
#include "tick.hpp"

namespace hello_world
{
namespace buzzer
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/

static const float kTimerFreq = 1e6f;
static const float kMaxDuty = 0.5f;  ///* PWM 最大占空比
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
 * @brief       蜂鸣器初始化
 * @param        htim: 定时器句柄指针
 * @param        channel: 定时器PWM对应输出通道
 * @param        kTuneListInfoPtr: 乐曲信息指针，
 * 请确保播放期间指针所指内容没有被释放
 * @retval       None
 * @note        使用前请先确保定时器的频率为 1MHz，
 * 声音强度与播放的音符会被自行限定到合理取值范围中，蜂鸣器默认开启
 */
Buzzer::Buzzer(TIM_HandleTypeDef *htim, uint32_t channel,
               PlayConfig play_config, const TuneListInfo *kTuneListInfoPtr)
    : kHtim_(htim), kChannel_(channel), play_config_(play_config)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(IS_TIM_INSTANCE(kHtim_->Instance), "Error TIM handle");
  HW_ASSERT(IS_TIM_CHANNELS(kChannel_), "Error TIM channel: %d", kChannel_);
  HW_ASSERT(play_config == kPlayConfigLoopPlayback ||
                play_config == kPlayConfigSinglePlayback,
            "Error play config: %d", tune_list_info.play_config);
#pragma endregion

  setNewTune(kTuneListInfoPtr);
}

void Buzzer::set_play_config(PlayConfig play_config)
{
/* 变量检查 */
#pragma region
  HW_ASSERT(play_config == kPlayConfigLoopPlayback ||
                play_config == kPlayConfigSinglePlayback,
            "Error play config: %d", tune_list_info.play_config);
#pragma endregion

  play_config_ = play_config;
}

/**
 * @brief       设置新的乐曲信息
 * @param        kTuneListInfoPtr: 乐曲信息指针，
 * 请确保播放期间指针所指内容没有被释放
 * @retval       None
 * @note        使用前请先确保定时器的频率为 1MHz，
 * 声音强度与播放的音符会被自行限定到合理取值范围中，调用后蜂鸣器会自动开启
 */
void Buzzer::setNewTune(const TuneListInfo *kTuneListInfoPtr)
{
  tune_idx_ = 0;
  tune_start_tick_ = 0;
  tune_switch_ = true;
  is_playing_ = true;
  last_is_playing_ = false;
  kTuneListInfoPtr_ = kTuneListInfoPtr;

  __HAL_TIM_SET_AUTORELOAD(kHtim_, 0);
  __HAL_TIM_SET_COMPARE(kHtim_, kChannel_, 0);
  HAL_TIM_PWM_Start(kHtim_, kChannel_);
}

/**
 * @brief       播放乐曲
 * @retval       None
 * @note        请确保该方法以至少 1kHz 的频率被调用
 */
void Buzzer::play()
{
  if (!is_playing_) {
    return;
  }

  uint32_t tick = tick::GetTickMs();

  /* 播放结束处理 */
  if (kTuneListInfoPtr_->list[tune_idx_] == kTuneEnd) {
    if (play_config_ == kPlayConfigLoopPlayback) {
      tune_switch_ = true;
      tune_idx_ = 0;
    } else {
      is_playing_ = last_is_playing_ = false;
      tune_switch_ = false;
      tune_idx_ = 0;
      mute();
      return;
    }
  }

  /* 切换音符 */
  if (tune_switch_) {
    tune_switch_ = false;
    uint32_t auto_reload = tune2AutoReload(kTuneListInfoPtr_->list[tune_idx_]);
    float intensity_scale =
        hello_world::Bound(kTuneListInfoPtr_->intensity_scale, 0, 1);
    float duty = kMaxDuty * intensity_scale;
    __HAL_TIM_SET_AUTORELOAD(kHtim_, auto_reload);
    __HAL_TIM_SET_COMPARE(kHtim_, kChannel_, auto_reload * duty);
  }

  if (!last_is_playing_ && is_playing_) {  // 刚开始播放
    tune_start_tick_ = tick;
  } else if (tick - tune_start_tick_ >= kTuneListInfoPtr_->tune_duration) {
    tune_start_tick_ = tick;
    tune_idx_++;

    if (tune_idx_ >= kTuneListMaxLen) {  // 音符溢出处理
      is_playing_ = last_is_playing_ = false;
      tune_switch_ = false;
      tune_idx_ = 0;
      mute();
      return;
    } else {
      tune_switch_ = true;
    }
  }

  last_is_playing_ = is_playing_;
}

/**
 * @brief       将音符转化为对应的定时器重载值
 * @param        tune: 音符
 * @retval       对应的定时器重载值
 * @note        音符会被自行限定到合理取值范围中
 */
uint32_t Buzzer::tune2AutoReload(Tune tune) const
{
  if (tune < kTuneC3 || tune >= kTuneRst) {
    return 0;
  } else {
    return static_cast<uint32_t>(
        // 440 * 2^((n-69)/12) Hz
        kTimerFreq / (440.0f * powf(1.05946f, (tune)-69)) - 1);
  }
}
}  // namespace buzzer
}  // namespace hello_world