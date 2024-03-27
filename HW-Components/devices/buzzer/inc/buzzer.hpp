/**
 *******************************************************************************
 * @file      : buzzer.hpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2023-11-28      Caikunzhen      1. 未测试版本
 *  V1.0.0      2023-12-05      Caikunzhen      1. 完成测试
 *  V1.0.1      2024-01-22      Caikunzhen      1. 修复频率计算错误问题
 *******************************************************************************
 * @attention : 使用前请先确保定时器的频率为 1MHz
 *******************************************************************************
 *  Copyright (c) 2023 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_BUZZER_BUZZER_HPP_
#define HW_COMPONENTS_DEVICES_BUZZER_BUZZER_HPP_

/* Includes ------------------------------------------------------------------*/
#include "allocator.hpp"
#include "stm32h7xx_hal.h"

namespace hello_world
{
namespace buzzer
{
/* Exported macro ------------------------------------------------------------*/

enum Tune : uint8_t {
  kTuneC3 = 48U,        ///* C3
  kTuneC3S,             ///* C3# (Sharp)
  kTuneD3F = kTuneC3S,  ///* D3b (Flat)
  kTuneD3,              ///* D3
  kTuneD3S,             ///* D3# (Sharp)
  kTuneE3F = kTuneD3S,  ///* E3b (Flat)
  kTuneE3,              ///* E3
  kTuneF3,              ///* F3
  kTuneF3S,             ///* F3# (Sharp)
  kTuneG3F = kTuneF3S,  ///* G3b (Flat)
  kTuneG3,              ///* G3
  kTuneG3S,             ///* G3# (Sharp)
  kTuneA3F = kTuneG3S,  ///* A3b (Flat)
  kTuneA3,              ///* A3
  kTuneA3S,             ///* A3# (Sharp)
  kTuneB3F = kTuneA3S,  ///* B3b (Flat)
  kTuneB3,              ///* B3

  kTuneC4,              ///* C4 (中央C)
  kTuneC4S,             ///* C4# (Sharp)
  kTuneD4F = kTuneC4S,  ///* D4b (Flat)
  kTuneD4,              ///* D4
  kTuneD4S,             ///* D4# (Sharp)
  kTuneE4F = kTuneD4S,  ///* E4b (Flat)
  kTuneE4,              ///* E4
  kTuneF4,              ///* F4
  kTuneF4S,             ///* F4# (Sharp)
  kTuneG4F = kTuneF4S,  ///* G4b (Flat)
  kTuneG4,              ///* G4
  kTuneG4S,             ///* G4# (Sharp)
  kTuneA4F = kTuneG4S,  ///* A4b (Flat)
  kTuneA4,              ///* A4, 440Hz
  kTuneA4S,             ///* A4# (Sharp)
  kTuneB4F = kTuneA4S,  ///* B4b (Flat)
  kTuneB4,              ///* B4

  kTuneC5,              ///* C5
  kTuneC5S,             ///* C5# (Sharp)
  kTuneD5F = kTuneC5S,  ///* D5b (Flat)
  kTuneD5,              ///* D5
  kTuneD5S,             ///* D5# (Sharp)
  kTuneE5F = kTuneD5S,  ///* E5b (Flat)
  kTuneE5,              ///* E5
  kTuneF5,              ///* F5
  kTuneF5S,             ///* F5# (Sharp)
  kTuneG5F = kTuneF5S,  ///* G5b (Flat)
  kTuneG5,              ///* G5
  kTuneG5S,             ///* G5# (Sharp)
  kTuneA5F = kTuneG5S,  ///* A5b (Flat)
  kTuneA5,              ///* A5
  kTuneA5S,             ///* A5# (Sharp)
  kTuneB5F = kTuneA5S,  ///* B5b (Flat)
  kTuneB5,              ///* B5

  kTuneC6,              ///* C6
  kTuneC6S,             ///* C6# (Sharp)
  kTuneD6F = kTuneC6S,  ///* D6b (Flat)
  kTuneD6,              ///* D6
  kTuneD6S,             ///* D6# (Sharp)
  kTuneE6F = kTuneD6S,  ///* E6b (Flat)
  kTuneE6,              ///* E6
  kTuneF6,              ///* F6
  kTuneF6S,             ///* F6# (Sharp)
  kTuneG6F = kTuneF6S,  ///* G6b (Flat)
  kTuneG6,              ///* G6
  kTuneG6S,             ///* G6# (Sharp)
  kTuneA6F = kTuneG6S,  ///* A6b (Flat)
  kTuneA6,              ///* A6
  kTuneA6S,             ///* A6# (Sharp)
  kTuneB6F = kTuneA6S,  ///* B6b (Flat)
  kTuneB6,              ///* B6

  kTuneRst,  ///* 休止符
  kTuneEnd   ///* 结束标志
};

enum PlayConfig : uint8_t {
  kPlayConfigLoopPlayback,    ///* 循环播放
  kPlayConfigSinglePlayback,  ///* 单次播放
};
/* Exported constants --------------------------------------------------------*/

const size_t kTuneListMaxLen = 512u;
/* Exported types ------------------------------------------------------------*/

struct TuneListInfo {
  float intensity_scale;   ///* 声音强度，(0, 1]
  uint16_t tune_duration;  ///* 单音持续时长，单位：ms
  Tune list[kTuneListMaxLen];
};

class Buzzer : public MemMang
{
 public:
  Buzzer(TIM_HandleTypeDef *htim, uint32_t channel, PlayConfig play_config,
         const TuneListInfo *kTuneListInfoPtr);
  virtual ~Buzzer() {}

  bool is_playing(void) const { return is_playing_; }

  PlayConfig get_play_config(void) const { return play_config_; }

  void set_play_config(PlayConfig play_config);

  /**
   * @brief       关闭蜂鸣器
   * @retval       None
   * @note        None
   */
  void mute(void) const { HAL_TIM_PWM_Stop(kHtim_, kChannel_); }

  void play(void);

  void setNewTune(const TuneListInfo *kTuneListInfoPtr);

  TIM_HandleTypeDef *const kHtim_;
  const uint32_t kChannel_;

 private:
  uint32_t tune2AutoReload(Tune tune) const;

  const TuneListInfo *kTuneListInfoPtr_;
  size_t tune_idx_;
  uint32_t tune_start_tick_;
  PlayConfig play_config_;
  bool tune_switch_;
  bool is_playing_;
  bool last_is_playing_;
};
/* Exported variables --------------------------------------------------------*/

static const TuneListInfo kWarningList1B = {
    .intensity_scale = 1.0f,
    .tune_duration = 500,
    .list = {kTuneA4, kTuneRst, kTuneEnd}};

static const TuneListInfo kWarningList2B = {
    .intensity_scale = 1.0f,
    .tune_duration = 250,
    .list = {kTuneA4, kTuneRst, kTuneA4, kTuneRst, kTuneRst,
             kTuneRst, kTuneRst, kTuneRst, kTuneEnd}};

static const TuneListInfo kWarningList3B = {
    .intensity_scale = 1.0f,
    .tune_duration = 167,
    .list = {kTuneA4, kTuneRst, kTuneA4, kTuneRst, kTuneA4, kTuneRst,
             kTuneRst, kTuneRst, kTuneRst, kTuneRst, kTuneRst, kTuneRst,
             kTuneEnd}};

static const TuneListInfo kWarningList4B = {
    .intensity_scale = 1.0f,
    .tune_duration = 125,
    .list = {kTuneA4, kTuneRst, kTuneA4, kTuneRst,
             kTuneA4, kTuneRst, kTuneA4, kTuneRst,
             kTuneRst, kTuneRst, kTuneRst, kTuneRst,
             kTuneRst, kTuneRst, kTuneRst, kTuneRst, kTuneEnd}};

/** 德彪西-贝加梅斯克组曲IV */
static const TuneListInfo kMusicDemo = {
    .intensity_scale = 0.005f,
    .tune_duration = 100,
    .list = {
        kTuneRst, kTuneRst, kTuneRst, kTuneRst,
        kTuneRst, kTuneRst, kTuneRst, kTuneRst,
        kTuneG5F, kTuneG5F, kTuneG5F, kTuneG5F,
        kTuneG5F, kTuneG5F, kTuneG5F, kTuneG5F,

        kTuneG5F, kTuneG5F, kTuneG5F, kTuneRst,
        kTuneD6F, kTuneD6F, kTuneRst, kTuneRst,
        kTuneB5, kTuneB5, kTuneB5, kTuneB5,
        kTuneB5, kTuneB5, kTuneB5, kTuneB5,

        kTuneB5, kTuneB5, kTuneB5, kTuneRst,
        kTuneA5, kTuneRst, kTuneA5F, kTuneRst,
        kTuneG5F, kTuneG5F, kTuneRst, kTuneRst,
        kTuneE5, kTuneRst, kTuneD5, kTuneRst,

        kTuneRst, kTuneRst, kTuneD5F, kTuneRst,
        kTuneD5, kTuneRst, kTuneE5, kTuneRst,
        kTuneG5F, kTuneG5F, kTuneRst, kTuneRst,
        kTuneA5, kTuneA5, kTuneRst, kTuneRst,

        kTuneA5F, kTuneA5F, kTuneA5F, kTuneA5F,
        kTuneA5F, kTuneA5F, kTuneA5F, kTuneA5F,
        kTuneA5F, kTuneA5F, kTuneA5F, kTuneA5F,
        kTuneA5F, kTuneA5F, kTuneA5F, kTuneA5F,

        kTuneA5F, kTuneA5F, kTuneA5F, kTuneA5F,
        kTuneA5F, kTuneA5F, kTuneA5F, kTuneRst,
        kTuneA5, kTuneA5, kTuneA5, kTuneRst,
        kTuneA6F, kTuneA6F, kTuneRst, kTuneRst,

        kTuneA5F, kTuneA5F, kTuneA5F, kTuneA5F,
        kTuneA5F, kTuneA5F, kTuneA5F, kTuneA5F,
        kTuneG5F, kTuneG5F, kTuneG5F, kTuneRst,
        kTuneE6, kTuneE6, kTuneRst, kTuneRst,

        kTuneE5, kTuneE5, kTuneE5, kTuneE5,
        kTuneRst, kTuneRst, kTuneRst, kTuneRst,
        kTuneE5F, kTuneE5F, kTuneE5F, kTuneRst,
        kTuneD6F, kTuneD6F, kTuneRst, kTuneRst,

        kTuneD5F, kTuneD5F, kTuneD5F, kTuneD5F,
        kTuneD5F, kTuneD5F, kTuneD5F, kTuneD5F,
        kTuneD5F, kTuneD5F, kTuneD5F, kTuneD5F,
        kTuneD5F, kTuneD5F, kTuneD5F, kTuneD5F,

        kTuneEnd}};
/* Exported function prototypes ----------------------------------------------*/
}  // namespace buzzer
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_BUZZER_BUZZER_HPP_ */
