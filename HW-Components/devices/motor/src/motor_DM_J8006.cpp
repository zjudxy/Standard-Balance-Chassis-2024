/**
 *******************************************************************************
 * @file      : motor_DM_J8006.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2024-05-01      Caikunzhen      1. 未测试版本
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2023 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "motor_DM_J8006.hpp"

#include <string.h>

#include "assert.hpp"

namespace hello_world
{
namespace motor
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

static const MotorBaseInfo kDM_J8006MotorBaseInfo{
    .raw_input_lim = kInvalidValue,
    .torq_input_lim = 20.0f,
    .curr_input_lim = 18.0f,
    .torq_const = 1.23f,
    .redu_rat = 1.0f,  ///* DM J8006 为输出端编码，且减速器（6:1）难以拆卸
    .angle_rat = kInvalidValue,
    .vel_rat = kInvalidValue,
    .curr_rat = kInvalidValue,
    .torq_rat = kInvalidValue,
    .cross_0_value = static_cast<uint16_t>(kInvalidValue),
    .raw_mapping_type = kRawMappingTypeTorq,
};

static const float kMaxAngle = PI;
static const uint8_t kAngleBits = 16;
static const float kMaxVel = 21.0f;
static const uint8_t kVelBits = 12;
static const float kMaxTorq = 21.0f;
static const uint8_t kTorqBits = 12;
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
 * @brief       根据给定的范围和位数，将 uint 转换为 float
 * @param        x_uint: 待转换的 uint
 * @param        x_max: 给定范围的最大值
 * @param        x_min: 给定范围的最小值
 * @param        bits: 整形的位数
 * @retval       转换后的 float 变量
 * @note        该函数仅供 DM_J8006 电机数据处理用
 */
static inline float _Uint2Float(
    uint16_t x_int, float x_max, float x_min, uint8_t bits)
{
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
 * @brief       根据给定的范围和位数，将 float 转换为 uint
 * @param        x_float: 待转换的 float
 * @param        x_max: 给定范围的最大值
 * @param        x_min: 给定范围的最小值
 * @param        bits: 整形的位数
 * @retval       转换后的 uint 变量
 * @note        该函数仅供 DM_J8006 电机数据处理用
 */
static inline uint16_t _Float2Uint(
    float x_float, float x_max, float x_min, uint8_t bits)
{
  float span = x_max - x_min;
  float offset = x_min;
  return (uint16_t)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}

/**
 * @brief       DM_J8006 初始化
 * @param        id: 电机 ID，1到10
 * @param        optinal_params: 电机可选配置参数
 * @param        auto_enable: 电机自动使能，老版本电机需设置为false
 * @retval       None
 * @note        老版本电机需将auto_enable设置为false，并需每隔一段时间通过
 * set_input_type方法将输入方式修改为kInputTypeCmd，并使用setInput发送kCmdEnable以
 * 确保电机处于使能状态，然后再将输入方式修改为非命令输入，向电机发送指令
 */
DM_J8006::DM_J8006(uint8_t id, const OptionalParams& optinal_params,
                   bool auto_enable) : Motor()
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(1 <= id && id <= 10, "Error id: %d", id);
  HW_ASSERT(
      optinal_params.dir == kDirFwd || optinal_params.dir == kDirRev,
      "Error dir: %d", optinal_params.dir);
  HW_ASSERT(
      optinal_params.angle_range == kAngleRange0To2Pi ||
          optinal_params.angle_range == kAngleRangeNegInfToPosInf ||
          optinal_params.angle_range == kAngleRangeNegPiToPosPi,
      "Error angle range: %d", optinal_params.angle_range);
  HW_ASSERT(
      optinal_params.input_type == kInputTypeRaw ||
          optinal_params.input_type == kInputTypeTorq ||
          optinal_params.input_type == kInputTypeCurr,
      "Error input type: %d", optinal_params.input_type);
  HW_ASSERT(
      optinal_params.ex_redu_rat > 0,
      "Error external reduction ration: %f", optinal_params.ex_redu_rat);
  HW_ASSERT(
      optinal_params.max_torq_input_lim > 0,
      "Error max torque input limit: %f", optinal_params.max_torq_input_lim);
  HW_ASSERT(
      optinal_params.max_curr_input_lim > 0,
      "Error max current input limit: %f", optinal_params.max_curr_input_lim);
#pragma endregion

  motor_info_ = kDM_J8006MotorBaseInfo;

  /* 根据 ID 设定报文 ID */
  motor_info_.tx_id = kTx0_ + id;
  motor_info_.rx_id = kRx0_ + id;
  motor_info_.id = id;

  motor_info_.dir = optinal_params.dir;
  motor_info_.angle_range = optinal_params.angle_range;
  motor_info_.input_type = optinal_params.input_type;
  motor_info_.angle_offset = optinal_params.angle_offset;
  /* 转子端力矩限制 */
  float rotor_torq_lim = motor_info_.torq_input_lim / motor_info_.redu_rat;
  if (optinal_params.remove_build_in_reducer) {
    motor_info_.redu_rat = optinal_params.ex_redu_rat;
  } else {
    motor_info_.redu_rat *= optinal_params.ex_redu_rat;
  }

  /* 计算输入限制 */
  float max_torq_input = rotor_torq_lim * motor_info_.redu_rat;
  if (optinal_params.max_torq_input_lim != std::numeric_limits<float>::max()) {
    max_torq_input = hello_world::Min(max_torq_input, optinal_params.max_torq_input_lim);
  }
  float max_curr_input = motor_info_.curr_input_lim;
  if (optinal_params.max_curr_input_lim != std::numeric_limits<float>::max()) {
    max_curr_input = hello_world::Min(max_curr_input, optinal_params.max_curr_input_lim);
  }

  max_torq_input = hello_world::Min(max_torq_input, kMaxTorq);
  max_torq_input = hello_world::Min(max_torq_input, curr2torq(max_curr_input));
  motor_info_.raw_input_lim = kInvalidValue;
  motor_info_.torq_input_lim = max_torq_input;
  motor_info_.curr_input_lim = torq2curr(max_torq_input);

  wait_to_handle_cmd_ = (Cmd)(kCmdDisable);

  is_connected_ = false;
  is_enabled_ = false;
  auto_enable_ = auto_enable;
}

State DM_J8006::setInput(float input)
{
  float torq_input = 0;
  switch (motor_info_.input_type) {
    case kInputTypeRaw:
      torq_input = raw2torq(input);
      torq_input_ = hello_world::Bound(
          input, motor_info_.torq_input_lim, -motor_info_.torq_input_lim);
      curr_input_ = raw2torq(raw_input_);
      raw_input_ = torq2raw(torq_input_);
      if (fabsf(torq_input_) < fabsf(torq_input)) {
        return kStateInputValueOverflow;
      } else {
        return kStateOk;
      }
    case kInputTypeTorq:
      torq_input_ = hello_world::Bound(
          input, motor_info_.torq_input_lim, -motor_info_.torq_input_lim);
      raw_input_ = torq2raw(torq_input_);
      curr_input_ = torq2curr(torq_input_);
      if (fabsf(torq_input_) < fabsf(input)) {
        return kStateInputValueOverflow;
      } else {
        return kStateOk;
      }
    case kInputTypeCurr:
      curr_input_ = hello_world::Bound(
          input, motor_info_.curr_input_lim, -motor_info_.curr_input_lim);
      raw_input_ = curr2raw(curr_input_);
      torq_input_ = curr2torq(torq_input_);
      if (fabsf(curr_input_) < fabsf(input)) {
        return kStateInputValueOverflow;
      } else {
        return kStateOk;
      }
    case kInputTypeCmd:
      if (input == kCmdEnable || input == kCmdDisable ||
          input == kCmdClearErr) {
        wait_to_handle_cmd_ = (Cmd)(wait_to_handle_cmd_ | (uint8_t)input);
        return kStateOk;
      } else {
        return kStateInputValueOverflow;
      }
      break;
    default:
      return kStateInputTypeError;
  }
}

State DM_J8006::set_input_type(InputType input_type)
{
  if (input_type == kInputTypeRaw || input_type == kInputTypeTorq ||
      input_type == kInputTypeCurr || input_type == kInputTypeCmd) {
    motor_info_.input_type = input_type;
    return kStateOk;
  } else {
    return kStateInputTypeError;
  }
}

/**
 * @brief       将要发给电调的期望输值编码为对应的 CAN 报文
 * @param        tx_data: 将要发出的 CAN 报文
 * @param        tx_msg_std_id: 发给电调的报文的ID
 * @retval       当 ID 匹配时返回 kStateOk，否则返回 kStateIdDismatch
 * @note        1. 只有当 tx_msg_std_id 与实际的发给电机报文的 ID 对应时才会生成
 *              报文，该 ID 可以通过 tx_id 方法获取
 *              2. 只会在报文需要的位置上进行修改，区域位置的内容会保持不变
 */
State DM_J8006::encode(uint8_t tx_data[8], uint32_t tx_msg_std_id)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(tx_data, "Error pointer");
#pragma endregion

  if (tx_msg_std_id != motor_info_.tx_id) {
    return kStateIdDismatch;
  } else {
    if (wait_to_handle_cmd_ == kCmdNone &&
        motor_info_.input_type != kInputTypeCmd) {
      if (is_enabled_ || !auto_enable_) {
        uint16_t input = curr2raw(
            static_cast<float>(motor_info_.dir) * curr_input_);
        input = hello_world::Bound(
            input, static_cast<uint16_t>(0),
            static_cast<uint16_t>((0x01 << kTorqBits) - 1));
        memset(tx_data, 0x00, sizeof(uint8_t) * 7);
        tx_data[6] = (input >> 8) & 0x0F;
        tx_data[7] = input & 0xFF;
      } else {
        wait_to_handle_cmd_ = (Cmd)(wait_to_handle_cmd_ | kCmdEnable);
      }
    }

    if (wait_to_handle_cmd_ & kCmdClearErr) {
      memset(tx_data, 0xFF, sizeof(uint8_t) * 7);
      tx_data[7] = 0xFB;
      if (is_connected_) {
        wait_to_handle_cmd_ = (Cmd)(wait_to_handle_cmd_ & (~kCmdClearErr));
      }
    } else if (wait_to_handle_cmd_ & kCmdDisable) {
      memset(tx_data, 0xFF, sizeof(uint8_t) * 7);
      tx_data[7] = 0xFD;
      if (is_connected_) {
        wait_to_handle_cmd_ = (Cmd)(wait_to_handle_cmd_ & (~kCmdDisable));
      }
    } else if (wait_to_handle_cmd_ & kCmdEnable) {
      memset(tx_data, 0xFF, sizeof(uint8_t) * 7);
      tx_data[7] = 0xFC;
      if (is_connected_) {
        wait_to_handle_cmd_ = (Cmd)(wait_to_handle_cmd_ & (~kCmdEnable));
      }
    }

    return kStateOk;
  }
}

/**
 * @brief       将电调发回的 CAN 报文进行解包
 * @param        rx_data: 电调发回的 CAN 报文
 * @param        rx_msg_std_id: 电调发回的报文的ID
 * @retval       当 ID 匹配时返回 kStateOk，否则返回 kStateIdDismatch
 * @note        只有当 rx_msg_std_id 与实际的电机会发回报文的 ID 对应时才会对报文
 *              进行解包，该 ID 可以通过 rx_id 方法获取
 */
State DM_J8006::decode(const uint8_t rx_data[8], uint32_t rx_msg_std_id)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(rx_data, "Error pointer");
#pragma endregion

  if (rx_msg_std_id != motor_info_.rx_id) {
    return kStateIdDismatch;
  } else {
    uint16_t raw_angle = static_cast<uint16_t>((rx_data[1] << 8) | rx_data[2]);
    float angle = _Uint2Float(raw_angle, kMaxAngle, -kMaxAngle, kAngleBits);
    float last_angle = _Uint2Float(
        last_raw_angle_, kMaxAngle, -kMaxAngle, kAngleBits);

    /* 判断是否旋转了一圈 */
    float round = round_;
    float delta_angle = angle - last_angle;
    if (fabsf(delta_angle) > 2 * PI * kCross0ValueThres) {
      delta_angle < 0 ? round++ : round--;

      /* 避免圈数溢出 */
      if (motor_info_.angle_range != kAngleRangeNegInfToPosInf) {
        round = fmodf(round, motor_info_.redu_rat);
      }
    }

    float actual_ang =
        static_cast<float>(motor_info_.dir) * (angle + round * 2 * PI) /
        motor_info_.redu_rat;
    float raw = static_cast<uint16_t>(((rx_data[4] & 0x0F) << 8) | rx_data[5]);

    /* 统一对电机状态赋值 */
    round_ = round;
    actual_angle_ = normAngle(actual_ang);
    angle_ = normAngle(actual_ang - motor_info_.angle_offset);
    vel_ = static_cast<float>(motor_info_.dir) *
           _Uint2Float(
               static_cast<int16_t>((rx_data[3] << 4) | (rx_data[3] >> 4)),
               kMaxVel, -kMaxVel, kVelBits) /
           motor_info_.redu_rat;
    torq_ = static_cast<float>(motor_info_.dir) * raw2torq(raw);
    curr_ = static_cast<float>(motor_info_.dir) * raw2curr(raw);
    MOS_temp_ = rx_data[6];
    rotor_temp_ = rx_data[7];
    state_code_ = static_cast<StateCode>(rx_data[0] >> 4);

    if (state_code_ == kStateCodeMotorEnable) {
      is_enabled_ = true;
    } else if (state_code_ == kStateCodeMotorDisable) {
      is_enabled_ = false;
    } else {
      wait_to_handle_cmd_ =
          (Cmd)(wait_to_handle_cmd_ | kCmdClearErr | kCmdDisable | kCmdEnable);
    }

    last_raw_angle_ = raw_angle;

    is_connected_ = true;

    return kStateOk;
  }
}

/**
 * @brief       将原始报文内容转换为输出端力矩
 * @param        raw: 原始报文数值
 * @retval       原始报文对应的输出端力矩值，单位：N·m
 * @note        报文的对应的物理意义需要与设定的相符，设定对应情可以通过
 *              motor_info 方法获取电机信息结构体，查看其中的 raw_mapping_type
 *              变量
 */
float DM_J8006::raw2torq(float raw) const
{
  return _Uint2Float(raw, kMaxTorq, -kMaxTorq, kTorqBits) *
         motor_info_.redu_rat;
}

/**
 * @brief       将输出端力矩转换为原始报文内容
 * @param        torq: 输出端力矩值，单位：N·m
 * @retval       输出端力矩值对应的原始报文
 * @note        报文的对应的物理意义需要与设定的相符，设定对应情可以通过
 *              motor_info 方法获取电机信息结构体，查看其中的 raw_mapping_type
 *              变量
 */
float DM_J8006::torq2raw(float torq) const
{
  return _Float2Uint(
      torq / motor_info_.redu_rat, kMaxTorq, -kMaxTorq, kTorqBits);
}

/**
 * @brief       将原始报文内容转换为转子电流
 * @param        raw: 原始报文数值
 * @retval       原始报文对应的转子电流值，单位：A
 * @note        报文的对应的物理意义需要与设定的相符，设定对应情可以通过
 *              motor_info 方法获取电机信息结构体，查看其中的 raw_mapping_type
 *              变量
 */
float DM_J8006::raw2curr(float raw) const
{
  return _Uint2Float(raw, kMaxTorq, -kMaxTorq, kTorqBits) /
         motor_info_.torq_const;
}

/**
 * @brief       将转子电流转换为原始报文内容
 * @param        curr: 转子电流值，单位：A
 * @retval       转子电流值对应的原始报文
 * @note        报文的对应的物理意义需要与设定的相符，设定对应情可以通过
 *              motor_info 方法获取电机信息结构体，查看其中的 raw_mapping_type
 *              变量
 */
float DM_J8006::curr2raw(float curr) const
{
  return _Float2Uint(
      curr * motor_info_.torq_const, kMaxTorq, -kMaxTorq, kTorqBits);
}
}  // namespace motor
}  // namespace hello_world
