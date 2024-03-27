/**
 *******************************************************************************
 * @file      : motor_MF9025v2.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2023-11-25      Caikunzhen      1. 未测试版本
 *  V1.0.0      2024-01-25      Caikunzhen      1. 完成测试
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2023 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "motor_MF9025v2.hpp"

#include "assert.hpp"

namespace hello_world
{
namespace motor
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

static const MotorBaseInfo kMF9025v2MotorBaseInfo{
    .raw_input_lim = 2000,
    .torq_input_lim = 11.0f,
    .curr_input_lim = 33.0f,
    .torq_const = 0.32f,
    .redu_rat = 1.0f,
    .angle_rat = 2 * PI / 0xFFFF,
    .vel_rat = PI / 180,
    .curr_rat = 32.0f / 2000,
    .torq_rat = kInvalidValue,
    .cross_0_value = 0xFFFFU,
    .raw_mapping_type = kRawMappingTypeCurr,
};
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
 * @brief       MF9025v2 初始化
 * @param        id: 电机 ID，1到4
 * @param        optinal_params: 电机可选配置参数
 * @retval       None
 * @note        None
 */
MF9025v2::MF9025v2(uint8_t id, const OptionalParams& optinal_params) : Motor()
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(1 <= id && id <= 4, "Error id: %d", id);
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
      optinal_params.max_raw_input_lim > 0,
      "Error max raw input limit: %f", optinal_params.max_raw_input_lim);
  HW_ASSERT(
      optinal_params.max_torq_input_lim > 0,
      "Error max torque input limit: %f", optinal_params.max_torq_input_lim);
  HW_ASSERT(
      optinal_params.max_curr_input_lim > 0,
      "Error max current input limit: %f", optinal_params.max_curr_input_lim);
#pragma endregion

  motor_info_ = kMF9025v2MotorBaseInfo;

  /* 根据 ID 设定报文 ID */
  motor_info_.tx_id = kTx1_4_;
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
  float max_raw_input = motor_info_.raw_input_lim;
  if (optinal_params.max_raw_input_lim != std::numeric_limits<float>::max()) {
    max_raw_input = hello_world::Min(max_raw_input, optinal_params.max_raw_input_lim);
  }
  float max_torq_input = rotor_torq_lim * motor_info_.redu_rat;
  if (optinal_params.max_torq_input_lim != std::numeric_limits<float>::max()) {
    max_torq_input = hello_world::Min(max_torq_input, optinal_params.max_torq_input_lim);
  }
  float max_curr_input = motor_info_.curr_input_lim;
  if (optinal_params.max_curr_input_lim != std::numeric_limits<float>::max()) {
    max_curr_input = hello_world::Min(max_curr_input, optinal_params.max_curr_input_lim);
  }

  max_raw_input = hello_world::Min(max_raw_input, torq2raw(max_torq_input));
  max_raw_input = hello_world::Min(max_raw_input, curr2raw(max_curr_input));
  motor_info_.raw_input_lim = max_raw_input;
  motor_info_.torq_input_lim = raw2torq(max_raw_input);
  motor_info_.curr_input_lim = raw2curr(max_raw_input);
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
State MF9025v2::encode(uint8_t tx_data[8], uint32_t tx_msg_std_id)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(tx_data, "Error pointer");
#pragma endregion

  if (tx_msg_std_id != motor_info_.tx_id) {
    return kStateIdDismatch;
  } else {
    uint8_t index = motor_info_.id - 1;

    int16_t input =
        hello_world::Bound(static_cast<float>(motor_info_.dir) * raw_input_,
                           motor_info_.raw_input_lim, -motor_info_.raw_input_lim);
    tx_data[2 * index] = input;
    tx_data[2 * index + 1] = input >> 8;

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
State MF9025v2::decode(const uint8_t rx_data[8], uint32_t rx_msg_std_id)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(rx_data, "Error pointer");
#pragma endregion

  if (rx_msg_std_id != motor_info_.rx_id) {
    return kStateIdDismatch;
  } else {
    uint16_t raw_angle = static_cast<uint16_t>((rx_data[7] << 8) | rx_data[6]);

    /* 判断是否旋转了一圈 */
    float round = round_;
    float delta_angle = raw_angle - last_raw_angle_;
    if (fabsf(delta_angle) > motor_info_.cross_0_value * kCross0ValueThres) {
      delta_angle < 0 ? round++ : round--;

      /* 避免圈数溢出 */
      if (motor_info_.angle_range != kAngleRangeNegInfToPosInf) {
        round = fmodf(round, motor_info_.redu_rat);
      }
    }

    float actual_ang =
        static_cast<float>(motor_info_.dir) *
        (raw_angle * motor_info_.angle_rat + round * 2 * PI) /
        motor_info_.redu_rat;
    float raw = static_cast<float>(motor_info_.dir) *
                static_cast<int16_t>((rx_data[3] << 8) | rx_data[2]);

    /* 统一对电机状态赋值 */
    round_ = round;
    actual_angle_ = normAngle(actual_ang);
    angle_ = normAngle(actual_ang - motor_info_.angle_offset);
    vel_ = static_cast<float>(motor_info_.dir) *
           static_cast<int16_t>((rx_data[5] << 8) | rx_data[4]) *
           motor_info_.vel_rat / motor_info_.redu_rat;
    torq_ = raw2torq(raw);
    curr_ = raw2curr(raw);
    temp_ = rx_data[1];

    last_raw_angle_ = raw_angle;

    return kStateOk;
  }
}
}  // namespace motor
}  // namespace hello_world
