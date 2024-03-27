/**
 *******************************************************************************
 * @file      : BMI088.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2023-12-08      Caikunzhen      1. 完成第一版测试
 *  V1.0.1      2023-12-12      Caikunzhen      1. 添加旋转配置
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2023 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "BMI088.hpp"

#include <cstring>

#include "BMI088_reg.hpp"
#include "assert.hpp"
#include "base.hpp"

namespace hello_world
{
namespace imu
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/

static const uint8_t kMaxMultiTransLen = 8u;
static const uint8_t kPaddingValue = 0x55;
static uint8_t kPaddingData[kMaxMultiTransLen] = {
    kPaddingValue,
    kPaddingValue,
    kPaddingValue,
    kPaddingValue,
    kPaddingValue,
    kPaddingValue,
    kPaddingValue,
    kPaddingValue,
};

static const float kAccSens[4] = {
    3 * kGravAcc / 32768,
    6 * kGravAcc / 32768,
    12 * kGravAcc / 32768,
    24 * kGravAcc / 32768,
};

static const float kGyroSens[5] = {
    2000 * kDeg2RadCoff / 32768,
    1000 * kDeg2RadCoff / 32768,
    500 * kDeg2RadCoff / 32768,
    250 * kDeg2RadCoff / 32768,
    125 * kDeg2RadCoff / 32768,
};

static const float kTempSens = 0.125f;
static const float kTempOffset = 23.0f;

static const uint8_t kWriteSign = 0x00;
static const uint8_t kReadSign = 0x80;

static const uint8_t kGyroRawDataLen = 6u;
static const uint8_t kAccRawDataLen = 6u;
static const uint8_t kTempRawDataLen = 2u;

static const uint8_t kSpiTimeout = 5u;  ///* 单位：ms

static const uint16_t kAccResetTime = 2u;  ///* 加速度计重启耗时，单位：ms
static const uint8_t kGyroResetTime = 1u;  ///* 陀螺仪重启耗时，单位：ms
static const uint16_t kCommWaitTime = 2u;  ///* 通信间隔时间，单位：us
/** 寄存器写入后读取间隔时间，单位：us */
static const uint16_t kCheckWaitTime = 120u;

/** 加速度计自检测短延时，单位：ms */
static const uint8_t kAccSelfTestShortDelay = 2u;
/** 加速度计自检测长延时，单位：ms */
static const uint8_t kAccSelfTestLongDelay = 50u;
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

static void DelayUs(uint16_t us);

/**
 * @brief       BMI088 初始化
 * @param        hw_config: 硬件配置
 * @param        rot_mat_flatten: 旋转矩阵（展平）
 * [r00 r01 r02 r10 r11 r12 r20 r21 r22]，使用后可释放
 * @param        config: 设备配置
 * @retval       None
 * @note        None
 */
BMI088::BMI088(const BMI088HWConfig &hw_config, const float rot_mat_flatten[9],
               const BMI088Config &config)
    : kHspi_(hw_config.hspi),
      kAccCsPort_(hw_config.acc_cs_port),
      kAccCsPin_(hw_config.acc_cs_pin),
      kGyroCsPort_(hw_config.gyro_cs_port),
      kGyroCsPin_(hw_config.gyro_cs_pin),
      config_(config)
{
  /* 变量检查 */
#pragma region
  HW_ASSERT(IS_SPI_ALL_INSTANCE(kHspi_->Instance), "Error SPI handle");
  HW_ASSERT(IS_GPIO_ALL_INSTANCE(kAccCsPort_), "Error GPIO port");
  HW_ASSERT(IS_GPIO_PIN(kAccCsPin_), "Error GPIO pin");
  HW_ASSERT(IS_GPIO_ALL_INSTANCE(kGyroCsPort_), "Error GPIO port");
  HW_ASSERT(IS_GPIO_PIN(kGyroCsPin_), "Error GPIO pin");
  HW_ASSERT(rot_mat_flatten, "Empty rotation matrix");
  HW_ASSERT(config_.acc_range >= kBMI088AccRange3G &&
                config_.acc_range <= kBMI088AccRange24G,
            "Error Acc range: %d", config_.acc_range);
  HW_ASSERT(config_.acc_odr >= kBMI088AccOdr12_5 &&
                config_.acc_odr <= kBMI088AccOdr1600,
            "Error Acc ODR: %d", config_.acc_odr);
  HW_ASSERT(config_.acc_osr >= kBMI088AccOsr4 &&
                config_.acc_osr <= kBMI088AccOsrNormal,
            "Error Acc OSR: %d", config_.acc_osr);
  HW_ASSERT(config_.gyro_range >= kBMI088GyroRange2000Dps &&
                config_.gyro_range <= kBMI088GyroRange125Dps,
            "Error Gyro range: %d", config_.gyro_range);
  HW_ASSERT(config_.gyro_odr_fbw >= kBMI088GyroOdrFbw2000_532 &&
                config_.gyro_odr_fbw <= kBMI088GyroOdrFbw100_32,
            "Error Gyro ODR and FBW: %d", config_.gyro_odr_fbw);
#pragma endregion

  memcpy(rot_mat_flatten_, rot_mat_flatten, sizeof(float) * 9);
  arm_mat_init_f32(&rot_mat_, 3, 3, rot_mat_flatten_);

  accCsH();
  gyroCsH();
}

/**
 * @brief       进行 BMI088 芯片配置
 * @param        self_test: 是否开启传感器自检测
 * @retval       错误状态
 * @note        该函数内部会阻塞运行，请不要在中断中调用
 */
BMI088ErrState BMI088::init(bool self_test) const
{
  BMI088ErrState err_state = kBMI088ErrStateNoErr;

  err_state = BMI088ErrState(err_state | gyroInit(self_test));
  err_state = BMI088ErrState(err_state | accInit(self_test));

  return err_state;
}

/**
 * @brief       获取传感器数据
 * @param        acc_data: 加速度计三轴数据，[ax ay az]，单位：m/s^2
 * @param        gyro_data: 陀螺仪三轴数据，[wx wy wz]，单位：rad/s
 * @param        temp_ptr: 温度数据指针，单位：℃
 * @retval       None
 * @note        传入 null_ptr 则不获取对应数据
 */
void BMI088::getData(float acc_data[3], float gyro_data[3], float *temp_ptr) const
{
  float tmp_data[3];

  if (acc_data != nullptr) {
    DelayUs(kCommWaitTime);
    getAccData(tmp_data);
    arm_mat_vec_mult_f32(&rot_mat_, tmp_data, acc_data);
  }

  if (gyro_data != nullptr) {
    DelayUs(kCommWaitTime);
    getGyroData(tmp_data);
    arm_mat_vec_mult_f32(&rot_mat_, tmp_data, gyro_data);
  }

  if (temp_ptr != nullptr) {
    DelayUs(kCommWaitTime);
    *temp_ptr = getTemp();
  }
}

/**
 * @brief       加速度计配置
 * @param        self_test: 是否进行自检测
 * @retval       None
 * @note        错误状态
 */
BMI088ErrState BMI088::accInit(bool self_test) const
{
  DelayUs(kCommWaitTime);
  accRead(BMI088_ACC_ACC_CHIP_ID);  // 假读取

  DelayUs(kCommWaitTime);
  if (accRead(BMI088_ACC_ACC_CHIP_ID) != BMI088_ACC_ACC_CHIP_ID_VALUE) {
    return kBMI088ErrStateAccNotFound;
  }

  if (self_test) {
    if (accSelfTest() != kBMI088ErrStateNoErr) {
      return kBMI088ErrStateAccSelfTestFailed;
    }
  }

  // 配置一些寄存器
  uint8_t acc_conf = (config_.acc_osr << _BMI088_ACC_ACC_CONF_ACC_OSR_SHFITS) |
                     (config_.acc_odr << _BMI088_ACC_ACC_CONF_ACC_ODR_SHFITS);
  uint8_t acc_init_config[4][2] = {
      {BMI088_ACC_ACC_RANGE, config_.acc_range},
      {BMI088_ACC_ACC_CONF, acc_conf},
      {BMI088_ACC_ACC_PWR_CTRL, BMI088_ACC_ACC_PWR_CTRL_ON},
      {BMI088_ACC_ACC_PWR_CONF, BMI088_ACC_ACC_PWR_CONF_ACTIVE_MODE}};

  DelayUs(kCommWaitTime);
  accWrite(BMI088_ACC_ACC_SOFTRESET, BMI088_ACC_ACC_SOFTRESET_VALUE);
  HAL_Delay(kAccResetTime);

  DelayUs(kCommWaitTime);
  accRead(BMI088_ACC_ACC_CHIP_ID);  // 假读取

  for (uint8_t i = 0; i < 4; i++) {
    DelayUs(kCommWaitTime);
    accWrite(acc_init_config[i][0], acc_init_config[i][1]);
    DelayUs(kCheckWaitTime);
    if (accRead(acc_init_config[i][0]) != acc_init_config[i][1]) {
      return kBMI088ErrStateAccConfigErr;
    }
  }

  return kBMI088ErrStateNoErr;
}

/**
 * @brief       陀螺仪配置
 * @param        self_test: 是否进行自检测
 * @retval       None
 * @note        错误状态
 */
BMI088ErrState BMI088::gyroInit(bool self_test) const
{
  DelayUs(kCommWaitTime);
  if (gyroRead(BMI088_GYRO_GYRO_CHIP_ID) != BMI088_GYRO_GYRO_CHIP_ID_VALUE) {
    return kBMI088ErrStateGyroNotFound;
  }

  if (self_test) {
    if (gyroSelfTest() != kBMI088ErrStateNoErr) {
      return kBMI088ErrStateGyroSelfTestFailed;
    }
  }

  // 配置一些寄存器
  uint8_t gyro_bandwidth = config_.gyro_odr_fbw |
                           BMI088_GYRO_GYRO_BANDWIDTH_MUST_SET;
  uint8_t gyro_init_config[3][2] = {
      {BMI088_GYRO_GYRO_RANGE, config_.gyro_range},
      {BMI088_GYRO_GYRO_BANDWIDTH, gyro_bandwidth},
      {BMI088_GYRO_GYRO_LPM1, BMI088_GYRO_GYRO_LPM1_NORMAL}};
  DelayUs(kCommWaitTime);
  gyroWrite(BMI088_GYRO_GYRO_SOFTRESET, BMI088_GYRO_GYRO_SOFTRESET_VALUE);
  HAL_Delay(kGyroResetTime);

  for (uint8_t i = 0; i < 3; i++) {
    DelayUs(kCommWaitTime);
    gyroWrite(gyro_init_config[i][0], gyro_init_config[i][1]);
    DelayUs(kCheckWaitTime);
    if (gyroRead(gyro_init_config[i][0]) != gyro_init_config[i][1]) {
      return kBMI088ErrStateGyroConfigErr;
    }
  }

  return kBMI088ErrStateNoErr;
}

/**
 * @brief       加速度计自检测
 * @retval       错误状态
 * @note        None
 */
BMI088ErrState BMI088::accSelfTest(void) const
{
  uint8_t acc_init_config[4][2] = {
      {BMI088_ACC_ACC_RANGE, BMI088_ACC_ACC_RANGE_ACC_RANGE_24G},
      {BMI088_ACC_ACC_CONF, BMI088_ACC_ACC_CONF_NORMAL |
                                BMI088_ACC_ACC_CONF_ACC_ODR_1600_HZ},
      {BMI088_ACC_ACC_PWR_CTRL, BMI088_ACC_ACC_PWR_CTRL_ON},
      {BMI088_ACC_ACC_PWR_CONF, BMI088_ACC_ACC_PWR_CONF_ACTIVE_MODE}};

  for (uint8_t i = 0; i < 4; i++) {
    DelayUs(kCommWaitTime);
    accWrite(acc_init_config[i][0], acc_init_config[i][1]);
    DelayUs(kCheckWaitTime);
    if (accRead(acc_init_config[i][0]) != acc_init_config[i][1]) {
      return kBMI088ErrStateAccSelfTestFailed;
    }
  }

  HAL_Delay(kAccSelfTestShortDelay);

  float pos_self_test_data[3], neg_self_test_data[3];

  accWrite(BMI088_ACC_ACC_SELF_TEST, BMI088_ACC_ACC_SELF_TEST_POSITIVE_SIGNAL);
  HAL_Delay(kAccSelfTestLongDelay);
  getAccData(pos_self_test_data);

  accWrite(BMI088_ACC_ACC_SELF_TEST, BMI088_ACC_ACC_SELF_TEST_NEGATIVE_SIGNAL);
  HAL_Delay(kAccSelfTestLongDelay);
  getAccData(neg_self_test_data);

  float min_diff_thres[3] = {kGravAcc, kGravAcc, 0.5 * kGravAcc};
  for (uint8_t i = 0; i < 3; i++) {
    if (pos_self_test_data[i] - neg_self_test_data[i] < min_diff_thres[i]) {
      return kBMI088ErrStateAccSelfTestFailed;
    }
  }

  return kBMI088ErrStateNoErr;
}

/**
 * @brief       陀螺仪自检测
 * @retval       错误状态
 * @note        None
 */
BMI088ErrState BMI088::gyroSelfTest(void) const
{
  // 配置一些寄存器
  DelayUs(kCommWaitTime);
  gyroWrite(BMI088_GYRO_GYRO_SELF_TEST, BMI088_GYRO_GYRO_SELF_TEST_TRIG_BIST);

  uint8_t test_result;
  while (1) {
    DelayUs(kCommWaitTime);
    test_result = gyroRead(BMI088_GYRO_GYRO_SELF_TEST);

    if (test_result & BMI088_GYRO_GYRO_SELF_TEST_BIST_RDY) {
      if (test_result & BMI088_GYRO_GYRO_SELF_TEST_BIST_FAIL) {
        return kBMI088ErrStateGyroSelfTestFailed;
      } else {
        return kBMI088ErrStateNoErr;
      }
    }
  }
}

/**
 * @brief       获取陀螺仪数据
 * @param        gyro_data: 陀螺仪三轴数据，[wx wy wz]，单位：rad/s
 * @retval       None
 * @note        None
 */
void BMI088::getGyroData(float gyro_data[3]) const
{
  uint8_t gyro_raw_data[kGyroRawDataLen];
  gyroMultiRead(BMI088_GYRO_RATE_X_LSB, kGyroRawDataLen, gyro_raw_data);

  for (uint8_t i = 0; i < 3; i++) {
    gyro_data[i] = kGyroSens[config_.gyro_range] *
                   static_cast<int16_t>(
                       gyro_raw_data[2 * i] | gyro_raw_data[2 * i + 1] << 8);
  }
}

/**
 * @brief       获取加速度计数据
 * @param        acc_data: 加速度计三轴数据，[ax ay az]，单位：m/s^2
 * @retval       None
 * @note        None
 */
void BMI088::getAccData(float acc_data[3]) const
{
  uint8_t acc_raw_data[kAccRawDataLen];
  accMultiRead(BMI088_ACC_ACC_X_LSB, kAccRawDataLen, acc_raw_data);

  for (uint8_t i = 0; i < 3; i++) {
    acc_data[i] = kAccSens[config_.acc_range] *
                  static_cast<int16_t>(
                      acc_raw_data[2 * i] | acc_raw_data[2 * i + 1] << 8);
  }
}

/**
 * @brief       获取温度数据
 * @retval       温度，单位：℃
 * @note        None
 */
float BMI088::getTemp(void) const
{
  uint8_t temp_raw_data[kTempRawDataLen];
  accMultiRead(BMI088_ACC_TEMP_MSB, kTempRawDataLen, temp_raw_data);

  int16_t temp_raw =
      static_cast<uint16_t>((temp_raw_data[0] << _BMI088_ACC_TEMP_MSB_SHIFTS) |
                            (temp_raw_data[1] >> _BMI088_ACC_TEMP_LSB_SHIFTS));

  if (temp_raw > 1023) {
    temp_raw -= 2048;
  }

  return temp_raw * kTempSens + kTempOffset;
}

/**
 * @brief       向陀螺仪寄存器写入数据
 * @param        mem_addr: 寄存器地址
 * @param        value: 待写入数据
 * @retval       None
 * @note        None
 */
void BMI088::gyroWrite(uint8_t mem_addr, uint8_t value) const
{
  uint8_t tx_data[2] = {mem_addr |= kWriteSign, value};
  gyroCsL();
  HAL_SPI_Transmit(kHspi_, tx_data, 2, kSpiTimeout);
  gyroCsH();
}

/**
 * @brief       读取陀螺仪寄存器数据
 * @param        mem_addr: 寄存器地址
 * @retval       读取数据
 * @note        None
 */
uint8_t BMI088::gyroRead(uint8_t mem_addr) const
{
  uint8_t rx_data[2], tx_data[2] = {mem_addr |= kReadSign, kPaddingValue};
  gyroCsL();
  HAL_SPI_TransmitReceive(kHspi_, tx_data, rx_data, 2, kSpiTimeout);
  gyroCsH();

  return rx_data[1];
}

/**
 * @brief       陀螺仪多寄存器数据读取
 * @param        start_mem_addr: 寄存器起始地址
 * @param        len: 带读取数据长度
 * @param        rx_data: 读取得到的数据
 * @retval       None
 * @note        None
 */
void BMI088::gyroMultiRead(
    uint8_t start_mem_addr, uint8_t len, uint8_t rx_data[]) const
{
  gyroCsL();
  start_mem_addr |= kReadSign;
  HAL_SPI_Transmit(kHspi_, &start_mem_addr, 1, kSpiTimeout);
  HAL_SPI_TransmitReceive(kHspi_, kPaddingData, rx_data, len, kSpiTimeout);
  gyroCsH();
}

/**
 * @brief       向加速度计寄存器写入数据
 * @param        mem_addr: 寄存器地址
 * @param        value: 待写入数据
 * @retval       None
 * @note        None
 */
void BMI088::accWrite(uint8_t mem_addr, uint8_t value) const
{
  uint8_t pTxData[2] = {mem_addr |= kWriteSign, value};
  accCsL();
  HAL_SPI_Transmit(kHspi_, pTxData, 2, kSpiTimeout);
  accCsH();
}

/**
 * @brief       读取加速度计寄存器数据
 * @param        mem_addr: 寄存器地址
 * @retval       读取数据
 * @note        None
 */
uint8_t BMI088::accRead(uint8_t mem_addr) const
{
  uint8_t rx_data[3], tx_data[3] = {
                          mem_addr |= kReadSign, kPaddingValue, kPaddingValue};
  accCsL();
  HAL_SPI_TransmitReceive(kHspi_, tx_data, rx_data, 3, kSpiTimeout);
  accCsH();

  return rx_data[2];
}

/**
 * @brief       加速度计多寄存器数据读取
 * @param        start_mem_addr: 寄存器起始地址
 * @param        len: 带读取数据长度
 * @param        rx_data: 读取得到的数据
 * @retval       None
 * @note        None
 */
void BMI088::accMultiRead(
    uint8_t start_mem_addr, uint8_t len, uint8_t rx_data[]) const
{
  uint8_t tx_data[2] = {start_mem_addr |= kReadSign, kPaddingValue};

  accCsL();
  HAL_SPI_TransmitReceive(kHspi_, tx_data, rx_data, 2, kSpiTimeout);
  HAL_SPI_TransmitReceive(kHspi_, kPaddingData, rx_data, len, kSpiTimeout);
  accCsH();
}

/**
 * @brief       微秒级延时
 * @param        us: 需要延时的时间，单位：us
 * @retval       None
 * @note        None
 */
static void DelayUs(uint16_t us)
{
  uint32_t reload = SysTick->LOAD;
  uint32_t ticks = us * (SystemCoreClock / 1e6f);
  uint32_t t_last = SysTick->VAL;

  uint32_t t_now = 0;
  uint32_t t_cnt = 0;

  while (t_cnt < ticks) {
    t_now = SysTick->VAL;
    if (t_now != t_last) {
      if (t_now < t_last) {
        t_cnt += t_last - t_now;
      } else {
        t_cnt += reload - t_now + t_last;
      }
      t_last = t_now;
    }
  }
}
}  // namespace imu
}  // namespace hello_world
