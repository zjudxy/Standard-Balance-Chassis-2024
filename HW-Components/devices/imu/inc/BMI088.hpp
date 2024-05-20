/**
 *******************************************************************************
 * @file      : BMI088.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_IMU_BMI088_HPP_
#define HW_COMPONENTS_DEVICES_IMU_BMI088_HPP_

/* Includes ------------------------------------------------------------------*/
#include "allocator.hpp"
#include "arm_math.h"
#include "stm32_hal.hpp"

namespace hello_world
{
namespace imu
{
/* Exported macro ------------------------------------------------------------*/

enum BMI088ErrState : uint8_t {  ///* 通过或运算拼接
  kBMI088ErrStateNoErr = 0u,
  kBMI088ErrStateAccNotFound = 1u << 0,
  kBMI088ErrStateGyroNotFound = 1u << 1,
  kBMI088ErrStateAccSelfTestFailed = 1u << 2,
  kBMI088ErrStateGyroSelfTestFailed = 1u << 3,
  kBMI088ErrStateAccConfigErr = 1u << 4,
  kBMI088ErrStateGyroConfigErr = 1u << 5,
};

enum BMI088AccRange : uint8_t {  ///* BMI088 加速度计量程，单位：g
  kBMI088AccRange3G = 0x0,
  kBMI088AccRange6G = 0x1,
  kBMI088AccRange12G = 0x2,
  kBMI088AccRange24G = 0x3,
};

/** BMI088 加速度计输出频率（Output Data Rate），单位：Hz */
enum BMI088AccOdr : uint8_t {
  kBMI088AccOdr12_5 = 0x5,
  kBMI088AccOdr25 = 0x6,
  kBMI088AccOdr50 = 0x7,
  kBMI088AccOdr100 = 0x8,
  kBMI088AccOdr200 = 0x9,
  kBMI088AccOdr400 = 0xA,
  kBMI088AccOdr800 = 0xB,
  kBMI088AccOdr1600 = 0xC,
};

enum BMI088AccOsr : uint8_t {  ///* BMI088 加速度计过采样率（Oversampling Rate）
  kBMI088AccOsr4 = 0x8,        ///* 4倍过采样
  kBMI088AccOsr2 = 0x9,        ///* 2倍过采样
  kBMI088AccOsrNormal = 0xA,   ///* 不过采样
};

enum BMI088GyroRange : uint8_t {  ///* BMI088 陀螺仪量程，单位：°/s
  kBMI088GyroRange2000Dps = 0x0,
  kBMI088GyroRange1000Dps = 0x1,
  kBMI088GyroRange500Dps = 0x2,
  kBMI088GyroRange250Dps = 0x3,
  kBMI088GyroRange125Dps = 0x4,
};

/** BMI088 陀螺仪输出频率（Output Data Rate）与滤波器带宽（Filter Bandwidth），单位：Hz */
enum BMI088GyroOdrFbw : uint8_t {
  kBMI088GyroOdrFbw2000_532 = 0x0,
  kBMI088GyroOdrFbw2000_230 = 0x1,
  kBMI088GyroOdrFbw1000_116 = 0x2,
  kBMI088GyroOdrFbw400_47 = 0x3,
  kBMI088GyroOdrFbw200_23 = 0x4,
  kBMI088GyroOdrFbw100_12 = 0x5,
  kBMI088GyroOdrFbw200_64 = 0x6,
  kBMI088GyroOdrFbw100_32 = 0x7,
};
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

struct BMI088HWConfig {        ///* BMI088 硬件配置
  SPI_HandleTypeDef* hspi;     ///* IMU 对应的 SPI 句柄的指针
  GPIO_TypeDef* acc_cs_port;   ///* 加速度计片选端口
  uint32_t acc_cs_pin;         ///* 加速度计片选引脚
  GPIO_TypeDef* gyro_cs_port;  ///* 陀螺仪片选端口
  uint32_t gyro_cs_pin;        ///* 陀螺仪片选引脚
};

struct BMI088Config {  ///* BMI088 设备配置
  BMI088AccRange acc_range = kBMI088AccRange3G;
  BMI088AccOdr acc_odr = kBMI088AccOdr1600;
  BMI088AccOsr acc_osr = kBMI088AccOsr4;
  BMI088GyroRange gyro_range = kBMI088GyroRange1000Dps;
  BMI088GyroOdrFbw gyro_odr_fbw = kBMI088GyroOdrFbw1000_116;
};

class BMI088 : public MemMang
{
 public:
  BMI088(const BMI088HWConfig& hw_config, const float rot_mat_flatten[9],
         const BMI088Config& config = BMI088Config());
  virtual ~BMI088() {}

  BMI088ErrState init(bool self_test = false) const;

  void getData(float acc_data[3], float gyro_data[3], float* temp) const;

  SPI_HandleTypeDef* const kHspi_;   ///* IMU 对应的 SPI 句柄的指针
  GPIO_TypeDef* const kAccCsPort_;   ///* 加速度计片选端口
  const uint32_t kAccCsPin_;         ///* 加速度计片选引脚
  GPIO_TypeDef* const kGyroCsPort_;  ///* 陀螺仪片选端口
  const uint32_t kGyroCsPin_;        ///* 陀螺仪片选引脚
 private:
  inline void accCsL(void) const
  {
    HAL_GPIO_WritePin(kAccCsPort_, kAccCsPin_, GPIO_PIN_RESET);
  }
  inline void accCsH(void) const
  {
    HAL_GPIO_WritePin(kAccCsPort_, kAccCsPin_, GPIO_PIN_SET);
  }
  inline void gyroCsL(void) const
  {
    HAL_GPIO_WritePin(kGyroCsPort_, kGyroCsPin_, GPIO_PIN_RESET);
  }
  inline void gyroCsH(void) const
  {
    HAL_GPIO_WritePin(kGyroCsPort_, kGyroCsPin_, GPIO_PIN_SET);
  }

  BMI088ErrState accInit(bool self_test) const;

  BMI088ErrState gyroInit(bool self_test) const;

  BMI088ErrState accSelfTest(void) const;

  BMI088ErrState gyroSelfTest(void) const;

  void getGyroData(float gyro_data[3]) const;

  void getAccData(float acc_data[3]) const;

  float getTemp(void) const;

  void gyroWrite(uint8_t mem_addr, uint8_t value) const;
  uint8_t gyroRead(uint8_t mem_addr) const;
  void gyroMultiRead(
      uint8_t start_mem_addr, uint8_t len, uint8_t rx_data[]) const;

  void accWrite(uint8_t mem_addr, uint8_t value) const;
  uint8_t accRead(uint8_t mem_addr) const;
  void accMultiRead(
      uint8_t start_mem_addr, uint8_t len, uint8_t rx_data[]) const;

  BMI088Config config_;
  float rot_mat_flatten_[9];
  arm_matrix_instance_f32 rot_mat_;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace imu
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_IMU_BMI088_HPP_ */
