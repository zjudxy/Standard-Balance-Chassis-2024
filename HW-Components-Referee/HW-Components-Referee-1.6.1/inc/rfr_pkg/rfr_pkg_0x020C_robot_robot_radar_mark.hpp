/** 
 * @file      rfr_pkg_0x020C_robot_robot_radar_mark.hpp
 * @author    ZhouShichan (zsc19823382069@163.com)
 * @date      2024-01-25
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
 * | 1.0.0 | 2024-02-18 | ZhouShichan | 首次完成 |
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_REFEREE_PKG_RFR_PKG_0X020C_ROBOT_ROBOT_RADAR_MARK_HPP_
#define HW_COMPONENTS_DEVICES_REFEREE_PKG_RFR_PKG_0X020C_ROBOT_ROBOT_RADAR_MARK_HPP_

/* Includes ------------------------------------------------------------------*/
#include "rfr_pkg_core.hpp"
/* Exported constants --------------------------------------------------------*/
namespace hello_world
{
namespace referee
{
/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/**
 * @struct RobotRadarMarkData
 * @brief 雷达机器人标记进度数据
 */
struct __REFEREE_PACKED RobotRadarMarkData {
  uint8_t hero;        ///< 对方英雄机器人被标记进度：0—120
  uint8_t engineer;    ///< 对方工程机器人被标记进度：0—120
  uint8_t standard_3;  ///< 对方 3 号步兵机器人被标记进度：0—120
  uint8_t standard_4;  ///< 对方 4 号步兵机器人被标记进度：0—120
  uint8_t standard_5;  ///< 对方 5 号步兵机器人被标记进度：0—120
  uint8_t sentry;      ///< 对方哨兵机器人被标记进度：0—120
};

/** @class RobotRadarMarkPackage
 * @brief 雷达机器人标记进度数据包
 * 
 * 数据说明：
 * - 命令码：0x020C
 * - 数据长度：6
 * - 发送频率：1Hz
 * - 发送方/接收方：服务器->雷达机器人
 * - 所属数据链路：常规链路
 */
class RobotRadarMarkPackage : public ProtocolRxPackage
{
 public:
  typedef RobotRadarMarkData Data;

  virtual CmdId getCmdId() const override { return 0x020C; }
  virtual DataLength getDataLength() const override { return sizeof(Data); }
  virtual uint32_t getMaxRxIntervalMs() const override { return FREQ2INTERVAL(1); }

  const Data &getData() const { return data_; }

  virtual bool decode(const CmdId &cmd_id, const uint8_t *data_ptr) override
  {
    if (cmd_id == getCmdId()) {
      memcpy(&data_, data_ptr, sizeof(Data));
      last_decode_tick_ = getNowTickMs();
      is_handled_ = false;
      return true;
    }
    return false;
  };

 private:
  Data data_ = {0};
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace referee
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_REFEREE_PKG_RFR_PKG_0X020C_ROBOT_ROBOT_RADAR_MARK_HPP_ */
