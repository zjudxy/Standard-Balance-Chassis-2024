/** 
 * @file      rfr_pkg_0x020D_robot_sentry_decision.hpp
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
#ifndef HW_COMPONENTS_DEVICES_REFEREE_PKG_RFR_PKG_0X020D_ROBOT_SENTRY_DECISION_HPP_
#define HW_COMPONENTS_DEVICES_REFEREE_PKG_RFR_PKG_0X020D_ROBOT_SENTRY_DECISION_HPP_

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
 * @struct RobotSentryDecisionData
 * @brief 储存哨兵机器人的兑换信息
 */
struct __REFEREE_PACKED RobotSentryDecisionData {
  uint32_t allowance : 11;        ///< 除远程兑换外，哨兵成功兑换的发弹量
  uint32_t remote_allowance : 4;  ///< 哨兵成功远程兑换发弹量的次数
  uint32_t remote_hp : 4;         ///< 哨兵成功远程兑换血量的次数
  uint32_t reserved : 13;         ///< 保留位
};
/** @class RobotSentryDecisionPackage
 * @brief 哨兵自主决策信息同步
 * 
 * 数据说明：
 * - 命令码：0x020D
 * - 数据长度：4
 * - 发送频率：1Hz
 * - 发送方/接收方：服务器->哨兵机器人
 * - 所属数据链路：常规链路
 * @see RobotSentryDecisionData
 */
class RobotSentryDecisionPackage : public ProtocolRxPackage
{
 public:
  typedef RobotSentryDecisionData Data;

  virtual CmdId getCmdId() const override { return 0x020D; }
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

#endif /* HW_COMPONENTS_DEVICES_REFEREE_PKG_RFR_PKG_0X020D_ROBOT_SENTRY_DECISION_HPP_ */
