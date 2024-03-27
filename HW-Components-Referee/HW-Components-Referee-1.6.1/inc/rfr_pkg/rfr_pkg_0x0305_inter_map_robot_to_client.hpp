/** 
 * @file      rfr_pkg_0x0305_inter_map_robot_to_client.hpp
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
#ifndef HW_COMPONENTS_DEVICES_REFEREE_PKG_RFR_PKG_0X0305_INTER_MAP_ROBOT_TO_CLIENT_HPP_
#define HW_COMPONENTS_DEVICES_REFEREE_PKG_RFR_PKG_0X0305_INTER_MAP_ROBOT_TO_CLIENT_HPP_

/* Includes ------------------------------------------------------------------*/
#include "rfr_id.hpp"
#include "rfr_pkg_core.hpp"
/* Exported constants --------------------------------------------------------*/
namespace hello_world
{
namespace referee
{
/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/** 
 * @struct InterMapRobotToClientData
 * @brief 选手端小地图可接收机器人数据。
 */
struct __REFEREE_PACKED InterMapRobotToClientData {
  uint16_t target_robot_id;  ///< 目标机器人 ID
  float target_position_x;   ///< 目标 x 位置坐标，单位：m @note 当 x、y 超出边界时则不显示
  float target_position_y;   ///< 目标 y 位置坐标，单位：m @note 当 x、y 超出边界时则不显示
};

/** 
 * @class InterMapRobotToClientPackage
 * @brief 选手端小地图可接收机器人数据
 * 
 * 雷达可通过常规链路向己方所有选手端发送对方机器人的坐标数据，该位置会在己方选手端小地图显示。
 * 
 * 数据说明：
 * 
 * - 命令码：0x0305
 * - 数据长度：10
 * - 发送频率：频率上限为 10Hz
 * - 发送方/接收方：雷达->服务器->己方所有选手端
 * - 所属数据链路：常规链路
 */
class InterMapRobotToClientPackage : public ProtocolTxPackage
{
 public:
  typedef InterMapRobotToClientData Data;
  typedef ids::RobotId RobotId;

  virtual CmdId getCmdId() const override { return 0x0305; }
  virtual DataLength getDataLength() const override { return sizeof(Data); }
  virtual uint32_t getMinTxIntervalMs() const override { return FREQ2INTERVAL(10); }

  const Data &getData() const { return data_; };

  bool setTargetRobotId(RobotId id)
  {
    if (ids::IsRobotId(id)) {
      data_.target_robot_id = id;
      return true;
    } else {
      return false;
    }
  };

  bool setTargetPosition(float x, float y)
  {
    data_.target_position_x = x;
    data_.target_position_y = y;
    return true;
  };

  virtual bool encode(uint8_t *data) override
  {
    uint32_t now_tick = getNowTickMs();
    if (now_tick - last_encode_tick_ < getMinTxIntervalMs()) {
      return false;
    }
    memcpy(data, &data_, sizeof(Data));
    last_encode_tick_ = now_tick;
    return true;
  };

 protected:
  Data data_ = {0};
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace referee
}  // namespace hello_world

#endif /* HW_COMPONENTS_DEVICES_REFEREE_PKG_RFR_PKG_0X0305_INTER_MAP_ROBOT_TO_CLIENT_HPP_ */
