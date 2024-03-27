/** 
 * @file      rfr_pkg_0x0307_inter_robot_traj.hpp
 * @author    ZhouShichan (zsc19823382069@163.com)
 * @date      2024-02-18
 * @brief     
 * @par last editor  ZhouShichan (zsc19823382069@163.com)
 * @version   1.0.0
 * 
 * @copyright Copyright (c) 2024 Hello World Team, Zhejiang University. All Rights Reserved.
 * 
 * @note 
 * 
 * @par history
 * | Version | Date | Author | Description |
 * | :---: | :---: | :---: | :---: |
 * | 1.0.0 | 2024-MM-DD | ZhouShichan | 首次完成 |
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_REFEREE_PKG_RFR_PKG_0X0307_INTER_ROBOT_TRAJ_HPP_
#define HW_COMPONENTS_DEVICES_REFEREE_PKG_RFR_PKG_0X0307_INTER_ROBOT_TRAJ_HPP_

/* Includes ------------------------------------------------------------------*/
#include "rfr_pkg_core.hpp"
/* Exported macro ------------------------------------------------------------*/
namespace hello_world
{
namespace referee
{
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/** 
 * @enum RobotIntention
 * @brief 机器人意图
 */
enum RobotIntention : uint8_t {
  kRobotIntentionAttack = 1,  ///< 到目标点攻击
  kRobotIntentionDefend = 2,  ///< 到目标点防守
  kRobotIntentionMove = 3,    ///< 移动到目标点
};

/** 
 * @struct InterRobotTrajData
 * @brief 选手端小地图接收哨兵机器人的路径数据
 */
struct __REFEREE_PACKED InterRobotTrajData {
  /** 
   * 机器人意图
   * 
   * - 1: 到目标点攻击
   * - 2: 到目标点防守
   * - 3: 移动到目标点
   */
  uint8_t intention;
  /** 
   * 路径起点 x 轴坐标，单位：dm
   * 
   * @note 小地图左下角为坐标原点，水平向右为 X 轴正方向，竖直向上为 Y 轴正方向。显示位置将按照场地尺寸与小地图尺寸等比缩放，超出边界的位置将在边界处显示
   */
  uint16_t start_position_x;
  /** 
   * 路径起点 y 轴坐标，单位：dm
   * 
   * @note 小地图左下角为坐标原点，水平向右为 X 轴正方向，竖直向上为 Y 轴正方向。显示位置将按照场地尺寸与小地图尺寸等比缩放，超出边界的位置将在边界处显示
   */
  uint16_t start_position_y;
  /** 
   * 路径点 x 轴增量数组，单位：dm
   * 
   * @note 增量相较于上一个点位进行计算，共 49 个新点位，X 与 Y 轴增量对应组成点位
   */
  int8_t delta_x[49];
  /** 
   * 路径点 y 轴增量数组，单位：dm
   * 
   * @note 增量相较于上一个点位进行计算，共 49 个新点位，X 与 Y 轴增量对应组成点位
   */
  int8_t delta_y[49];
  /** 
   * 发送者 ID
   * 
   * @note 需与自身 ID 匹配，ID 编号详见附录
   */
  uint16_t sender_id;
};

/** 
 * @class InterRobotTrajPackage
 * @brief 选手端小地图接收哨兵或选择半自动控制方式的机器人的路径数据
 * 
 * 哨兵机器人或选择半自动控制方式的机器人可通过常规链路向对应的操作手选手端发送路径坐标数据，该路径会在小地图上显示。
 * 
 * 数据说明：
 * 
 * - 命令码：0x0307
 * - 数据长度：105
 * - 发送频率：频率上限为 1Hz
 * - 发送方/接收方：哨兵/半自动控制机器人->对应操作手选手端
 * - 所属数据链路：常规链路
 * 
 * @attention 在《串口协议v1.6.1> P7 表2-1 中，0x0307 的 数据段长度 字段为 103 ，与 P31 表3-3 中的 大小 字段和结构体的定义不一致，此处以结构体大小为准。
 */
class InterRobotTrajPackage : public ProtocolTxPackage
{
 public:
  typedef InterRobotTrajData Data;

  virtual CmdId getCmdId() const override { return 0x0307; };
  virtual DataLength getDataLength() const override { return sizeof(Data); };
  virtual uint32_t getMinTxIntervalMs() const override { return FREQ2INTERVAL(1); };

  const Data &getData() const { return data_; }

  bool encode(uint8_t *data) override
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

#endif /* HW_COMPONENTS_DEVICES_REFEREE_PKG_RFR_PKG_0X0307_INTER_ROBOT_TRAJ_HPP_ */
