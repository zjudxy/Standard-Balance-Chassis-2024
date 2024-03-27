

/**
 * @file      referee_size_check.hpp
 * @brief
 * @author    ZhouShichan (zsc19823382069@163.com)
 * @version   1.0.0
 * @date      2023-11-11
 *
 * @copyright Copyright (c) 2023 Hello World Team, Zhejiang University. All Rights Reserved.
 *
 * @attention
 *
 * @par history
 * | Version | Date | Author | Description |
 * | :---: | :---: | :---: | :---: |
 * | 1.0.0 | {now_year} | ZhouShichan | description |
 * @par last editor  ZhouShichan (zsc19823382069@163.com)
 * @par last edit time  2023-11-11
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONETS_REFEREE_SIZE_CHECK_H_
#define HW_COMPONETS_REFEREE_SIZE_CHECK_H_

/* Includes ------------------------------------------------------------------*/
#include "referee_custom_define.hpp"
#include "referee_protocol.hpp"

namespace hello_world
{
namespace devices
{
namespace referee
{
static_assert(sizeof(CompetitionStatus) == kFrameDataLengthCompetitionStatus, "CompetitionStatus size error");
static_assert(sizeof(CompetitionResult) == kFrameDataLengthCompetitionResult, "CompetitionResult size error");
static_assert(sizeof(CompetitionRobotsHp) == kFrameDataLengthCompetitionRobotsHp, "CompetitionRobotsHp size error");
static_assert(sizeof(TeamEvent) == kFrameDataLengthTeamEvent, "TeamEvent size error");
static_assert(sizeof(TeamSupplierAction) == kFrameDataLengthTeamSupplierAction, "TeamSupplierAction size error");
static_assert(sizeof(TeamRefereeWarning) == kFrameDataLengthTeamRefereeWarning, "TeamRefereeWarning size error");
static_assert(sizeof(RobotDartStatus) == kFrameDataLengthRobotDartStatus, "RobotDartStatus size error");
static_assert(sizeof(RobotPerformance) == kFrameDataLengthRobotPerformance, "RobotPerformance size error");
static_assert(sizeof(RobotPowerHeat) == kFrameDataLengthRobotPowerHeat, "RobotPowerHeat size error");
static_assert(sizeof(RobotPosition) == kFrameDataLengthRobotPosition, "RobotPosition size error");
static_assert(sizeof(RobotBuff) == kFrameDataLengthRobotBuff, "RobotBuff size error");
static_assert(sizeof(RobotAerialStatus) == kFrameDataLengthRobotAerialStatus, "RobotAerialStatus size error");
static_assert(sizeof(RobotDamage) == kFrameDataLengthRobotDamage, "RobotDamage size error");
static_assert(sizeof(RobotShootData) == kFrameDataLengthRobotShootData, "RobotShootData size error");
static_assert(sizeof(RobotProjectileAllowance) == kFrameDataLengthRobotProjectileAllowance, "RobotProjectileAllowance size error");
static_assert(sizeof(RobotRfid) == kFrameDataLengthRobotRfid, "RobotRfid size error");
static_assert(sizeof(RobotDartClientCommand) == kFrameDataLengthRobotDartClientCommand, "RobotDartClientCommand size error");
static_assert(sizeof(RobotsGroundPosition) == kFrameDataLengthRobotsGroundPosition, "RobotsGroundPosition size error");
static_assert(sizeof(RobotRadarMarkProgress) == kFrameDataLengthRobotRadarMarkProgress, "RobotRadarMarkProgress size error");
// static_assert(sizeof(InterAmongRobots) == kFrameDataLengthInterAmongRobots, "InterAmongRobots size error");
static_assert(sizeof(InterCustomCtrllerToRobot) == kFrameDataLengthInterCustomCtrllerToRobot, "InterCustomCtrllerToRobot size error");
static_assert(sizeof(InterClientMapToRobots) == kFrameDataLengthInterClientMapToRobots, "InterClientMapToRobots size error");
static_assert(sizeof(InterCtrllerToRobot) == kFrameDataLengthInterCtrllerToRobot, "InterCtrllerToRobot size error");
static_assert(sizeof(InterRadarMapToClient) == kFrameDataLengthInterRadarMapToClient, "InterRadarMapToClient size error");
static_assert(sizeof(InterCustomCtrllerToClient) == kFrameDataLengthInterCustomCtrllerToClient, "InterCustomCtrllerToClient size error");
static_assert(sizeof(InterSentryTrajectoryToClient) == kFrameDataLengthInterSentryTrajectoryToClient, "InterSentryTrajectoryToClient size error");

static_assert(sizeof(InterRadarWarning) == kInterAmongRobotsDataLengthRadarWarning, "InterRadarWarning size error");
static_assert(sizeof(InterRadarWarning) <= kFrameDataLengthInterAmongRobots, "InterRadarWarning size error");
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace referee

}  // namespace devices

}  // namespace hello_world

#endif /* HW_COMPONETS_REFEREE_SIZE_CHECK_H_ */
