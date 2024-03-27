

/**
 * @file      referee.hpp
 * @brief
 * @author    ZhouShichan (zsc19823382069@163.com)
 * @version   1.0.0
 * @date      2023-10-28
 *
 * @copyright Copyright (c) 2023 Hello World Team, Zhejiang University. All Rights Reserved.
 *
 * @attention
 *
 * @par history
 * | Version | Date | Author | Description |
 * | :---: | :---: | :---: | :---: |
 * | 1.0.0 | 2023-10-28 | ZhouShichan | description |
 * @par last editor  ZhouShichan (zsc19823382069@163.com)
 * @par last edit time  2023-10-28
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_CPP_DEVICES_REFEREE_H_
#define HW_COMPONENTS_CPP_DEVICES_REFEREE_H_

/* Includes ------------------------------------------------------------------*/
#include <cstddef>

#include "referee_custom_define.hpp"
#include "referee_protocol.hpp"
#include "referee_rx_template.hpp"
namespace hello_world
{
namespace devices
{

namespace referee
{
/* Exported macro ------------------------------------------------------------*/
const size_t kRefereeMaxBuffer = 0xFF;
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

class Referee
{
   public:
    typedef enum {
        kRxStatusWaitingHeaderSof = 0u,
        kRxStatusHeader,
        kRxStatusCmdDataTail,
    } RxStatus;

    RxPackage<CompetitionStatus> competition_status_ = RxPackage<CompetitionStatus>(kFrameCmdIdCompetitionStatus, kFrameFrequenceCompetitionStatus);
    RxPackage<CompetitionResult> competition_result_ = RxPackage<CompetitionResult>(kFrameCmdIdCompetitionResult, kFrameFrequenceCompetitionResult);
    RxPackage<CompetitionRobotsHp> competition_robots_hp_ = RxPackage<CompetitionRobotsHp>(kFrameCmdIdCompetitionRobotsHp, kFrameFrequenceCompetitionRobotsHp);
    RxPackage<TeamEvent> team_event_ = RxPackage<TeamEvent>(kFrameCmdIdTeamEvent, kFrameFrequenceTeamEvent);
    RxPackage<TeamSupplierAction> team_supplier_action_ = RxPackage<TeamSupplierAction>(kFrameCmdIdTeamSupplierAction, kFrameFrequenceTeamSupplierAction);
    RxPackage<TeamRefereeWarning> team_referee_warning_ = RxPackage<TeamRefereeWarning>(kFrameCmdIdTeamRefereeWarning, kFrameFrequenceTeamRefereeWarning);
    RxPackage<RobotDartStatus> robot_dart_status_ = RxPackage<RobotDartStatus>(kFrameCmdIdRobotDartStatus, kFrameFrequenceRobotDartStatus);
    RxPackage<RobotPerformance> robot_performance_ = RxPackage<RobotPerformance>(kFrameCmdIdRobotPerformance, kFrameFrequenceRobotPerformance);
    RxPackage<RobotPowerHeat> robot_power_heat_ = RxPackage<RobotPowerHeat>(kFrameCmdIdRobotPowerHeat, kFrameFrequenceRobotPowerHeat);
    RxPackage<RobotPosition> robot_position_ = RxPackage<RobotPosition>(kFrameCmdIdRobotPosition, kFrameFrequenceRobotPosition);
    RxPackage<RobotBuff> robot_buff_ = RxPackage<RobotBuff>(kFrameCmdIdRobotBuff, kFrameFrequenceRobotBuff);
    RxPackage<RobotAerialStatus> robot_aerial_status_ = RxPackage<RobotAerialStatus>(kFrameCmdIdRobotAerialStatus, kFrameFrequenceRobotAerialStatus);
    RxPackage<RobotDamage> robot_damage_ = RxPackage<RobotDamage>(kFrameCmdIdRobotDamage, kFrameFrequenceRobotDamage);
    RxPackage<RobotShootData> robot_shoot_data_ = RxPackage<RobotShootData>(kFrameCmdIdRobotShootData, kFrameFrequenceRobotShootData);
    RxPackage<RobotProjectileAllowance> robot_projectile_allowance_ = RxPackage<RobotProjectileAllowance>(kFrameCmdIdRobotProjectileAllowance, kFrameFrequenceRobotProjectileAllowance);
    RxPackage<RobotRfid> robot_rfid_ = RxPackage<RobotRfid>(kFrameCmdIdRobotRfid, kFrameFrequenceRobotRfid);
    RxPackage<RobotDartClientCommand> robot_dart_client_command_ = RxPackage<RobotDartClientCommand>(kFrameCmdIdRobotDartClientCommand, kFrameFrequenceRobotDartClientCommand);
    RxPackage<RobotsGroundPosition> robots_ground_position_ = RxPackage<RobotsGroundPosition>(kFrameCmdIdRobotsGroundPosition, kFrameFrequenceRobotsGroundPosition);
    RxPackage<RobotRadarMarkProgress> robot_radar_mark_progress_ = RxPackage<RobotRadarMarkProgress>(kFrameCmdIdRobotRadarMarkProgress, kFrameFrequenceRobotRadarMarkProgress);
    RxPackage<InterCustomCtrllerToRobot> inter_custom_controller_to_robot_ = RxPackage<InterCustomCtrllerToRobot>(kFrameCmdIdInterCustomCtrllerToRobot, kFrameFrequenceInterCustomCtrllerToRobot);
    RxPackage<InterClientMapToRobots> inter_client_map_to_robots_ = RxPackage<InterClientMapToRobots>(kFrameCmdIdInterClientMapToRobots, kFrameFrequenceInterClientMapToRobots);
    RxPackage<InterCtrllerToRobot> inter_controller_to_robot_ = RxPackage<InterCtrllerToRobot>(kFrameCmdIdInterCtrllerToRobot, kFrameFrequenceInterCtrllerToRobot);
    RxPackageInterRadarWarning inter_radar_warning_ = RxPackageInterRadarWarning();

    bool decodeFrame(const uint8_t* p_frame, const size_t _frame_length);
    bool processByte(const uint8_t byte);
    void restartDecodeFrame(size_t frame_length = kRefereeMaxBuffer);
    template <typename T>
    bool encodeFrame(const T& data, const FrameCmdId frame_cmd_id, const uint16_t sub_content_id, const uint16_t receiver_id, uint8_t* p_frame, size_t& frame_length);

   private:
    const static size_t kRefereeRxPackageNumber = 23;
    RxPackageInfo* const kRefereeRxPackagePtrs[kRefereeRxPackageNumber] = {
        &competition_status_, &competition_result_, &competition_robots_hp_,  // competition package *3
        &team_event_, &team_supplier_action_, &team_referee_warning_,         // team package *3
        &robot_dart_status_, &robot_performance_, &robot_power_heat_, &robot_position_, &robot_buff_,
        &robot_aerial_status_, &robot_damage_, &robot_shoot_data_, &robot_projectile_allowance_, &robot_rfid_,
        &robot_dart_client_command_, &robots_ground_position_, &robot_radar_mark_progress_,             // robot package * 13
        &inter_custom_controller_to_robot_, &inter_client_map_to_robots_, &inter_controller_to_robot_,  // interactive package *3
        &inter_radar_warning_                                                                           // custom interactive package *1
    };
    size_t last_updated_frame_index_ = 0;

    struct {
        union {
            struct __REFEREE_PACKED {
                FrameHeader frame_header;
                FrameCmdId frame_cmd_id;
            } header;
            uint8_t frame_raw[kRefereeMaxBuffer];
        };

        RxStatus rx_status = kRxStatusWaitingHeaderSof;
        size_t frame_data_index = 0;
        size_t frame_expect_length = 0;
    } rx_info_;  // for debug

    union {
        struct __REFEREE_PACKED {
            FrameHeader frame_header;
            FrameCmdId frame_cmd_id;
            InterFrameHeader inter_frame_header;
        } header;
        uint8_t frame_raw[kRefereeMaxBuffer];
    } tx_info_;

    bool decodeRxPackage(const uint8_t* p_frame_cmd_data);
    bool encodeSetCrc(uint8_t* p_frame, size_t frame_length);
};

template <typename T>
bool Referee::encodeFrame(const T& data, const FrameCmdId frame_cmd_id, const uint16_t sub_content_id, const uint16_t receiver_id, uint8_t* p_frame, size_t& frame_length)
{
    bool process_result = false;
    static_assert(sizeof(T) + kFramePartLengthExceptData <= kRefereeMaxBuffer, "Frame data size is too large.");
    size_t data_index = kFramePartIndexData;
    tx_info_.header.frame_header.sof = kRefereeFrameHeaderSof;
    tx_info_.header.frame_header.data_length = sizeof(T);
    tx_info_.header.frame_header.sequence_number++;
    tx_info_.header.frame_cmd_id = frame_cmd_id;
    if (frame_cmd_id == kFrameCmdIdInterAmongRobots) {
        static_assert(sizeof(T) + kFramePartLengthExceptData + sizeof(InterFrameHeader) <= kRefereeMaxBuffer, "InterAmongRobots Frame data size is too large.");
        tx_info_.header.frame_header.data_length += sizeof(InterFrameHeader);
        data_index += sizeof(InterFrameHeader);
        tx_info_.header.inter_frame_header.sub_content_id = sub_content_id;
        tx_info_.header.inter_frame_header.sender_id = robot_performance_.data().robot_id;
        if (sub_content_id == kUiSubCmdIdDeleteLayer || sub_content_id == kUiSubCmdIdDrawCharacter || sub_content_id == kUiSubCmdIdDrawGraphic1 || sub_content_id == kUiSubCmdIdDrawGraphic2 || sub_content_id == kUiSubCmdIdDrawGraphic5 || sub_content_id == kUiSubCmdIdDrawGraphic7) {
            tx_info_.header.inter_frame_header.receiver_id = tx_info_.header.inter_frame_header.sender_id | 0x0100;
        } else {
            tx_info_.header.inter_frame_header.receiver_id = receiver_id;
        }
    }
    memcpy(&tx_info_.frame_raw[data_index], &data, sizeof(T));
    size_t _frame_length = data_index + sizeof(T) + sizeof(Crc16_t);
    process_result = encodeSetCrc(tx_info_.frame_raw, _frame_length);
    if (process_result) {
        frame_length = _frame_length;
        memcpy(p_frame, tx_info_.frame_raw, frame_length);
    };

    return process_result;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace referee

}  // namespace devices
}  // namespace hello_world

#endif /* HW_COMPONENTS_CPP_DEVICES_REFEREE_H_ */
