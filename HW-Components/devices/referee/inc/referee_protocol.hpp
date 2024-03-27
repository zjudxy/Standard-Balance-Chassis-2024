

/**
 * @file      referee_rx_data.hpp
 * @brief
 * @author    ZhouShichan (zsc19823382069@163.com)
 * @version   1.0.0
 * @date      2023-10-29
 *
 * @copyright Copyright (c) 2023 Hello World Team, Zhejiang University. All Rights Reserved.
 *
 * @attention
 *
 * @par history
 * | Version | Date | Author | Description |
 * | :---: | :---: | :---: | :---: |
 * | 1.0.0 | 2023-10-29 | ZhouShichan | description |
 * @par last editor  ZhouShichan (zsc19823382069@163.com)
 * @par last edit time  2023-10-29
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HWCOMPONETS_DEVICES_REFEREE_PROTOCOL_H_
#define HWCOMPONETS_DEVICES_REFEREE_PROTOCOL_H_

/* Includes ------------------------------------------------------------------*/
#include <cstddef>
#include <cstdint>

#ifndef __REFEREE_PACKED
#define __REFEREE_PACKED __attribute__((packed))
#endif

namespace hello_world
{
namespace devices
{

namespace referee
{

/* Exported macro ------------------------------------------------------------*/
const uint8_t kRefereeFrameHeaderSof = 0xA5;

/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

typedef enum : uint16_t {
    kFrameCmdIdNone = 0x0000,
    kFrameCmdIdCompetitionStatus = 0x0001,
    kFrameCmdIdCompetitionResult = 0x0002,
    kFrameCmdIdCompetitionRobotsHp = 0x003,
    kFrameCmdIdTeamEvent = 0x0101,
    kFrameCmdIdTeamSupplierAction = 0x0102,
    kFrameCmdIdTeamRefereeWarning = 0x0104,
    kFrameCmdIdRobotDartStatus = 0x0105,
    kFrameCmdIdRobotPerformance = 0x0201,
    kFrameCmdIdRobotPowerHeat = 0x0202,
    kFrameCmdIdRobotPosition = 0x0203,
    kFrameCmdIdRobotBuff = 0x0204,
    kFrameCmdIdRobotAerialStatus = 0x0205,
    kFrameCmdIdRobotDamage = 0x0206,
    kFrameCmdIdRobotShootData = 0x0207,
    kFrameCmdIdRobotProjectileAllowance = 0x0208,
    kFrameCmdIdRobotRfid = 0x0209,
    kFrameCmdIdRobotDartClientCommand = 0x020A,
    kFrameCmdIdRobotsGroundPosition = 0x020B,
    kFrameCmdIdRobotRadarMarkProgress = 0x020C,
    kFrameCmdIdInterAmongRobots = 0x0301,
    kFrameCmdIdInterCustomCtrllerToRobot = 0x0302,
    kFrameCmdIdInterClientMapToRobots = 0x0303,
    kFrameCmdIdInterCtrllerToRobot = 0x0304,
    kFrameCmdIdInterRadarMapToClient = 0x0305,
    kFrameCmdIdInterCustomCtrllerToClient = 0x0306,
    kFrameCmdIdInterSentryTrajectoryToClient = 0x0307,
} FrameCmdId;

typedef enum : size_t {
    kFrameDataLengthCompetitionStatus = 11u,
    kFrameDataLengthCompetitionResult = 1u,
    kFrameDataLengthCompetitionRobotsHp = 32u,
    kFrameDataLengthTeamEvent = 4u,
    kFrameDataLengthTeamSupplierAction = 4u,
    kFrameDataLengthTeamRefereeWarning = 2u,
    kFrameDataLengthRobotDartStatus = 1u,
    kFrameDataLengthRobotPerformance = 27u,
    kFrameDataLengthRobotPowerHeat = 16u,
    kFrameDataLengthRobotPosition = 16u,
    kFrameDataLengthRobotBuff = 5u,
    kFrameDataLengthRobotAerialStatus = 2u,
    kFrameDataLengthRobotDamage = 1u,
    kFrameDataLengthRobotShootData = 7u,
    kFrameDataLengthRobotProjectileAllowance = 6u,
    kFrameDataLengthRobotRfid = 4u,
    kFrameDataLengthRobotDartClientCommand = 6u,
    kFrameDataLengthRobotsGroundPosition = 40u,
    kFrameDataLengthRobotRadarMarkProgress = 6u,
    kFrameDataLengthInterAmongRobots = 128u,
    kFrameDataLengthInterCustomCtrllerToRobot = 30u,
    kFrameDataLengthInterClientMapToRobots = 15u,
    kFrameDataLengthInterCtrllerToRobot = 12u,
    kFrameDataLengthInterRadarMapToClient = 10u,
    kFrameDataLengthInterCustomCtrllerToClient = 8u,
    kFrameDataLengthInterSentryTrajectoryToClient = 103u,
} FrameDataLength;

typedef enum {
    kFrameFrequenceCompetitionStatus = 3u,
    kFrameFrequenceCompetitionResult = 0u,
    kFrameFrequenceCompetitionRobotsHp = 3u,
    kFrameFrequenceTeamEvent = 2u,
    kFrameFrequenceTeamSupplierAction = 0u,
    kFrameFrequenceTeamRefereeWarning = 0u,
    kFrameFrequenceRobotDartStatus = 3u,
    kFrameFrequenceRobotPerformance = 10u,
    kFrameFrequenceRobotPowerHeat = 50u,
    kFrameFrequenceRobotPosition = 10u,
    kFrameFrequenceRobotBuff = 3u,
    kFrameFrequenceRobotAerialStatus = 10u,
    kFrameFrequenceRobotDamage = 0u,
    kFrameFrequenceRobotShootData = 0u,
    kFrameFrequenceRobotProjectileAllowance = 10u,
    kFrameFrequenceRobotRfid = 3u,
    kFrameFrequenceRobotDartClientCommand = 10u,
    kFrameFrequenceRobotsGroundPosition = 1u,
    kFrameFrequenceRobotRadarMarkProgress = 1u,
    kFrameFrequenceInterAmongRobots = 10u,
    kFrameFrequenceInterCustomCtrllerToRobot = 30u,
    kFrameFrequenceInterClientMapToRobots = 0u,
    kFrameFrequenceInterCtrllerToRobot = 30u,
    kFrameFrequenceInterRadarMapToClient = 10u,
    kFrameFrequenceInterCustomCtrllerToClient = 30u,
    kFrameFrequenceInterSentryTrajectoryToClient = 1u,
} FrameFrequence;

typedef struct __REFEREE_PACKED {
    uint8_t sof;
    uint16_t data_length;
    uint8_t sequence_number;
    uint8_t crc8;
} FrameHeader;

typedef enum {
    kCompetitionTypeRmuc = 1u,
    kCompetitionTypeRmutc = 2u,
    kCompetitionTypeIcra = 3u,
    kCompetitionTypeRmul3v3 = 4u,
    kCompetitionTypeRmul1v1 = 5u,
} CompetitionType;

typedef enum {
    kCompetitionStagePrecompetition = 0u,
    kCompetitionStageSetup = 1u,
    kCompetitionStageInitialization = 2u,
    kCompetitionStage5SecondCountdown = 3u,
    kCompetitionStageCompetitionOngoing = 4u,
    kCompetitionStageCalculatingResults = 5u
} CompetitionStage;

typedef struct __REFEREE_PACKED {
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t sync_time_stamp;
} CompetitionStatus;


typedef enum {
    kCompetitionResultDraw = 0u,
    kCompetitionResultRedTeamWins = 1u,
    kCompetitionResultBlueTeamWins = 2u,
} CompetitionResults;

typedef struct __REFEREE_PACKED {
    uint8_t result;
} CompetitionResult;

typedef struct __REFEREE_PACKED {
    uint16_t red_1_robot_HP;  /**< RED 1 Hero HP. If the robot has not entered the stage or has been ejected, the HP is zero. */
    uint16_t red_2_robot_HP;  /**< RED 2 Engineer HP. If the robot has not entered the stage or has been ejected, the HP is zero. */
    uint16_t red_3_robot_HP;  /**< RED 3 Standard HP. If the robot has not entered the stage or has been ejected, the HP is zero. */
    uint16_t red_4_robot_HP;  /**< RED 4 Standard HP. If the robot has not entered the stage or has been ejected, the HP is zero. */
    uint16_t red_5_robot_HP;  /**< RED 5 Standard HP. If the robot has not entered the stage or has been ejected, the HP is zero. */
    uint16_t red_7_robot_HP;  /**< RED 7 Sentry HP. If the robot has not entered the stage or has been ejected, the HP is zero. */
    uint16_t red_outpost_HP;  /**< Red Outpost HP. If the robot has not entered the stage or has been ejected, the HP is zero. */
    uint16_t red_base_HP;     /**< Red Base HP. If the robot has not entered the stage or has been ejected, the HP is zero. */
    uint16_t blue_1_robot_HP; /**< Blue 1 Hero HP. If the robot has not entered the stage or has been ejected, the HP is zero. */
    uint16_t blue_2_robot_HP; /**< Blue 2 Engineer HP. If the robot has not entered the stage or has been ejected, the HP is zero. */
    uint16_t blue_3_robot_HP; /**< Blue 3 Standard HP. If the robot has not entered the stage or has been ejected, the HP is zero. */
    uint16_t blue_4_robot_HP; /**< Blue 4 Standard HP. If the robot has not entered the stage or has been ejected, the HP is zero. */
    uint16_t blue_5_robot_HP; /**< Blue 5 Standard HP. If the robot has not entered the stage or has been ejected, the HP is zero. */
    uint16_t blue_7_robot_HP; /**< Blue 7 Sentry HP. If the robot has not entered the stage or has been ejected, the HP is zero. */
    uint16_t blue_outpost_HP; /**< Blue Outpost HP. If the robot has not entered the stage or has been ejected, the HP is zero. */
    uint16_t blue_base_HP;    /**< Blue Base HP. If the robot has not entered the stage or has been ejected, the HP is zero. */
} CompetitionRobotsHp;

typedef struct __REFEREE_PACKED {
    uint8_t front_restoration_is_occupied : 1;           /**< Occupation status of the Restoration Zone in front of own Official Projectile Supplier; 1 means it is occupied. */
    uint8_t left_restoration_is_occupied : 1;            /**< Occupation status of the Restoration Zone to the left of own Official Projectile Supplier (facing the Restoration Zone); 1 means it is occupied. */
    uint8_t right_restoration_is_occupied : 1;           /**< Occupation status of the Restoration Zone to the right of own Official Projectile Supplier (facing the Restoration Zone); 1 means it is occupied. */
    uint8_t power_rune_activation_point_is_occupied : 1; /**< Occupation status of own Power Rune Activation Point; 1 means it is occupied. */
    uint8_t small_power_rune_is_activated : 1;           /**< Activation status of own Small Power Rune; 1 means it is activated. */
    uint8_t large_power_rune_is_activated : 1;           /**< Activation status of own Large Power Rune; 1 means it is activated. */
    uint8_t ring_highland_is_occupied : 1;               /**< Occupation status of own Ring-Shaped Elevated Ground; 1 means it is occupied. */
    uint8_t r3b3_trapezoidal_highland_is_occupied : 1;   /**< Occupation status of own R3 Trapezoid-Shaped Elevated Ground; 1 means it is occupied. */
    uint8_t r4b4_trapezoidal_highland_is_occupied : 1;   /**< Occupation status of own R4 Trapezoidal Elevated Ground; 1 means it is occupied. */
    uint16_t base_virtual_shield : 8;                    /**< Value of own Base's Virtual Shield (0 - 250) */
    uint16_t outposet_hp : 11;                           /**< HP of own Outpost (0 - 1500) */
    uint8_t sentry_is_in_patrol_zone : 1;                /**< Whether the Sentry is in own Patrol Zone */
    uint8_t reserved_bits : 3;                           /**< Reserved */
} TeamEvent;

typedef enum {
    kSupplierIdLeft = 1u,  /**< Official Projectile Supplier Outlet to the left of own side (facing the Official Projectile Supplier) */
    kSupplierIdRight = 2u, /**< Official Projectile Supplier Outlet to the right of own side (facing the Official Projectile Supplier) */
} SupplierId;

typedef enum SupplingRobotIds {
    kSupplingRobotIdNone = 0u,            /**< No robot is reloading projectiles */
    kSupplingRobotIdRedHero = 1u,         /**< Red Hero Robot is reloading projectiles */
    kSupplingRobotIdRedStandard3 = 3u,    /**< Red Standard Robot 3 is reloading projectiles */
    kSupplingRobotIdRedStandard4 = 4u,    /**< Red Standard Robot 4 is reloading projectiles */
    kSupplingRobotIdRedStandard5 = 5u,    /**< Red Standard Robot 5 is reloading projectiles */
    kSupplingRobotIdBlueHero = 101u,      /**< Blue Hero Robot is reloading projectiles */
    kSupplingRobotIdBlueStandard3 = 103u, /**< Blue Standard Robot 3 is reloading projectiles */
    kSupplingRobotIdBlueStandard4 = 104u, /**< Blue Standard Robot 4 is reloading projectiles */
    kSupplingRobotIdBlueStandard5 = 105u, /**< Blue Standard Robot 5 is reloading projectiles */
} SupplingRobotId;

typedef enum {
    kSupplierStepClosed = 0u,    /**< Status of the projectile outlet is closed. */
    kSupplierStepPreparing = 1u, /**< Status of the projectile outlet is preparing projectiles. */
    kSupplierStepReleasing = 2u, /**< Status of the projectile outlet is releasing projectiles. */
} SupplierStep;

typedef enum {
    kSuppliedNumber50 = 50u,   /**< Number of supplied projectiles is 50 */
    kSuppliedNumber100 = 100u, /**< Number of supplied projectiles is 100 */
    kSuppliedNumber150 = 150u, /**< Number of supplied projectiles is 150 */
    kSuppliedNumber200 = 200u, /**< Number of supplied projectiles is 200 */
} SuppliedNumber;

typedef struct __REFEREE_PACKED {
    uint8_t supply_projectile_id;   /**< Official Projectile Supplier Outlet ID */
    uint8_t supply_robot_id;        /**< Reloading robot ID */
    uint8_t supply_projectile_step; /**< Status of the projectile outlet */
    uint8_t supply_projectile_num;  /**< Number of supplied projectiles */
} TeamSupplierAction;

typedef enum {
    kPenaltyLevelNone = 0u,
    kPenaltyLevelYellowCard = 1u,
    kPenaltyLevelRedCard = 2u,
    kPenaltyLevelForfeiture = 3u,
} PenaltyLevel;

typedef struct __REFEREE_PACKED {
    uint8_t level;
    uint8_t offending_robot_id; /**< In the case of a forfeiture or where both teams have been issued a Yellow Card, the value is 0.*/
} TeamRefereeWarning;

typedef struct __REFEREE_PACKED {
    uint8_t dart_remaining_time;
} RobotDartStatus;

typedef struct __REFEREE_PACKED {
    uint8_t robot_id;                          /**< Current robot ID */
    uint8_t robot_level;                       /**< Robot level */
    uint16_t current_hp;                       /**< Robot's current HP */
    uint16_t maximum_hp;                       /**< Robot maximum HP */
    uint16_t shooter_cooling_value_17mm1;      /**< Robot 1 17mm barrel cooling value per second */
    uint16_t shooter_barrel_heat_limit_17mm1;  /**< Robot 1 17mm barrel heat limit */
    uint16_t shooter_bullet_speed_limit_17mm1; /**< Robot 1 17mm Launching Mechanism’s Initial Launch Speed Limit (unit: m/s) */
    uint16_t shooter_cooling_value_17mm2;      /**< Robot 2 17mm barrel cooling value per second */
    uint16_t shooter_barrel_heat_limit_17mm2;  /**< Robot 2 17mm barrel heat limit */
    uint16_t shooter_bullet_speed_limit_17mm2; /**< Robot 2 17mm Launching Mechanism’s Initial Launch Speed Limit (unit: m/s) */
    uint16_t shooter_cooling_value_42mm;       /**< Robot 1 42mm barrel cooling value per second */
    uint16_t shooter_barrel_heat_limit_42mm;   /**< Robot 42mm barrel heat limit */
    uint16_t shooter_bullet_speed_limit_42mm;  /**< Robot 1 42mm Launching Mechanism’s Initial Launch Speed Limit (unit: m/s) */
    uint16_t chassis_power_limit;              /**< Robot Chassis Power Consumption Limit */
    uint8_t gimbal_port_is_output : 1;         /**< Output from gimbal port: 0 denotes zero output, 1 denotes 24V output */
    uint8_t chassis_port_is_output : 1;        /**< Output from chassis port: 0 denotes zero output, 1 denotes 24V output */
    uint8_t shooter_port_is_output : 1;        /**< Output from shooter port: 0 denotes zero output, 1 denotes 24V output */
} RobotPerformance;

typedef struct __REFEREE_PACKED {
    uint16_t chassis_voltage;           /**< Power Management Module chassis port output voltage (unit: mV) */
    uint16_t chassis_current;           /**< Power Management Module chassis port output current (unit: mA) */
    float chassis_power;                /**< Chassis Power (unit: W) */
    uint16_t chassis_buffer;            /**< Buffer Energy (unit: J) */
    uint16_t shooter_barrel_heat_17mm1; /**< Barrel heat of the 1st 17mm Launching Mechanism */
    uint16_t shooter_barrel_heat_17mm2; /**< Barrel heat of the 2nd Launching Mechanism */
    uint16_t shooter_barrel_heat_42mm2; /**< Barrel heat of the 42mm Launching Mechanism */
} RobotPowerHeat;

typedef struct __REFEREE_PACKED {
    float x;     /**< The x-coordinate of this robot’s position; unit: m  */
    float y;     /**< The y-coordinate of this robot’s position; unit: m  */
    float z;     /**< The z-coordinate of this robot’s position; unit: m  */
    float angle; /**< Direction of this robot’s Speed Monitor Module; unit: degree. True north is 0 degrees. */
} RobotPosition;

typedef struct __REFEREE_PACKED {
    uint8_t recovery_buff;      /**< HP Recovery Buff (in percentage; a value of 10 means the maximum HP recovery per second is 10%) */
    uint8_t cooling_value_buff; /**< Robot barrel cooling rate (in absolute value; a value of 5 means a cooling rate of 5 times) */
    uint8_t defence_buff;       /**< Robot defense buff (in percentage; a value of 50 means a defense buff of 50%) */
    uint16_t attack_buff;       /**< Robot attack buff (in percentage; a value of 50 means an attack buff of 50%) */
} RobotBuff;

typedef enum {
    kAirforceStatusCooling = 0u, /**< Aerial robot status is cooling */
    kAirforceStatusReady = 1u,   /**< Cooling of Aerial robot is completed */
    kAirforceStatusOnging = 2u,  /**< Air support is ongoing */
} AirforceStatus;

typedef struct __REFEREE_PACKED {
    uint8_t airforce_status; /**< Aerial robot status */
    uint8_t time_remain;     /**< Remaining time of this status (unit is "s", rounded down to the nearest integer, and when the remaining cooling time is 1.9s, the value is 1) */
} RobotAerialStatus;

typedef enum {
    kHpDeductionReasonArmorHit = 0u,         /**< HP deduction due to Armor being hit by projectiles */
    kHpDeductionReasonModuleOffline,         /**< HP deduction due to the Critical Module Going Offline Referee System Modules going offline */
    kHpDeductionReasonExceedingBulletSpeed,  /**< HP deduction due to exceeding the Initial Launching Speed Limit */
    kHpDeductionReasonExceedingBarrelHeat,   /**< HP deduction due to exceeding Barrel Heat Limit */
    kHpDeductionReasonExceedingChassisPower, /**< HP deduction due to exceeding Chassis Power Consumption Limit */
    kHpDeductionReasonArmorCollision,        /**< HP deduction due to the Armor Module suffering a collision */
} HpDeductionReason;

typedef struct __REFEREE_PACKED {
    uint8_t module_id : 4; /**< When HP deduction is due to an Armor Module or Speed Monitor Module, the 4-bit value is the ID of the Armor Module or Speed Monitor Module; if it is due to other reasons, the value is 0. */
    uint8_t hp_deduction_reason : 4;
} RobotDamage;

typedef struct __REFEREE_PACKED {
    uint8_t bullet_type;
    uint8_t shooter_id;

    uint8_t launching_frequency; /**< Projectile launch frequency(unit: Hz) */
    float bullet_speed;          /**< Projectile initial speed (unit: m/s) */
} RobotShootData;

typedef struct __REFEREE_PACKED {
    uint16_t projectile_allowance_17mm; /**< 17mm projectile allowance. */
    uint16_t projectile_allowance_42mm; /**< 42mm projectile allowance. */
    uint16_t remaining_gold_coin;       /**< Number of remaining gold coins. */
} RobotProjectileAllowance;

typedef struct __REFEREE_PACKED {
    uint8_t own_base_buff_point_is_detected : 1;                  /**< Bit 0: Own Base Buff Point */
    uint8_t own_ring_highland_buff_point_is_detected : 1;         /**< Bit 1: Own Elevated Ground Buff Point */
    uint8_t opp_ring_highland_buff_point_is_detected : 1;         /**< Bit 2: Opponent's Elevated Ground Buff Point */
    uint8_t own_r3b3_trapezoidal_highland_is_detected : 1;        /**< Bit 3: Own R3/B3 Trapezoid-Shaped Elevated Ground */
    uint8_t opp_r3b3_trapezoidal_highland_is_detected : 1;        /**< Bit 4: Opponent's R3/B3 Trapezoid-Shaped Elevated Ground */
    uint8_t own_r4b4_trapezoidal_highland_is_detected : 1;        /**< Bit 5: Own R4/B4 Trapezoid-Shaped Elevated Ground */
    uint8_t opp_r4b4_trapezoidal_highland_is_detected : 1;        /**< Bit 6: Opponent's R4/B4 Trapezoid-Shaped Elevated Ground */
    uint8_t own_power_rune_activation_point_is_detected : 1;      /**< Bit 7: Own Power Rune Activation Point */
    uint8_t own_front_launch_ramp_buff_point_is_detected : 1;     /**< Bit 8: Own Launch Ramp Buff Point (in front of the Launch Ramp near own side) */
    uint8_t own_behind_launch_ramp_buff_point_is_detected : 1;    /**< Bit 9: Own Launch Ramp Buff Point (behind the Launch Ramp near own side) */
    uint8_t opp_front_launch_ramp_buff_point_is_detected : 1;     /**< Bit 10: Opponent's Launch Ramp Buff Point (in front of the Launch Ramp near the other side) */
    uint8_t opp_behind_launch_ramp_buff_point_is_detected : 1;    /**< Bit 11: Opponent's Launch Ramp Buff Point (behind the Launch Ramp near the other side) */
    uint8_t own_outpost_buff_point_is_detected : 1;               /**< Bit 12: Own Outpost Buff Point */
    uint8_t own_restoration_zone_is_detected : 1;                 /**< Bit 13: Own Restoration Zone (deemed activated if any one is detected) */
    uint8_t own_sentry_patrol_zones_is_detected : 1;              /**< Bit 14: Own Sentry Patrol Zones */
    uint8_t opp_sentry_patrol_zones_is_detected : 1;              /**< Bit 15: Opponent's Sentry Patrol Zones */
    uint8_t own_large_resource_island_buff_point_is_detected : 1; /**< Bit 16: Own Large Resource Island Buff Point */
    uint8_t opp_large_resource_island_buff_point_is_detected : 1; /**< Bit 17: Opponent's Large Resource Island Buff Point */
    uint8_t own_controlled_zones_is_detected : 1;                 /**< Bit 18: Own Controlled Zones */
    uint8_t opp_controlled_zones_is_detected : 1;                 /**< Bit 19: Opponent's Controlled Zones */
    uint16_t reserved : 12;                                       /**< Bits 20-31: Reserved */
} RobotRfid;

typedef enum {
    kDartStationStatusClosed = 1u, /**< The Dart Station is closed. */
    kDartStationStatusMoving,      /**< The Dart Station is in the process of opening or closing. */
    kDartStationStatusOpened       /**< The Dart Station is fully opened. */
} DartStationStatus;

typedef enum {
    kDartTargetOutpost = 0u, /**< Dart target is outpost (Outpost as default). */
    kDartTargetBase          /**< Dart target is the base. */
} DartTarget;

typedef struct __REFEREE_PACKED {
    uint8_t dart_launch_opening_status; /**< Current status of the Dart Launching Station. */
    uint8_t dart_attack_target;         /**< Selected Dart target: Outpost or Base. */
    uint16_t target_change_time;        /**< Time remaining in the competition when switching targets (unit: s). Defaults to 0 if no target switching is involved. */
    uint16_t latest_launch_cmd_time;    /**< Time remaining in the competition when the Operator confirms the launch command for the last time (unit: s). Initial value is 0. */
} RobotDartClientCommand;

typedef struct __REFEREE_PACKED {
    float hero_x;       /**< The x-axis coordinate of the Hero Robot; unit: meters (m). */
    float hero_y;       /**< The y-axis coordinate of the Hero Robot; unit: meters (m). */
    float engineer_x;   /**< The x-axis coordinate of the Engineer Robot; unit: meters (m). */
    float engineer_y;   /**< The y-axis coordinate of the Engineer Robot; unit: meters (m). */
    float standard_3_x; /**< The x-axis coordinate of Standard Robot No. 3; unit: meters (m). */
    float standard_3_y; /**< The y-axis coordinate of Standard Robot No. 3; unit: meters (m). */
    float standard_4_x; /**< The x-axis coordinate of Standard Robot No. 4; unit: meters (m). */
    float standard_4_y; /**< The y-axis coordinate of Standard Robot No. 4; unit: meters (m). */
    float standard_5_x; /**< The x-axis coordinate of Standard Robot No. 5; unit: meters (m). */
    float standard_5_y; /**< The y-axis coordinate of Standard Robot No. 5; unit: meters (m). */
} RobotsGroundPosition;

typedef struct __REFEREE_PACKED {
    uint8_t mark_hero_progress;       /**< Marked progress of opponent's Hero Robot: 0-120. */
    uint8_t mark_engineer_progress;   /**< Marked progress of opponent's Engineer Robot: 0-120. */
    uint8_t mark_standard_3_progress; /**< Marked progress of opponent's Standard Robot No. 3: 0-120. */
    uint8_t mark_standard_4_progress; /**< Marked progress of opponent's Standard Robot No. 4: 0-120. */
    uint8_t mark_standard_5_progress; /**< Marked progress of opponent's Standard Robot No. 5: 0-120. */
    uint8_t mark_sentry_progress;     /**< Marked progress of opponent's Sentry Robot: 0-120. */
} RobotRadarMarkProgress;

typedef enum : uint16_t {
    kRobotIdRedHero = 1,
    kRobotIdRedEngineer = 2,
    kRobotIdRedStandard3 = 3,
    kRobotIdRedStandard4 = 4,
    kRobotIdRedStandard5 = 5,
    kRobotIdRedAerial = 6,
    kRobotIdRedSentry = 7,
    kRobotIdRedDart = 8,
    kRobotIdRedRadar = 9,
    kRobotIdRedOutpost = 10,
    kRobotIdRedBase = 11,
    kRobotIdBlueHero = 101,
    kRobotIdBlueEngineer = 102,
    kRobotIdBlueStandard3 = 103,
    kRobotIdBlueStandard4 = 104,
    kRobotIdBlueStandard5 = 105,
    kRobotIdBlueAerial = 106,
    kRobotIdBlueSentry = 107,
    kRobotIdBlueDart = 108,
    kRobotIdBlueRadar = 109,
    kRobotIdBlueOutpost = 110,
    kRobotIdBlueBase = 111
} RobotId;

typedef enum : uint16_t {
    kClientIdRedHero = 0x0101,
    kClientIdRedEngineer = 0x0102,
    kClientIdRedStandard3 = 0x0103,
    kClientIdRedStandard4 = 0x0104,
    kClientIdRedStandard5 = 0x0105,
    kClientIdRedAerial = 0x0106,
    kClientIdBlueHero = 0x0165,
    kClientIdBlueEngineer = 0x0166,
    kClientIdBlueStandard3 = 0x0167,
    kClientIdBlueStandard4 = 0x0168,
    kClientIdBlueStandard5 = 0x0169,
    kClientIdAerial = 0x016A
} ClientId;

typedef enum : uint16_t {
    kUiSubCmdIdDeleteLayer = 0x0100,
    kUiSubCmdIdDrawGraphic1,
    kUiSubCmdIdDrawGraphic2,
    kUiSubCmdIdDrawGraphic5,
    kUiSubCmdIdDrawGraphic7,
    kUiSubCmdIdDrawCharacter,
} UiSubCmdId;

typedef struct __REFEREE_PACKED {
    uint16_t sub_content_id;
    uint16_t sender_id;
    uint16_t receiver_id;
} InterFrameHeader;

typedef enum {
    kDeleteLayerOperationNone = 0u,
    kDeleteLayerOperationSingle,
    kDeleteLayerOperationAll,
} DeleteLayerOperation;

typedef enum {
    kLayerNumber0 = 0,
    kLayerNumber1,
    kLayerNumber2,
    kLayerNumber3,
    kLayerNumber4,
    kLayerNumber5,
    kLayerNumber6,
    kLayerNumber7,
    kLayerNumber8,
    kLayerNumber9,
} LayerNumber;

typedef struct __REFEREE_PACKED {
    uint8_t delete_layer_operation;
    uint8_t layer;
} InterDeleteLayer;

typedef enum : uint8_t {
    kGraphicOperationNone = 0,
    kGraphicOperationAdd,
    kGraphicOperationRevise,
    kGraphicOperationDelete,
} GraphicOperation;

typedef enum : uint8_t {
    kGraphicTypeStraightLine = 0,
    kGraphicTypeRectangle,
    kGraphicTypeCircle,
    kGraphicTypeEllipse,
    kGraphicTypeSArc,
    kGraphicTypeFloatingNumber,
    kGraphicTypeInteger,
    kGraphicTypeCharacter,
} GraphicType;

typedef enum : uint8_t {
    kGraphicColorRedBlue = 0,
    kGraphicColorYellow,
    kGraphicColorGreen,
    kGraphicColorOrange,
    kGraphicColorPurplish,
    kGraphicColorPink,
    kGraphicColorCyan,
    kGraphicColorBlack,
    kGraphicColorWhite,
} GraphicColor;

typedef struct __REFEREE_PACKED {
    uint8_t graphic_name[3];
    uint32_t operate_tpye : 3;
    uint32_t graphic_tpye : 3;
    uint32_t layer : 4;
    uint32_t color : 4;
    uint32_t details_a : 9;
    uint32_t details_b : 9;
    uint32_t line_width : 10;
    uint32_t start_x : 11;
    uint32_t start_y : 11;
    uint32_t details_c : 10;
    uint32_t details_d : 11;
    uint32_t details_e : 11;
} InterGraphic;

typedef struct __REFEREE_PACKED {
    InterGraphic interaction_graphic;
} InterGraphicPack1;

typedef struct __REFEREE_PACKED {
    InterGraphic interaction_graphic[2];
} InterGraphicPack2;

typedef struct __REFEREE_PACKED {
    InterGraphic interaction_graphic[5];
} InterGraphicPack5;

typedef struct __REFEREE_PACKED {
    InterGraphic interaction_graphic[7];
} InterGraphicPack7;

typedef struct __REFEREE_PACKED {
    InterGraphic interaction_graphic;
    uint8_t characters[30];
} InterGraphicPackCharacter;

typedef struct __REFEREE_PACKED {
    uint8_t data[30];
} InterCustomCtrllerToRobot;  // 0x302

typedef struct __REFEREE_PACKED {
    float target_position_x;
    float target_position_y;
    float target_position_z;
    uint8_t command_keyboard;
    uint16_t target_robot_id;
} InterClientMapToRobots;  // 0x303

typedef struct __REFEREE_PACKED {
    uint8_t w : 1;
    uint8_t s : 1;
    uint8_t a : 1;
    uint8_t d : 1;
    uint8_t shift : 1;
    uint8_t ctrl : 1;
    uint8_t q : 1;
    uint8_t e : 1;
    uint8_t r : 1;
    uint8_t f : 1;
    uint8_t g : 1;
    uint8_t z : 1;
    uint8_t x : 1;
    uint8_t c : 1;
    uint8_t v : 1;
    uint8_t b : 1;
} MainCtrllerKeyboardData;

typedef union {
    uint16_t keyboard_value;
    MainCtrllerKeyboardData key;
} MainCtrllerKeyboard;

typedef struct __REFEREE_PACKED {
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    int8_t left_button_down;
    int8_t right_button_down;
    MainCtrllerKeyboard keyboard;
    uint16_t reserved;
} InterCtrllerToRobot;  // 0x304

typedef struct __REFEREE_PACKED {
    uint16_t target_robot_id;
    float target_position_x;
    float target_position_y;
} InterRadarMapToClient;  // 0x305

typedef struct __REFEREE_PACKED {
    uint16_t key_value;
    uint16_t x_position : 12;
    uint16_t mouse_left : 4;
    uint16_t y_position : 12;
    uint16_t mouse_right : 4;
    uint16_t reserved;
} InterCustomCtrllerToClient;  // 0x306

typedef struct __REFEREE_PACKED {
    uint8_t intention;
    int16_t start_position_x;
    uint16_t start_position_y;
    int8_t delta_x[49];
    int8_t delta_y[49];
} InterSentryTrajectoryToClient;  // 0x307

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace referee

}  // namespace devices

}  // namespace hello_world

#endif /* HWCOMPONETS_DEVICES_REFEREE_PROTOCOL_H_ */
