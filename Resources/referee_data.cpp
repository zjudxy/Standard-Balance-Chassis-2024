

/**
 * @file      referee_data.cpp
 * @brief     裁判系统数据
 * @author    shlies (wyc050123@gmail.com)
 * @version   1.0.0
 * @date      2024-03-10
 *
 * @copyright Copyright (c) 2024 Hello World Team, Zhejiang University. All Rights Reserved.
 *
 * @attention
 *
 * @par history
 * | Version | Date | Author | Description |
 * | :---: | :---: | :---: | :---: |
 * | 1.0.0 | 2024-MM-DD | shlies | description |
 * @par last editor  shlies (wyc050123@gmail.com)
 * @par last edit time  2024-03-10
 */
/* Includes ------------------------------------------------------------------*/
#include "referee_data.hpp"
#include "chassis.hpp"
/* Private macro -------------------------------------------------------------*/

// 唯一ID
const uint8_t kAutoShootRectangle[3] = {'1', '0', '1'};
const uint8_t kAutoShootTargetRectangle[3] = {'1', '0', '2'};

const uint8_t kSuperCapVerticalLine[3] = {0x02, 0, 0x01};
const uint8_t kSuperCapPercentage[3] = {0x02, 0, 0x02};

const uint8_t kRobotBoundLine[2][3] = {{0x02, 0, 0x03}, {0x02, 0, 0x04}};

const uint8_t kBalance_num_retangle[3] = {0x03, 0, 0x05};

const uint8_t kMiddleVerticalLine[3] = {0x03, 0, 0x01};
const uint8_t kMiddleHorizital1[3] = {0x03, 0, 0x02};
const uint8_t kMiddleHorizital2[3] = {0x03, 0, 0x03};
const uint8_t kMiddleHorizital3[3] = {0x03, 0, 0x04};

const uint8_t kLegPos[4][3] = {{0x04, 0, 0x01}, {0x04, 0, 0x02}, {0x04, 0, 0x03}, {0x04, 0, 0x04}}; // 腿部位置图形 1为左橙 2为右绿 3为连接线

const uint8_t kLegHeightline[5][3] = {0x04, 0, 0x05, 0x04, 0, 0x06, 0x04, 0, 0x07, 0x04, 0, 0x08, 0x04, 0, 0x09}; // 腿部高度基准线
const uint8_t kForwardLine[3] = {0x05, 0, 0x01};                                                                  // 枪口固定线
const uint8_t kLegHeightCurr[2][3] = {0x05, 0, 0x02, 0x05, 0, 0x03};                                              // 腿部高度线
const uint8_t kAttackRetangle[3][3] = {{0x05, 0, 0x04}, {0x05, 0, 0x05}, {0x05, 0, 0x06}};                        // 受击打图形
const uint8_t kAttackArch[2][3] = {{0x05, 0, 0x07}, {0x05, 0, 0x08}};                                             // 受击打图形

const uint8_t kString[6][3] = {0x06, 0, 0x01, 0x06, 0, 0x02, 0x06, 0, 0x03, 0x06, 0, 0x04, 0x06, 0, 0x05, 0x06, 0, 0x06}; // 字符串

/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
const UiInitData default_ui_init_data = {
    .middle_vertical_line = {960, 540, 960, 300},
    .front_7m_height = 500,
    .front_7m_half_length = 15,
    .front_5m_height = 440,
    .front_5m_half_length = 10,
    .front_3m_height = 360,
    .front_3m_half_length = 5,

    .auto_shoot_edge = {700, 350, 1200, 600},
    .auto_shoot_target_Rect_size = {25, 25},

    .robot_bound_line_left = {580, 40, 725, 230},
    .robot_bound_line_right = {1340, 40, 1195, 230},

    .super_cap_vertical_line = {820, 100, 1100, 100},
    .super_cap_horizontal_line_half_length = 4,

    .armer_detect_radius = 80,
    .armer_detect_center = {660, 120},

    .leg_height_line_start = {1150, 50, 1150 + 140, 50 + 150},

    .attack_arch = {960, 540},
    .attack_arch_radius = {150, 150},

    .attack_string_size = 20,
    .base_attack_pos = {600, 780},
    .hero_attack_pos = {700, 780},
    .outpost_attack_pos = {800, 780},

    .mode_string_size = 30,
    .chassis_mode_string_pos = {60, 700},
    .steer_mode_string_pos = {60, 600},
    .balance_num_string_pos = {60, 800},

    .balance_num_pos = {60, 500},
    .minipc_states_pos = {200, 500},

    .refresh_times = 0,
    .hurt_refresh_interval = 0,

};
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
Referee::Referee(UiInitData ui_init_data)
{

    ui_init_data_ = ui_init_data;
}

// 线条旋转
void Rotation(int16_t line[4], int16_t center[2], float angle, int16_t rotated_line[4])
{
    float x1 = line[0] - center[0];
    float y1 = line[1] - center[1];
    float x2 = line[2] - center[0];
    float y2 = line[3] - center[1];
    rotated_line[0] = x1 * cos(angle) - y1 * sin(angle) + center[0];
    rotated_line[1] = x1 * sin(angle) + y1 * cos(angle) + center[1];
    rotated_line[2] = x2 * cos(angle) - y2 * sin(angle) + center[0];
    rotated_line[3] = x2 * sin(angle) + y2 * cos(angle) + center[1];
}
// 超电 准星线 自瞄目标1
void Referee::uiAddComponents1(referee::InterGraphic7Package *pkg)
{
    // 中心准星竖线
    referee::StraightLine middle_vertical_line(kMiddleVerticalLine,
                                               referee::Graphic::Operation::kGraphicAdd,
                                               referee::Graphic::Layer::kGraphicLayer0,
                                               referee::Graphic::Color::kColorWhite);
    middle_vertical_line.setStartPos(ui_init_data_.middle_vertical_line[0], ui_init_data_.middle_vertical_line[1]);
    middle_vertical_line.setEndPos(ui_init_data_.middle_vertical_line[2], ui_init_data_.middle_vertical_line[3]);
    middle_vertical_line.setLineWidth(1);
    // 中心准星横线最上面一条
    referee::StraightLine middle_horizotal1(kMiddleHorizital1,
                                            referee::Graphic::Operation::kGraphicAdd,
                                            referee::Graphic::Layer::kGraphicLayer0,
                                            referee::Graphic::Color::kColorWhite);
    middle_horizotal1.setStartPos(960 - ui_init_data_.front_7m_half_length, ui_init_data_.front_7m_height);
    middle_horizotal1.setEndPos(960 + ui_init_data_.front_7m_half_length, ui_init_data_.front_7m_height);
    middle_horizotal1.setLineWidth(1);
    // 中心准星横线中间一条
    referee::StraightLine middle_horizotal2(kMiddleHorizital2,
                                            referee::Graphic::Operation::kGraphicAdd,
                                            referee::Graphic::Layer::kGraphicLayer0,
                                            referee::Graphic::Color::kColorWhite);
    middle_horizotal2.setStartPos(960 - ui_init_data_.front_5m_half_length, ui_init_data_.front_5m_height);
    middle_horizotal2.setEndPos(960 + ui_init_data_.front_5m_half_length, ui_init_data_.front_5m_height);
    middle_horizotal2.setLineWidth(1);
    // 中心准星横线最下面一条
    referee::StraightLine middle_horizotal3(kMiddleHorizital3,
                                            referee::Graphic::Operation::kGraphicAdd,
                                            referee::Graphic::Layer::kGraphicLayer0,
                                            referee::Graphic::Color::kColorWhite);
    middle_horizotal3.setStartPos(960 - ui_init_data_.front_3m_half_length, ui_init_data_.front_3m_height);
    middle_horizotal3.setEndPos(960 + ui_init_data_.front_3m_half_length, ui_init_data_.front_3m_height);
    middle_horizotal3.setLineWidth(1);
    // 超电百分比框
    referee::Rectangle super_cap_rectangle(kSuperCapVerticalLine,
                                           referee::Graphic::Operation::kGraphicAdd,
                                           referee::Graphic::Layer::kGraphicLayer0,
                                           referee::Graphic::Color::kColorWhite);
    super_cap_rectangle.setStartPos(ui_init_data_.super_cap_vertical_line[0], ui_init_data_.super_cap_vertical_line[1] - 15);
    super_cap_rectangle.setEndPos(ui_init_data_.super_cap_vertical_line[2], ui_init_data_.super_cap_vertical_line[3] + 15);
    super_cap_rectangle.setLineWidth(4);

    // 自瞄目标框
    referee::Rectangle auto_shoot_target_rect(kAutoShootTargetRectangle,
                                              referee::Graphic::Operation::kGraphicAdd,
                                              referee::Graphic::Layer::kGraphicLayer0,
                                              referee::Graphic::Color::kColorWhite);
    auto_shoot_target_rect.setStartPos(ui_init_data_.auto_shoot_target_Rect_size[0], ui_init_data_.auto_shoot_target_Rect_size[1]);
    auto_shoot_target_rect.setEndPos(ui_init_data_.auto_shoot_target_Rect_size[0] + 40, ui_init_data_.auto_shoot_target_Rect_size[1] + 40);
    auto_shoot_target_rect.setLineWidth(0);

    // 超电百分比实际值
    referee::StraightLine super_cap_percentage(kSuperCapPercentage,
                                               referee::Graphic::Operation::kGraphicAdd,
                                               referee::Graphic::Layer::kGraphicLayer1,
                                               referee::Graphic::Color::kColorPurple);

    super_cap_percentage.setStartPos(ui_init_data_.super_cap_vertical_line[0], ui_init_data_.super_cap_vertical_line[1]);
    super_cap_percentage.setEndPos(uint16_t((float)ui_init_data_.super_cap_vertical_line[0] * (1 - ui_components_.super_cap_percentage / 100.0f) + ((float)ui_init_data_.super_cap_vertical_line[2] * ui_components_.super_cap_percentage / 100.0f)),
                                   ui_init_data_.super_cap_vertical_line[3]);
    super_cap_percentage.setLineWidth(25);

    pkg->setStraightLineAt(middle_vertical_line, 0);
    pkg->setStraightLineAt(middle_horizotal1, 1);
    pkg->setStraightLineAt(middle_horizotal2, 2);
    pkg->setStraightLineAt(middle_horizotal3, 3);
    pkg->setRectangleAt(super_cap_rectangle, 4);
    pkg->setRectangleAt(auto_shoot_target_rect, 5);
    pkg->setStraightLineAt(super_cap_percentage, 6);
}

// 车道线
void Referee::uiAddComponents2(referee::InterGraphic7Package *pkg)
{
    referee::StraightLine robot_bound_line_left(kRobotBoundLine[0],
                                                referee::Graphic::Operation::kGraphicAdd,
                                                referee::Graphic::Layer::kGraphicLayer0,
                                                referee::Graphic::Color::kColorOrange);
    robot_bound_line_left.setStartPos(ui_init_data_.robot_bound_line_left[0][0], ui_init_data_.robot_bound_line_left[0][1]);
    robot_bound_line_left.setEndPos(ui_init_data_.robot_bound_line_left[1][0], ui_init_data_.robot_bound_line_left[1][1]);
    robot_bound_line_left.setLineWidth(2);

    referee::StraightLine robot_bound_line_right(kRobotBoundLine[1],
                                                 referee::Graphic::Operation::kGraphicAdd,
                                                 referee::Graphic::Layer::kGraphicLayer0,
                                                 referee::Graphic::Color::kColorOrange);
    robot_bound_line_right.setStartPos(ui_init_data_.robot_bound_line_right[0][0], ui_init_data_.robot_bound_line_right[0][1]);
    robot_bound_line_right.setEndPos(ui_init_data_.robot_bound_line_right[1][0], ui_init_data_.robot_bound_line_right[1][1]);
    robot_bound_line_right.setLineWidth(2);

    pkg->setStraightLineAt(robot_bound_line_left, 0);
    pkg->setStraightLineAt(robot_bound_line_right, 1);
}
// 车身状态
void Referee::uiAddComponents3(referee::InterGraphic7Package *pkg)
{

    // 画腿部位置基圆
    referee::Circle leg_base_circle(kLegPos[0],
                                    referee::Graphic::Operation::kGraphicAdd,
                                    referee::Graphic::Layer::kGraphicLayer0,
                                    referee::Graphic::Color::kColorPurple,
                                    ui_init_data_.armer_detect_center[0], ui_init_data_.armer_detect_center[1],
                                    ui_init_data_.armer_detect_radius, 1);

    // 画双腿位置
    referee::Circle left_leg_circle(kLegPos[1],
                                    referee::Graphic::Operation::kGraphicAdd,
                                    referee::Graphic::Layer::kGraphicLayer0,
                                    referee::Graphic::Color::kColorOrange,
                                    ui_init_data_.armer_detect_center[0] - ui_init_data_.armer_detect_radius, ui_init_data_.armer_detect_center[1],
                                    6, 3);

    referee::Circle right_leg_circle(kLegPos[2],
                                     referee::Graphic::Operation::kGraphicAdd,
                                     referee::Graphic::Layer::kGraphicLayer0,
                                     referee::Graphic::Color::kColorGreen,
                                     ui_init_data_.armer_detect_center[0] + ui_init_data_.armer_detect_radius, ui_init_data_.armer_detect_center[1],
                                     6, 3);

    // 画连接线
    referee::StraightLine leg_link_line(kLegPos[3],
                                        referee::Graphic::Operation::kGraphicAdd,
                                        referee::Graphic::Layer::kGraphicLayer0,
                                        referee::Graphic::Color::kColorCyan);
    leg_link_line.setStartPos(ui_init_data_.armer_detect_center[0] - ui_init_data_.armer_detect_radius, ui_init_data_.armer_detect_center[1]);
    leg_link_line.setEndPos(ui_init_data_.armer_detect_center[0] + ui_init_data_.armer_detect_radius, ui_init_data_.armer_detect_center[1]);
    leg_link_line.setLineWidth(2);

    // 画枪口固定线
    referee::StraightLine forward_line(kForwardLine,
                                       referee::Graphic::Operation::kGraphicAdd,
                                       referee::Graphic::Layer::kGraphicLayer0,
                                       referee::Graphic::Color::kColorPink);
    forward_line.setStartPos(ui_init_data_.armer_detect_center[0], ui_init_data_.armer_detect_center[1]);
    forward_line.setEndPos(ui_init_data_.armer_detect_center[0], ui_init_data_.armer_detect_center[1] + ui_init_data_.armer_detect_radius);
    forward_line.setLineWidth(8);

    // 画腿部高度线

    float start_X[2];
    start_X[0] = (ui_init_data_.leg_height_line_start[1][0] - ui_init_data_.leg_height_line_start[0][0]) / 7 * 2 + ui_init_data_.leg_height_line_start[0][0];
    start_X[1] = (ui_init_data_.leg_height_line_start[1][0] - ui_init_data_.leg_height_line_start[0][0]) / 7 * 5 + ui_init_data_.leg_height_line_start[0][0];
    float line_Width = (ui_init_data_.leg_height_line_start[1][0] - ui_init_data_.leg_height_line_start[0][0]) / 7 * 2;
    float line_len = ui_init_data_.leg_height_line_start[1][1] - ui_init_data_.leg_height_line_start[0][1];

    referee::StraightLine Left_leg_height_curr(kLegHeightCurr[0],
                                               referee::Graphic::Operation::kGraphicAdd,
                                               referee::Graphic::Layer::kGraphicLayer0,
                                               referee::Graphic::Color::kColorOrange);
    Left_leg_height_curr.setStartPos(start_X[0], ui_init_data_.leg_height_line_start[0][1]);
    Left_leg_height_curr.setEndPos(start_X[0], ui_init_data_.leg_height_line_start[0][1] + line_len);
    Left_leg_height_curr.setLineWidth(line_Width);

    referee::StraightLine Right_leg_height_curr(kLegHeightCurr[1],
                                                referee::Graphic::Operation::kGraphicAdd,
                                                referee::Graphic::Layer::kGraphicLayer0,
                                                referee::Graphic::Color::kColorGreen);
    Right_leg_height_curr.setStartPos(start_X[1], ui_init_data_.leg_height_line_start[0][1]);
    Right_leg_height_curr.setEndPos(start_X[1], ui_init_data_.leg_height_line_start[0][1] + line_len);
    Right_leg_height_curr.setLineWidth(line_Width);

    pkg->setCircleAt(leg_base_circle, 0);
    pkg->setCircleAt(left_leg_circle, 1);
    pkg->setCircleAt(right_leg_circle, 2);
    pkg->setStraightLineAt(leg_link_line, 3);
    pkg->setStraightLineAt(forward_line, 4);
    pkg->setStraightLineAt(Left_leg_height_curr, 5);
    pkg->setStraightLineAt(Right_leg_height_curr, 6);
}

// 身体位置和腿长线
void Referee::uiAddComponents4(referee::InterGraphic7Package *pkg)
{

    // 腿长基准线
    float start_x = ui_init_data_.leg_height_line_start[0][0];
    float start_y = ui_init_data_.leg_height_line_start[0][1];
    float end_x = ui_init_data_.leg_height_line_start[1][0];
    float end_y = ui_init_data_.leg_height_line_start[1][1];
    float err_y = (end_y - start_y) / 4.0f;
    referee::StraightLine LegHeightline1(kLegHeightline[0],
                                         referee::Graphic::Operation::kGraphicAdd,
                                         referee::Graphic::Layer::kGraphicLayer0,
                                         referee::Graphic::Color::kColorYellow);
    LegHeightline1.setStartPos(start_x, start_y);
    LegHeightline1.setEndPos(end_x, start_y);
    LegHeightline1.setLineWidth(2);

    referee::StraightLine LegHeightline2(kLegHeightline[1],
                                         referee::Graphic::Operation::kGraphicAdd,
                                         referee::Graphic::Layer::kGraphicLayer0,
                                         referee::Graphic::Color::kColorYellow);
    LegHeightline2.setStartPos(start_x, start_y + err_y);
    LegHeightline2.setEndPos(end_x, start_y + err_y);
    LegHeightline2.setLineWidth(2);

    referee::StraightLine LegHeightline3(kLegHeightline[2],
                                         referee::Graphic::Operation::kGraphicAdd,
                                         referee::Graphic::Layer::kGraphicLayer0,
                                         referee::Graphic::Color::kColorYellow);
    LegHeightline3.setStartPos(start_x, start_y + err_y * 2);
    LegHeightline3.setEndPos(end_x, start_y + err_y * 2);
    LegHeightline3.setLineWidth(2);

    referee::StraightLine LegHeightline4(kLegHeightline[3],
                                         referee::Graphic::Operation::kGraphicAdd,
                                         referee::Graphic::Layer::kGraphicLayer0,
                                         referee::Graphic::Color::kColorYellow);
    LegHeightline4.setStartPos(start_x, start_y + err_y * 3);
    LegHeightline4.setEndPos(end_x, start_y + err_y * 3);
    LegHeightline4.setLineWidth(2);

    referee::StraightLine LegHeightline5(kLegHeightline[4],
                                         referee::Graphic::Operation::kGraphicAdd,
                                         referee::Graphic::Layer::kGraphicLayer0,
                                         referee::Graphic::Color::kColorYellow);
    LegHeightline5.setStartPos(start_x, start_y + err_y * 4);
    LegHeightline5.setEndPos(end_x, start_y + err_y * 4);
    LegHeightline5.setLineWidth(2);

    referee::Rectangle AutoShootRectangle(kAutoShootRectangle,
                                          referee::Graphic::Operation::kGraphicAdd,
                                          referee::Graphic::Layer::kGraphicLayer0,
                                          referee::Graphic::Color::kColorGreen);
    AutoShootRectangle.setStartPos(ui_init_data_.auto_shoot_edge[0], ui_init_data_.auto_shoot_edge[1]);
    AutoShootRectangle.setEndPos(ui_init_data_.auto_shoot_edge[2], ui_init_data_.auto_shoot_edge[3]);
    AutoShootRectangle.setLineWidth(4);

    referee::Integer Balance_num(kBalance_num_retangle,
                                 referee::Graphic::Operation::kGraphicAdd,
                                 referee::Graphic::Layer::kGraphicLayer0,
                                 referee::Graphic::Color::kColorGreen,
                                 ui_init_data_.balance_num_pos[0], ui_init_data_.balance_num_pos[1],
                                 10, 0);

    pkg->setStraightLineAt(LegHeightline1, 0);
    pkg->setStraightLineAt(LegHeightline2, 1);
    pkg->setStraightLineAt(LegHeightline3, 2);
    pkg->setStraightLineAt(LegHeightline4, 3);
    pkg->setStraightLineAt(LegHeightline5, 4);
    pkg->setRectangleAt(AutoShootRectangle, 5);
    pkg->setIntegerAt(Balance_num, 6);
}

// 基地受击打字符
void Referee::uiAddComponents5(referee::InterGraphicStringPackage *pkg)
{
    std::string base_attack_str = "Base";
    referee::String base_attack_string(kString[0],
                                       referee::Graphic::Operation::kGraphicAdd,
                                       referee::Graphic::Layer::kGraphicLayer0,
                                       referee::Graphic::Color::kColorYellow,
                                       ui_init_data_.base_attack_pos[0], ui_init_data_.base_attack_pos[1],
                                       ui_init_data_.attack_string_size, base_attack_str.size());

    pkg->setStrintg(base_attack_string, base_attack_str);
}

// 英雄受击打字符
void Referee::uiAddComponents6(referee::InterGraphicStringPackage *pkg)
{
    std::string hero_attack_str = "Hero";
    referee::String hero_attack_string(kString[1],
                                       referee::Graphic::Operation::kGraphicAdd,
                                       referee::Graphic::Layer::kGraphicLayer0,
                                       referee::Graphic::Color::kColorYellow,
                                       ui_init_data_.hero_attack_pos[0], ui_init_data_.hero_attack_pos[1],
                                       ui_init_data_.attack_string_size, hero_attack_str.size());

    pkg->setStrintg(hero_attack_string, hero_attack_str);
}

// 哨所受击打字符
void Referee::uiAddComponents7(referee::InterGraphicStringPackage *pkg)
{
    std::string outpost_attack_str = "Outpost";
    referee::String outpost_attack_string(kString[2],
                                          referee::Graphic::Operation::kGraphicAdd,
                                          referee::Graphic::Layer::kGraphicLayer0,
                                          referee::Graphic::Color::kColorYellow,
                                          ui_init_data_.outpost_attack_pos[0], ui_init_data_.outpost_attack_pos[1],
                                          ui_init_data_.attack_string_size, outpost_attack_str.size());

    pkg->setStrintg(outpost_attack_string, outpost_attack_str);
}

// 底盘模式字符
void Referee::uiAddComponents8(referee::InterGraphicStringPackage *pkg)
{
    std::string chassis_mode_str = "CHASSIS:DEAD";
    referee::String chassis_mode_string(kString[3],
                                        referee::Graphic::Operation::kGraphicAdd,
                                        referee::Graphic::Layer::kGraphicLayer0,
                                        referee::Graphic::Color::kColorYellow,
                                        ui_init_data_.chassis_mode_string_pos[0], ui_init_data_.chassis_mode_string_pos[1],
                                        ui_init_data_.mode_string_size, chassis_mode_str.size());

    pkg->setStrintg(chassis_mode_string, chassis_mode_str);
}

// 转向模式字符
void Referee::uiAddComponents9(referee::InterGraphicStringPackage *pkg)
{
    std::string steer_mode_str = "STEER:DEPART";
    referee::String steer_mode_string(kString[4],
                                      referee::Graphic::Operation::kGraphicAdd,
                                      referee::Graphic::Layer::kGraphicLayer0,
                                      referee::Graphic::Color::kColorYellow,
                                      ui_init_data_.steer_mode_string_pos[0], ui_init_data_.steer_mode_string_pos[1],
                                      ui_init_data_.mode_string_size, steer_mode_str.size());

    pkg->setStrintg(steer_mode_string, steer_mode_str);
}

// 平衡标号
void Referee::uiAddComponents11(referee::InterGraphicStringPackage *pkg)
{
    std::string balance_num_str = "Banlance:0";
    referee::String balance_num_string(kString[5],
                                       referee::Graphic::Operation::kGraphicAdd,
                                       referee::Graphic::Layer::kGraphicLayer0,
                                       referee::Graphic::Color::kColorYellow,
                                       ui_init_data_.balance_num_string_pos[0], ui_init_data_.balance_num_string_pos[1],
                                       ui_init_data_.mode_string_size / 2, balance_num_str.size());

    pkg->setStrintg(balance_num_string, balance_num_str);
}

// 平衡标号
void Referee::uiAddComponents12(referee::InterGraphicStringPackage *pkg)
{
    std::string minipc_state_str = "minipc:offline";
    referee::String minipc_state(kString[6],
                                 referee::Graphic::Operation::kGraphicAdd,
                                 referee::Graphic::Layer::kGraphicLayer0,
                                 referee::Graphic::Color::kColorYellow,
                                 ui_init_data_.minipc_states_pos[0], ui_init_data_.minipc_states_pos[1],
                                 ui_init_data_.mode_string_size / 2, minipc_state_str.size());

    pkg->setStrintg(minipc_state, minipc_state_str);
}
// 受击打显示框
void Referee::uiAddComponents10(referee::InterGraphic7Package *pag)
{
    referee::Rectangle base_attack_rectangle(kAttackRetangle[0],
                                             referee::Graphic::Operation::kGraphicAdd,
                                             referee::Graphic::Layer::kGraphicLayer0,
                                             referee::Graphic::Color::kColorPurple);
    base_attack_rectangle.setStartPos(ui_init_data_.base_attack_pos[0] - 10, ui_init_data_.base_attack_pos[1] - 30);
    base_attack_rectangle.setEndPos(ui_init_data_.base_attack_pos[0] + 80, ui_init_data_.base_attack_pos[1] + 10);
    base_attack_rectangle.setLineWidth(8);

    referee::Rectangle hero_attack_rectangle(kAttackRetangle[1],
                                             referee::Graphic::Operation::kGraphicAdd,
                                             referee::Graphic::Layer::kGraphicLayer0,
                                             referee::Graphic::Color::kColorPurple);
    hero_attack_rectangle.setStartPos(ui_init_data_.hero_attack_pos[0] - 10, ui_init_data_.hero_attack_pos[1] - 30);
    hero_attack_rectangle.setEndPos(ui_init_data_.hero_attack_pos[0] + 80, ui_init_data_.hero_attack_pos[1] + 10);
    hero_attack_rectangle.setLineWidth(8);

    referee::Rectangle outpost_attack_rectangle(kAttackRetangle[2],
                                                referee::Graphic::Operation::kGraphicAdd,
                                                referee::Graphic::Layer::kGraphicLayer0,
                                                referee::Graphic::Color::kColorPurple);
    outpost_attack_rectangle.setStartPos(ui_init_data_.outpost_attack_pos[0] - 10, ui_init_data_.outpost_attack_pos[1] - 30);
    outpost_attack_rectangle.setEndPos(ui_init_data_.outpost_attack_pos[0] + 140, ui_init_data_.outpost_attack_pos[1] + 10);
    outpost_attack_rectangle.setLineWidth(8);

    referee::Arc base_attack_arc_front(kAttackArch[0],
                                       referee::Graphic::Operation::kGraphicAdd,
                                       referee::Graphic::Layer::kGraphicLayer0,
                                       referee::Graphic::Color::kColorOrange);

    base_attack_arc_front.setCenterPos(ui_init_data_.base_attack_pos[0], ui_init_data_.base_attack_pos[1]);
    base_attack_arc_front.setRadius(ui_init_data_.attack_arch_radius[0], ui_init_data_.attack_arch_radius[1]);
    base_attack_arc_front.setAng(0, 0);
    base_attack_arc_front.setLineWidth(0);

    referee::Arc base_attack_arc_behind(kAttackArch[1],
                                        referee::Graphic::Operation::kGraphicAdd,
                                        referee::Graphic::Layer::kGraphicLayer0,
                                        referee::Graphic::Color::kColorOrange);
    base_attack_arc_behind.setCenterPos(ui_init_data_.base_attack_pos[0], ui_init_data_.base_attack_pos[1]);
    base_attack_arc_behind.setRadius(ui_init_data_.attack_arch_radius[0], ui_init_data_.attack_arch_radius[1]);
    base_attack_arc_behind.setAng(0, 0);
    base_attack_arc_behind.setLineWidth(0);

    pag->setRectangleAt(base_attack_rectangle, 0);
    pag->setRectangleAt(hero_attack_rectangle, 1);
    pag->setRectangleAt(outpost_attack_rectangle, 2);
    pag->setArcAt(base_attack_arc_front, 3);
    pag->setArcAt(base_attack_arc_behind, 4);
}

// 腿长状态 超电 我方单位受击打状态
void Referee::uiModifyComponents1(referee::InterGraphic7Package *pkg)
{

    float start_X[2];
    start_X[0] = (ui_init_data_.leg_height_line_start[1][0] - ui_init_data_.leg_height_line_start[0][0]) / 7 * 2 + ui_init_data_.leg_height_line_start[0][0];
    start_X[1] = (ui_init_data_.leg_height_line_start[1][0] - ui_init_data_.leg_height_line_start[0][0]) / 7 * 5 + ui_init_data_.leg_height_line_start[0][0];
    float line_Width = (ui_init_data_.leg_height_line_start[1][0] - ui_init_data_.leg_height_line_start[0][0]) / 7 * 2;
    float line_len = ui_init_data_.leg_height_line_start[1][1] - ui_init_data_.leg_height_line_start[0][1];
    float leg_line_leg[2] = {line_len * ui_components_.leg_height[0], line_len * ui_components_.leg_height[1]};

    referee::StraightLine Left_leg_height_curr(kLegHeightCurr[0],
                                               referee::Graphic::Operation::kGraphicModify,
                                               referee::Graphic::Layer::kGraphicLayer0,
                                               referee::Graphic::Color::kColorOrange);
    Left_leg_height_curr.setStartPos(start_X[0], ui_init_data_.leg_height_line_start[0][1]);
    Left_leg_height_curr.setEndPos(start_X[0], ui_init_data_.leg_height_line_start[0][1] + leg_line_leg[0]);
    Left_leg_height_curr.setLineWidth(line_Width);

    referee::StraightLine Right_leg_height_curr(kLegHeightCurr[1],
                                                referee::Graphic::Operation::kGraphicModify,
                                                referee::Graphic::Layer::kGraphicLayer0,
                                                referee::Graphic::Color::kColorGreen);
    Right_leg_height_curr.setStartPos(start_X[1], ui_init_data_.leg_height_line_start[0][1]);
    Right_leg_height_curr.setEndPos(start_X[1], ui_init_data_.leg_height_line_start[0][1] + leg_line_leg[1]);
    Right_leg_height_curr.setLineWidth(line_Width);

    // 画双腿位置
    referee::Circle left_leg_circle(kLegPos[1],
                                    referee::Graphic::Operation::kGraphicModify,
                                    referee::Graphic::Layer::kGraphicLayer0,
                                    referee::Graphic::Color::kColorOrange,
                                    ui_init_data_.armer_detect_center[0] - ui_init_data_.armer_detect_radius * sin(ui_components_.angle + PI / 2),
                                    ui_init_data_.armer_detect_center[1] + ui_init_data_.armer_detect_radius * cos(ui_components_.angle + PI / 2),
                                    6, 3);

    referee::Circle right_leg_circle(kLegPos[2],
                                     referee::Graphic::Operation::kGraphicModify,
                                     referee::Graphic::Layer::kGraphicLayer0,
                                     referee::Graphic::Color::kColorGreen,
                                     ui_init_data_.armer_detect_center[0] + ui_init_data_.armer_detect_radius * sin(ui_components_.angle + PI / 2),
                                     ui_init_data_.armer_detect_center[1] - ui_init_data_.armer_detect_radius * cos(ui_components_.angle + PI / 2),
                                     6, 3);

    // 画连接线
    referee::StraightLine leg_link_line(kLegPos[3],
                                        referee::Graphic::Operation::kGraphicModify,
                                        referee::Graphic::Layer::kGraphicLayer0,
                                        referee::Graphic::Color::kColorCyan);
    leg_link_line.setStartPos(ui_init_data_.armer_detect_center[0] - ui_init_data_.armer_detect_radius * sin(ui_components_.angle + PI / 2),
                              ui_init_data_.armer_detect_center[1] + ui_init_data_.armer_detect_radius * cos(ui_components_.angle + PI / 2));
    leg_link_line.setEndPos(ui_init_data_.armer_detect_center[0] + ui_init_data_.armer_detect_radius * sin(ui_components_.angle + PI / 2),
                            ui_init_data_.armer_detect_center[1] - ui_init_data_.armer_detect_radius * cos(ui_components_.angle + PI / 2));
    leg_link_line.setLineWidth(2);
    // 超电百分比实际值
    referee::StraightLine super_cap_percentage(kSuperCapPercentage,
                                               referee::Graphic::Operation::kGraphicModify,
                                               referee::Graphic::Layer::kGraphicLayer1,
                                               referee::Graphic::Color::kColorPurple);

    super_cap_percentage.setStartPos(ui_init_data_.super_cap_vertical_line[0], ui_init_data_.super_cap_vertical_line[1]);
    super_cap_percentage.setEndPos(uint16_t((float)ui_init_data_.super_cap_vertical_line[0] * (1 - ui_components_.super_cap_percentage / 100.0f) + ((float)ui_init_data_.super_cap_vertical_line[2] * ui_components_.super_cap_percentage / 100.0f)),
                                   ui_init_data_.super_cap_vertical_line[3]);
    super_cap_percentage.setLineWidth(25);

    uint8_t graphics_name[3] = {0};
    uint16_t startpos[2] = {0};
    uint16_t endpos[2] = {0};
    bool is_hurt = false;
    // 受击打更新
    switch (ui_init_data_.refresh_times % 3)
    {
    case 0:
        memcpy(graphics_name, kAttackRetangle[0], 3);
        startpos[0] = ui_init_data_.base_attack_pos[0] - 10;
        startpos[1] = ui_init_data_.base_attack_pos[1] - 30;
        endpos[0] = ui_init_data_.base_attack_pos[0] + 80;
        endpos[1] = ui_init_data_.base_attack_pos[1] + 10;
        is_hurt = ui_components_.is_base_attacked;
        break;
    case 1:
        memcpy(graphics_name, kAttackRetangle[1], 3);
        startpos[0] = ui_init_data_.hero_attack_pos[0] - 10;
        startpos[1] = ui_init_data_.hero_attack_pos[1] - 30;
        endpos[0] = ui_init_data_.hero_attack_pos[0] + 80;
        endpos[1] = ui_init_data_.hero_attack_pos[1] + 10;
        is_hurt = ui_components_.is_hero_attacked;
        break;
    case 2:
        memcpy(graphics_name, kAttackRetangle[2], 3);
        startpos[0] = ui_init_data_.outpost_attack_pos[0] - 10;
        startpos[1] = ui_init_data_.outpost_attack_pos[1] - 30;
        endpos[0] = ui_init_data_.outpost_attack_pos[0] + 140;
        endpos[1] = ui_init_data_.outpost_attack_pos[1] + 10;
        is_hurt = ui_components_.is_outpost_attacked;
        break;
    default:
        break;
    }
    referee::Rectangle attack_update_rectangle(graphics_name,
                                               referee::Graphic::Operation::kGraphicModify,
                                               referee::Graphic::Layer::kGraphicLayer0,
                                               referee::Graphic::Color::kColorPurple);
    attack_update_rectangle.setStartPos(startpos[0], startpos[1]);
    attack_update_rectangle.setEndPos(endpos[0], endpos[1]);
    attack_update_rectangle.setLineWidth(is_hurt ? 8 : 0);

    pkg->setStraightLineAt(Left_leg_height_curr, 0);
    pkg->setStraightLineAt(Right_leg_height_curr, 1);
    pkg->setCircleAt(left_leg_circle, 2);
    pkg->setCircleAt(right_leg_circle, 3);
    pkg->setStraightLineAt(leg_link_line, 4);
    pkg->setStraightLineAt(super_cap_percentage, 5);
    pkg->setRectangleAt(attack_update_rectangle, 6);
}

// 自瞄目标更新 装甲板受击打显示
void Referee::uiModifyComponents2(referee::InterGraphic7Package *pkg)
{
    static int auto_shoot_referch_times = 0;
    static int hurt_update_tick[2] = {0};
    uint16_t x_offset = 68;
    uint16_t y_offset = -158;
    // 自瞄目标框
    referee::Rectangle auto_shoot_target_rect(kAutoShootTargetRectangle,
                                              referee::Graphic::Operation::kGraphicModify,
                                              referee::Graphic::Layer::kGraphicLayer0,
                                              referee::Graphic::Color::kColorWhite);
    auto_shoot_target_rect.setStartPos(ui_components_.target_X - ui_init_data_.auto_shoot_target_Rect_size[0] + x_offset,
                                       ui_components_.target_Y - ui_init_data_.auto_shoot_target_Rect_size[1] + y_offset);
    auto_shoot_target_rect.setEndPos(ui_components_.target_X + ui_init_data_.auto_shoot_target_Rect_size[0] + x_offset,
                                     ui_components_.target_Y + ui_init_data_.auto_shoot_target_Rect_size[1] + y_offset);
    switch (ui_components_.target_state)
    {
    case kTargetStateLost:
        auto_shoot_target_rect.setLineWidth(0);
        break;
    case kTargetStateDetected:
        auto_shoot_target_rect.setLineWidth(1);
        auto_shoot_target_rect.setColor(referee::Graphic::Color::kColorOrange);
        break;
    case kTargetStateAimed:
        auto_shoot_target_rect.setLineWidth(3);
        auto_shoot_target_rect.setColor(referee::Graphic::Color::kColorCyan);
        break;
    default:
        auto_shoot_target_rect.setLineWidth(0);
        break;
    }

    bool is_hurt = false;
    static int hurt_start_ang = 0, hurt_end_ang = 0;

    static float32_t last_gimbal_ang[2] = {0};
    static float32_t last_armer_ang[2] = {0};

    float gimbal_ang = ui_components_.chassis_angle - ui_components_.angle;

    referee::Arc base_attack_arc_front(kAttackArch[0],
                                       referee::Graphic::Operation::kGraphicModify,
                                       referee::Graphic::Layer::kGraphicLayer0,
                                       referee::Graphic::Color::kColorOrange);
    if (ui_components_.is_shooted_armor_updated && ui_components_.shooted_armor == 0)
    {
        hurt_update_tick[0] = 500;
        last_gimbal_ang[0] = gimbal_ang;
        last_armer_ang[0] = ui_components_.angle;
    }
    hurt_start_ang = (-R2D(last_armer_ang[0] - (gimbal_ang - last_gimbal_ang[0])) - 30.0f);
    hurt_end_ang = (-R2D(last_armer_ang[0] - (gimbal_ang - last_gimbal_ang[0])) + 30.0f);
    hurt_start_ang = (int16_t)hello_world::HandleAngleCross0Deg(hurt_start_ang, 180);
    hurt_end_ang = (int16_t)hello_world::HandleAngleCross0Deg(hurt_end_ang, 180);

    base_attack_arc_front.setCenterPos(ui_init_data_.attack_arch[0], ui_init_data_.attack_arch[1]);
    base_attack_arc_front.setRadius(ui_init_data_.attack_arch_radius[0], ui_init_data_.attack_arch_radius[1]);
    base_attack_arc_front.setAng(hurt_start_ang, hurt_end_ang);
    base_attack_arc_front.setLineWidth(hurt_update_tick[0] ? 12 : 0);

    referee::Arc base_attack_arc_behind(kAttackArch[1],
                                        referee::Graphic::Operation::kGraphicModify,
                                        referee::Graphic::Layer::kGraphicLayer0,
                                        referee::Graphic::Color::kColorOrange);
    is_hurt = ui_components_.shooted_armor & ((0x01) << 4);
    if (ui_components_.is_shooted_armor_updated && ui_components_.shooted_armor == 1)
    {
        hurt_update_tick[1] = 500;
        last_gimbal_ang[1] = gimbal_ang;
        last_armer_ang[1] = ui_components_.angle + PI;
    }
    static int ang_debug[2] = {0};
    hurt_start_ang = (-R2D(last_armer_ang[1] - (gimbal_ang - last_gimbal_ang[1])) - 30.0f);
    hurt_end_ang = (-R2D(last_armer_ang[1] - (gimbal_ang - last_gimbal_ang[1])) + 30.0f);
    hurt_start_ang = (int16_t)hello_world::HandleAngleCross0Deg(hurt_start_ang, 180);
    hurt_end_ang = (int16_t)hello_world::HandleAngleCross0Deg(hurt_end_ang, 180);
    ang_debug[0] = hurt_start_ang;
    ang_debug[1] = hurt_end_ang;
    base_attack_arc_behind.setCenterPos(ui_init_data_.attack_arch[0], ui_init_data_.attack_arch[1]);
    base_attack_arc_behind.setRadius(ui_init_data_.attack_arch_radius[0], ui_init_data_.attack_arch_radius[1]);
    base_attack_arc_behind.setAng(hurt_start_ang, hurt_end_ang);
    base_attack_arc_behind.setLineWidth(hurt_update_tick[1] ? 12 : 0);

    int id = ui_components_.balance_num - 1;
    referee::Integer Balance_num(kBalance_num_retangle,
                                 referee::Graphic::Operation::kGraphicModify,
                                 referee::Graphic::Layer::kGraphicLayer0,
                                 referee::Graphic::Color::kColorGreen,
                                 ui_init_data_.balance_num_pos[0], ui_init_data_.balance_num_pos[1],
                                 10, id);

    pkg->setRectangleAt(auto_shoot_target_rect, 0);
    pkg->setArcAt(base_attack_arc_front, 1);
    pkg->setArcAt(base_attack_arc_behind, 2);
    pkg->setIntegerAt(Balance_num, 3);
    if (hurt_update_tick[0])
    {
        hurt_update_tick[0]--;
    }
    if (hurt_update_tick[1])
    {
        hurt_update_tick[1]--;
    }
    auto_shoot_referch_times++;
}

void Referee::uiModifyMode(referee::InterGraphicStringPackage *pag)
{
    std::string ui_string;
    referee::String string_graph;
    switch (ui_init_data_.refresh_times % 8)
    {
    case 0:
    case 1:
    case 2:
        switch (ui_components_.chassis_mode)
        {
        case CHASSIS_DEAD:
            ui_string = "CHASSIS:DEAD";
            break;
        case CHASSIS_MATURE:
            ui_string = "CHASSIS:MATURE";
            break;
        case CHASSIS_CONFIRM:
            ui_string = "CHASSIS:CONFIRM";
            break;
        case CHASSIS_INIT:
            ui_string = "CHASSIS:INIT";
            break;
        case CHASSIS_FLY:
            ui_string = "CHASSIS:FLY";
            break;
        case CHASSIS_RECOVERY:
            ui_string = "CHASSIS:RECOVERY";
            break;
        case CHASSIS_JUMP:
            ui_string = "CHASSIS:JUMP";
            break;
        default:
            break;
        }
        string_graph = referee::String(kString[3],
                                       referee::Graphic::Operation::kGraphicModify,
                                       referee::Graphic::Layer::kGraphicLayer0,
                                       referee::Graphic::Color::kColorYellow,
                                       ui_init_data_.chassis_mode_string_pos[0], ui_init_data_.chassis_mode_string_pos[1],
                                       ui_init_data_.mode_string_size, ui_string.size());
        pag->setStrintg(string_graph, ui_string);
        break;
    case 3:
    case 4:
    case 5:
        switch (ui_components_.steer_mode)
        {
        case STEER_DEPART:
            ui_string = "STEER:DEPART";
            break;
        case STEER_DEFENSE:
            ui_string = "STEER:DEFENSE";
            break;
        case STEER_GYRO:
            ui_string = "STEER:GYRO";
            break;
        case STEER_MOVE:
            ui_string = "STEER:MOVE";
            break;
        default:
            break;
        }
        string_graph = referee::String(kString[4],
                                       referee::Graphic::Operation::kGraphicModify,
                                       referee::Graphic::Layer::kGraphicLayer0,
                                       referee::Graphic::Color::kColorYellow,
                                       ui_init_data_.steer_mode_string_pos[0], ui_init_data_.steer_mode_string_pos[1],
                                       ui_init_data_.mode_string_size, ui_string.size());

        pag->setStrintg(string_graph, ui_string);
        break;
    case 6:
        switch (ui_components_.balance_num)
        {
        case 3:
            ui_string = "Banlance:3";
            break;
        case 4:
            ui_string = "Banlance:4";
            break;
        case 5:
            ui_string = "Banlance:5";
            break;
        default:
            break;
        }
        string_graph = referee::String(kString[5],
                                       referee::Graphic::Operation::kGraphicModify,
                                       referee::Graphic::Layer::kGraphicLayer0,
                                       referee::Graphic::Color::kColorYellow,
                                       ui_init_data_.balance_num_string_pos[0], ui_init_data_.balance_num_string_pos[1],
                                       ui_init_data_.mode_string_size / 2, ui_string.size());

        pag->setStrintg(string_graph, ui_string);
        break;
    case 7:
        switch (ui_components_.mini_pc_ok)
        {
        case 0:
            ui_string = "minipc:offline";
            break;
        case 1:
            ui_string = "minipc:online";
        default:
            break;
        }
        string_graph = referee::String(kString[6],
                                       referee::Graphic::Operation::kGraphicModify,
                                       referee::Graphic::Layer::kGraphicLayer0,
                                       referee::Graphic::Color::kColorYellow,
                                       ui_init_data_.balance_num_string_pos[0], ui_init_data_.balance_num_string_pos[1],
                                       ui_init_data_.mode_string_size / 2, ui_string.size());

        pag->setStrintg(string_graph, ui_string);
        break;
    default:
        break;
    }
}

bool Referee::drawUi(UiComponents ui_components, uint8_t *tx_data, size_t *tx_len, bool is_enabled)
{
    ui_components_ = ui_components;
    referee::InterGraphic7Package inter_graphic7_pkg;
    referee::InterGraphicStringPackage inter_graphic_string_pkg;
    referee::RfrId sender_id = ui_components.robot_id;
    inter_graphic7_pkg.setSenderId(sender_id);
    inter_graphic_string_pkg.setSenderId(sender_id);
    int package_type = Graphics;
    if (!is_enabled)
    {

        ui_init_data_.refresh_times = 0;
        referee::InterGraphicDeletePackage delete_pkg;
        delete_pkg.setDeleteOperation(referee::DeleteOperation::kDeleteAll);
        delete_pkg.setSenderId(sender_id);
        encoder_.encodeFrame(&delete_pkg, tx_data, tx_len);
        return true;
    }

    switch (ui_init_data_.refresh_times / 2)
    {
    case 0:
        uiAddComponents1(&inter_graphic7_pkg);
        break;
    case 1:
        uiAddComponents2(&inter_graphic7_pkg);
        break;
    case 2:
        uiAddComponents3(&inter_graphic7_pkg);
        break;
    case 3:
        uiAddComponents4(&inter_graphic7_pkg);
        break;
    case 4:
        uiAddComponents5(&inter_graphic_string_pkg);
        package_type = String;
        break;
    case 5:
        uiAddComponents6(&inter_graphic_string_pkg);
        package_type = String;
        break;
    case 6:
        uiAddComponents7(&inter_graphic_string_pkg);
        package_type = String;
        break;
    case 7:
        uiAddComponents8(&inter_graphic_string_pkg);
        package_type = String;
        break;
    case 8:
        uiAddComponents9(&inter_graphic_string_pkg);
        package_type = String;
        break;
    case 9:
        uiAddComponents10(&inter_graphic7_pkg);
        break;
    case 10:
        uiAddComponents11(&inter_graphic_string_pkg);
        package_type = String;
        break;
    case 11:
        uiAddComponents12(&inter_graphic_string_pkg);
        package_type = String;
        break;
    default:
        switch (ui_init_data_.refresh_times % 5)
        {
        case 1:
            uiModifyComponents1(&inter_graphic7_pkg);
            break;
        case 0:
        case 2:
        case 4:
            uiModifyComponents2(&inter_graphic7_pkg);
            break;
        case 3:
            uiModifyMode(&inter_graphic_string_pkg);
            package_type = String;
            break;
        default:
            break;
        }
        break;
    }

    switch (package_type)
    {
    case Graphics:
        if (encoder_.encodeFrame(&inter_graphic7_pkg, tx_data, tx_len))
        {
            ui_init_data_.refresh_times++;
            return true;
        }
        break;
    case String:
        if (encoder_.encodeFrame(&inter_graphic_string_pkg, tx_data, tx_len))
        {
            ui_init_data_.refresh_times++;
            return true;
        }
        break;
    default:
        break;
    }

    return false;
}
/* Private function definitions ----------------------------------------------*/
