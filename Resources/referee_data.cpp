

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
const uint8_t kAutoShootNum[8][3] = {{'1', '0', '2'}, {'1', '0', '3'}, {'1', '0', '4'}, {'1', '0', '5'}, {'1', '0', '6'}, {'1', '0', '7'}, {'1', '0', '8'}, {'1', '0', '9'}};

const uint8_t kSuperCapVerticalLine[3] = {0x02, 0, 0x01};
const uint8_t kSuperCapPercentage[3] = {0x02, 0, 0x02};

const uint8_t kMiddleVerticalLine[3] = {0x03, 0, 0x01};
const uint8_t kMiddleHorizital1[3] = {0x03, 0, 0x02};
const uint8_t kMiddleHorizital2[3] = {0x03, 0, 0x03};
const uint8_t kMiddleHorizital3[3] = {0x03, 0, 0x04};

const uint8_t kLegPos[4][3] = {{0x04, 0, 0x01}, {0x04, 0, 0x02}, {0x04, 0, 0x03}, {0x04, 0, 0x04}}; // 腿部位置图形 1为左橙 2为右绿 3为连接线

const uint8_t kLegHeightline[5][3] = {0x04, 0, 0x05, 0x04, 0, 0x06, 0x04, 0, 0x07, 0x04, 0, 0x08, 0x04, 0, 0x09}; // 腿部高度基准线
const uint8_t kForwardLine[3] = {0x05, 0, 0x01};                                                                  // 枪口固定线
const uint8_t kLegHeightCurr[2][3] = {0x05, 0, 0x02, 0x05, 0, 0x03};                                              // 腿部高度线
const uint8_t kAttackRetangle[3][3] = {{0x05, 0, 0x04}, {0x05, 0, 0x05}, {0x05, 0, 0x06}};                        // 受击打图形

const uint8_t kString[5][3] = {0x06, 0, 0x01, 0x06, 0, 0x02, 0x06, 0, 0x03, 0x06, 0, 0x04, 0x06, 0, 0x05};        // 字符串

/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
UiInitData default_ui_init_data = {
    .middle_vertical_line = {960, 540, 960, 300},
    .front_7m_height = 500,
    .front_7m_half_length = 15,
    .front_5m_height = 440,
    .front_5m_half_length = 10,
    .front_3m_height = 360,
    .front_3m_half_length = 5,

    .auto_shoot_edge = {600, 600, 1400, 900},
    .auto_shoot_target_num = {{1500, 850, 30}, {1540, 850, 30}, {1580, 850, 30}, {1620, 850, 30}, {1660, 850, 30}, {1700, 850, 30}, {1740, 850, 30}, {1780, 850, 30}},

    .super_cap_vertical_line = {820, 100, 1100, 100},
    .super_cap_horizontal_line_half_length = 4,

    .armer_detect_radius = 80,
    .armer_detect_center = {660, 120},

    .leg_height_line_start = {1150, 50, 1150 + 140, 50 + 150},

    .attack_string_size = 20,
    .base_attack_pos = {600, 780},
    .hero_attack_pos = {700, 780},
    .outpost_attack_pos = {800, 780},

    .mode_string_size = 30,
    .chassis_mode_string_pos = {60, 700},
    .steer_mode_string_pos = {60, 600},

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
//超电 准星线 自瞄目标1
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

    // 自瞄目标1的
    referee::Integer auto_shoot_target1(kAutoShootNum[0],
                                        referee::Graphic::Operation::kGraphicAdd,
                                        referee::Graphic::Layer::kGraphicLayer0,
                                        referee::Graphic::Color::kColorWhite,
                                        ui_init_data_.auto_shoot_target_num[0][0], ui_init_data_.auto_shoot_target_num[0][1],
                                        ui_init_data_.auto_shoot_target_num[0][2], 1);

    if (ui_components_.auto_shoot_target_list & (1 << 0))
    {
        auto_shoot_target1.setColor(referee::Graphic::Color::kColorPink);
    }
    if (ui_components_.auto_shoot_target_id == 1)
    {
        auto_shoot_target1.setColor(referee::Graphic::Color::kColorGreen);
    }
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
    pkg->setIntegerAt(auto_shoot_target1, 5);
    pkg->setStraightLineAt(super_cap_percentage, 6);
}

//自瞄目标圈2-8
void Referee::uiAddComponents2(referee::InterGraphic7Package *pkg)
{
    // 自瞄目标圆圈2-8
    for (size_t id = 1; id < 8; id++)
    {
        referee::Integer target_num(kAutoShootNum[id],
                                    referee::Graphic::Operation::kGraphicAdd,
                                    referee::Graphic::Layer::kGraphicLayer0,
                                    referee::Graphic::Color::kColorWhite, ui_init_data_.auto_shoot_target_num[id][0], ui_init_data_.auto_shoot_target_num[id][1],
                                    ui_init_data_.auto_shoot_target_num[id][2], (uint32_t)(id + 1));

        // referee::Circle auto_shoot_target(kAutoShootNum[id],
        // referee::Graphic::Operation::kGraphicAdd,
        // referee::Graphic::Layer::kGraphicLayer0,
        // referee::Graphic::Color::kColorWhite,
        // ui_init_data_.auto_shoot_target_num[id][0],ui_init_data_.auto_shoot_target_num[id][1],
        // ui_init_data_.auto_shoot_target_num[id][2],5);
        // auto_shoot_target.setCenterPos(ui_init_data_.auto_shoot_target_num[id][0],ui_init_data_.auto_shoot_target_num[id][1]);
        // auto_shoot_target.setRadius(ui_init_data_.auto_shoot_target_num[id][2]);
        // auto_shoot_target.setLineWidth(5);
        if (ui_components_.auto_shoot_target_list & (1 << id))
        {
            target_num.setColor(referee::Graphic::Color::kColorPink);
        }
        if (ui_components_.auto_shoot_target_id == id)
        {
            target_num.setColor(referee::Graphic::Color::kColorGreen);
        }
        pkg->setIntegerAt(target_num, id - 1);
    }
}

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


//身体位置和腿长线
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

    pkg->setStraightLineAt(LegHeightline1, 0);
    pkg->setStraightLineAt(LegHeightline2, 1);
    pkg->setStraightLineAt(LegHeightline3, 2);
    pkg->setStraightLineAt(LegHeightline4, 3);
    pkg->setStraightLineAt(LegHeightline5, 4);
}

//基地受击打字符
void Referee::uiAddComponents5(referee::InterGraphicStringPackage *pkg)
{
    std::string base_attack_str = "Base";
    referee::String base_attack_string(kString[0],
                                       referee::Graphic::Operation::kGraphicAdd,
                                       referee::Graphic::Layer::kGraphicLayer0,
                                       referee::Graphic::Color::kColorYellow,
                                       ui_init_data_.base_attack_pos[0], ui_init_data_.base_attack_pos[1],
                                       ui_init_data_.attack_string_size,base_attack_str.size());

    pkg->setStrintg(base_attack_string,base_attack_str);
}

//英雄受击打字符
void Referee::uiAddComponents6(referee::InterGraphicStringPackage *pkg)
{
    std::string hero_attack_str = "Hero";
    referee::String hero_attack_string(kString[1],
                                       referee::Graphic::Operation::kGraphicAdd,
                                       referee::Graphic::Layer::kGraphicLayer0,
                                       referee::Graphic::Color::kColorYellow,
                                       ui_init_data_.hero_attack_pos[0], ui_init_data_.hero_attack_pos[1],
                                       ui_init_data_.attack_string_size,hero_attack_str.size());

    pkg->setStrintg(hero_attack_string,hero_attack_str);
}

//哨所受击打字符
void Referee::uiAddComponents7(referee::InterGraphicStringPackage *pkg)
{
    std::string outpost_attack_str = "Outpost";
    referee::String outpost_attack_string(kString[2],
                                          referee::Graphic::Operation::kGraphicAdd,
                                          referee::Graphic::Layer::kGraphicLayer0,
                                          referee::Graphic::Color::kColorYellow,
                                          ui_init_data_.outpost_attack_pos[0], ui_init_data_.outpost_attack_pos[1],
                                          ui_init_data_.attack_string_size,outpost_attack_str.size());

    pkg->setStrintg(outpost_attack_string,outpost_attack_str);
}

//底盘模式字符
void Referee::uiAddComponents8(referee::InterGraphicStringPackage *pkg)
{
    std::string chassis_mode_str = "CHASSIS:DEAD";
    referee::String chassis_mode_string(kString[3],
                                        referee::Graphic::Operation::kGraphicAdd,
                                        referee::Graphic::Layer::kGraphicLayer0,
                                        referee::Graphic::Color::kColorYellow,
                                        ui_init_data_.chassis_mode_string_pos[0], ui_init_data_.chassis_mode_string_pos[1],
                                        ui_init_data_.mode_string_size,chassis_mode_str.size());

    pkg->setStrintg(chassis_mode_string,chassis_mode_str);
}

//转向模式字符
void Referee::uiAddComponents9(referee::InterGraphicStringPackage *pkg)
{
    std::string steer_mode_str = "STEER:DEPART";
    referee::String steer_mode_string(kString[4],
                                      referee::Graphic::Operation::kGraphicAdd,
                                      referee::Graphic::Layer::kGraphicLayer0,
                                      referee::Graphic::Color::kColorYellow,
                                      ui_init_data_.steer_mode_string_pos[0], ui_init_data_.steer_mode_string_pos[1],
                                      ui_init_data_.mode_string_size,steer_mode_str.size());

    pkg->setStrintg(steer_mode_string,steer_mode_str);
}

//受击打显示框
void Referee::uiAddComponents10(referee::InterGraphic7Package *pag){
    referee::Rectangle base_attack_rectangle(kAttackRetangle[0],
                       referee::Graphic::Operation::kGraphicAdd,
                       referee::Graphic::Layer::kGraphicLayer0,
                       referee::Graphic::Color::kColorPurple);
    base_attack_rectangle.setStartPos(ui_init_data_.base_attack_pos[0]-10,ui_init_data_.base_attack_pos[1]-30);
    base_attack_rectangle.setEndPos(ui_init_data_.base_attack_pos[0]+80,ui_init_data_.base_attack_pos[1]+10);
    base_attack_rectangle.setLineWidth(8);

    referee::Rectangle hero_attack_rectangle(kAttackRetangle[1],
                       referee::Graphic::Operation::kGraphicAdd,
                       referee::Graphic::Layer::kGraphicLayer0,
                       referee::Graphic::Color::kColorPurple);
    hero_attack_rectangle.setStartPos(ui_init_data_.hero_attack_pos[0]-10,ui_init_data_.hero_attack_pos[1]-30);
    hero_attack_rectangle.setEndPos(ui_init_data_.hero_attack_pos[0]+80,ui_init_data_.hero_attack_pos[1]+10);
    hero_attack_rectangle.setLineWidth(8);

    referee::Rectangle outpost_attack_rectangle(kAttackRetangle[2],
                       referee::Graphic::Operation::kGraphicAdd,
                       referee::Graphic::Layer::kGraphicLayer0,
                       referee::Graphic::Color::kColorPurple);
    outpost_attack_rectangle.setStartPos(ui_init_data_.outpost_attack_pos[0]-10,ui_init_data_.outpost_attack_pos[1]-30);
    outpost_attack_rectangle.setEndPos(ui_init_data_.outpost_attack_pos[0]+140,ui_init_data_.outpost_attack_pos[1]+10);
    outpost_attack_rectangle.setLineWidth(8);



    pag->setRectangleAt(base_attack_rectangle,0);
    pag->setRectangleAt(hero_attack_rectangle,1);
    pag->setRectangleAt(outpost_attack_rectangle,2);


}

//腿长状态 超电 我方单位受击打状态
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
//受击打更新
    switch (ui_init_data_.refresh_times%3)
    {
    case 0:
        memcpy(graphics_name, kAttackRetangle[0],3);
        startpos[0]=ui_init_data_.base_attack_pos[0]-10;
        startpos[1]=ui_init_data_.base_attack_pos[1]-30;
        endpos[0]=ui_init_data_.base_attack_pos[0]+80;
        endpos[1]=ui_init_data_.base_attack_pos[1]+10;
        is_hurt = ui_components_.is_base_attacked;
        break;
    case 1:
        memcpy(graphics_name, kAttackRetangle[1],3);
        startpos[0]=ui_init_data_.hero_attack_pos[0]-10;
        startpos[1]=ui_init_data_.hero_attack_pos[1]-30;
        endpos[0]=ui_init_data_.hero_attack_pos[0]+80;
        endpos[1]=ui_init_data_.hero_attack_pos[1]+10;
        is_hurt = ui_components_.is_hero_attacked;
        break;
    case 2:
        memcpy(graphics_name, kAttackRetangle[2],3);
        startpos[0]=ui_init_data_.outpost_attack_pos[0]-10;
        startpos[1]=ui_init_data_.outpost_attack_pos[1]-30;
        endpos[0]=ui_init_data_.outpost_attack_pos[0]+140;
        endpos[1]=ui_init_data_.outpost_attack_pos[1]+10;
        is_hurt = ui_components_.is_outpost_attacked;
        break;
    default:
        break;
    }
    referee::Rectangle attack_update_rectangle(graphics_name,
                                               referee::Graphic::Operation::kGraphicModify,
                                               referee::Graphic::Layer::kGraphicLayer0,
                                               referee::Graphic::Color::kColorPurple);
    attack_update_rectangle.setStartPos(startpos[0],startpos[1]);
    attack_update_rectangle.setEndPos(endpos[0],endpos[1]);
    attack_update_rectangle.setLineWidth(is_hurt?8:0);


    pkg->setStraightLineAt(Left_leg_height_curr, 0);
    pkg->setStraightLineAt(Right_leg_height_curr, 1);
    pkg->setCircleAt(left_leg_circle, 2);
    pkg->setCircleAt(right_leg_circle, 3);
    pkg->setStraightLineAt(leg_link_line, 4);
    pkg->setStraightLineAt(super_cap_percentage, 5);
    pkg->setRectangleAt(attack_update_rectangle, 6);




}
//自瞄目标更新
void Referee::uiModifyComponents2(referee::InterGraphic7Package *pkg)
{
    // 自瞄目标圆圈1-7
    for (size_t id = 0; id < 7; id++)
    {
        referee::Integer target_num(kAutoShootNum[id],
                                    referee::Graphic::Operation::kGraphicModify,
                                    referee::Graphic::Layer::kGraphicLayer0,
                                    referee::Graphic::Color::kColorWhite,
                                    ui_init_data_.auto_shoot_target_num[id][0],
                                    ui_init_data_.auto_shoot_target_num[id][1],
                                    ui_init_data_.auto_shoot_target_num[id][2], (uint32_t)(id + 1));


        
        // auto_shoot_target.setCenterPos(ui_init_data_.auto_shoot_target_num[id][0],ui_init_data_.auto_shoot_target_num[id][1]);
        // auto_shoot_target.setRadius(ui_init_data_.auto_shoot_target_num[id][2]);
        // auto_shoot_target.setLineWidth(5);
        if (ui_components_.auto_shoot_target_list & (1 << id))
        {
            target_num.setColor(referee::Graphic::Color::kColorPink);
        }
        if (ui_components_.auto_shoot_target_id == id)
        {
            target_num.setColor(referee::Graphic::Color::kColorGreen);
        }
        pkg->setIntegerAt(target_num, id);
    }
}

void Referee::uiModifyMode(referee::InterGraphicStringPackage *pag){
    if(ui_init_data_.refresh_times%8<4){//更新底盘模式
        std::string chassis_mode_str = "CHASSIS:DEAD";

        switch (ui_components_.chassis_mode)
        {
        case CHASSIS_DEAD:
            chassis_mode_str="CHASSIS:DEAD";
            break;
        case CHASSIS_MATURE:
            chassis_mode_str="CHASSIS:MATURE";
            break;
        case CHASSIS_CONFIRM:
            chassis_mode_str="CHASSIS:CONFIRM";
            break;
        case CHASSIS_INIT:
            chassis_mode_str="CHASSIS:INIT";
            break;
        case CHASSIS_FLY:
            chassis_mode_str="CHASSIS:FLY";
            break;
        case CHASSIS_RECOVERY:
            chassis_mode_str="CHASSIS:RECOVERY";
            break;
        case CHASSIS_JUMP:
            chassis_mode_str="CHASSIS:JUMP";
            break;
        default:
            break;
        }
        referee::String chassis_mode_string(kString[3],
                                        referee::Graphic::Operation::kGraphicModify,
                                        referee::Graphic::Layer::kGraphicLayer0,
                                        referee::Graphic::Color::kColorYellow,
                                        ui_init_data_.chassis_mode_string_pos[0], ui_init_data_.chassis_mode_string_pos[1],
                                        ui_init_data_.mode_string_size,chassis_mode_str.size());
        pag->setStrintg(chassis_mode_string,chassis_mode_str);
        
    }else{
        std::string steer_mode_str = "STEER:DEPART";
        switch (ui_components_.steer_mode)
        {
        case STEER_DEPART:
            steer_mode_str="STEER:DEPART";
            break;
        case STEER_DEFENSE:
            steer_mode_str="STEER:DEFENSE";
            break;
        case STEER_GYRO:
            steer_mode_str="STEER:GYRO";
            break;
        case STEER_MOVE:
            steer_mode_str="STEER:MOVE";
            break;
        default:
            break;
        }
        referee::String steer_mode_string(kString[4],
                                      referee::Graphic::Operation::kGraphicModify,
                                      referee::Graphic::Layer::kGraphicLayer0,
                                      referee::Graphic::Color::kColorYellow,
                                      ui_init_data_.steer_mode_string_pos[0], ui_init_data_.steer_mode_string_pos[1],
                                      ui_init_data_.mode_string_size,steer_mode_str.size());
        
        pag->setStrintg(steer_mode_string,steer_mode_str);
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
        delete_pkg.setDelOptration(referee::DeleteOperation::kDeleteAll);
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
    default:
        switch (ui_init_data_.refresh_times % 4)
        {
        case 0:
        case 2:
            uiModifyComponents1(&inter_graphic7_pkg);
            break;
        case 1:
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
