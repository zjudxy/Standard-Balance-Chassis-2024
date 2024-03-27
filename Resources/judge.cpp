/**
 *******************************************************************************
 * @file      : chassis_detect_task.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0     yyyy-mm-dd        dxy           <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
 *******************************************************************************
 * @file      :judge.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0     yyyy-mm-dd        dxy           <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "judge.hpp"
#include "referee.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/


// bool RobotReferee::drawUi(UiComponents ui_components, uint8_t* tx_data, size_t* tx_len)
// {
//   static uint8_t init_finish_flag = 0;
//   referee::InterGraphic5Package inter_graphic5_pkg;
//   referee::refereeId sender_id = robot_chassis->js_package_.robot_performance->getData().robot_id;
//   inter_graphic5_pkg.setSenderId(sender_id);

//   referee::Rectangle rectangle(kAutoShootRectangle,
//                            referee::Graphic::Operation::kGraphicAdd,
//                            referee::Graphic::Layer::kGraphicLayer0,
//                            referee::Graphic::Color::kColorCyan);
//   rectangle.setStartPos(ui_components.AutoShootEdge[0], ui_components.AutoShootEdge[1]);
//   rectangle.setEndPos(ui_components.AutoShootEdge[2], ui_components.AutoShootEdge[3]);
//   rectangle.setLineWidth(2);

//   referee::StraightLine vertical_line(kMiddleVerticalLine,
//                                   referee::Graphic::Operation::kGraphicAdd,
//                                   referee::Graphic::Layer::kGraphicLayer0,
//                                   referee::Graphic::Color::kColorCyan);
//   vertical_line.setStartPos(960, 0);
//   vertical_line.setEndPos(960, 1080);
//   vertical_line.setLineWidth(2);

//   referee::StraightLine horizotal_line1(kMiddleHorizital1,
//                                     referee::Graphic::Operation::kGraphicAdd,
//                                     referee::Graphic::Layer::kGraphicLayer0,
//                                     referee::Graphic::Color::kColorCyan);
//   horizotal_line1.setStartPos(0, 540);
//   horizotal_line1.setEndPos(1920, 540);
//   horizotal_line1.setLineWidth(2);

//   if (!init_finish_flag) {
//     rectangle.setOperation(referee::Graphic::Operation::kGraphicAdd);
//     vertical_line.setOperation(referee::Graphic::Operation::kGraphicAdd);
//     horizotal_line1.setOperation(referee::Graphic::Operation::kGraphicAdd);
//     init_finish_flag++;
//   }

//   inter_graphic5_pkg.setRectangleAt(rectangle, 0);
//   inter_graphic5_pkg.setStraightLineAt(vertical_line, 1);
//   inter_graphic5_pkg.setStraightLineAt(horizotal_line1, 2);
//   inter_graphic5_pkg.setRectangleAt(rectangle, 3);
//   inter_graphic5_pkg.setRectangleAt(rectangle, 4);

//   if (init_finish_flag > 10) {
//     init_finish_flag = 0;
//   }

//   static bool ok_flag = 0;
//   ok_flag = encoder_ptr->encodeFrame(&inter_graphic5_pkg, tx_data, tx_len);
//   return ok_flag;
// }
// #include "chassis_ui_task.hpp"

// #include "chassis.hpp"
// #include "chassis_referee.hpp"
// #include "referee.hpp"
// #include "usart.h"
// extern RobotChassis* robot_chassis;
// namespace rfr = hello_world::referee;
// rfr::RfrEncoder* rfr_encoder = nullptr;
// UiDrawer* ui_drawer_ptr = nullptr;
// rfr::InterGraphic7Package* inter_graphic7_pkg;
// rfr::RfrId sender_id;

// const uint8_t kAutoShootRectangle[3] = {0x001, 0, 0x001};
// const uint8_t kAutoShootCircle[8][3] = {{0x001, 0, 0x002},
//                                         {0x001, 0, 0x003},
//                                         {0x001, 0, 0x004},
//                                         {0x001, 0, 0x005},
//                                         {0x001, 0, 0x006},
//                                         {0x001, 0, 0x007},
//                                         {0x001, 0, 0x008},
//                                         {0x001, 0, 0x009}};

// const uint8_t kSuperCapVerticalLine[3] = {0x002, 0, 0x001};
// const uint8_t kSuperCapPercentage[3] = {0x002, 0, 0x001};
// const uint8_t kMiddleVerticalLine[3] = {0x03, 0, 0x001};

// const uint8_t kMiddleHorizital1[3] = {0x03, 0, 0x002};
// const uint8_t kMiddleHorizital2[3] = {0x03, 0, 0x003};
// const uint8_t kMiddleHorizital3[3] = {0x03, 0, 0x004};

// const uint8_t kArmoredPlate[4][3] = {{0x004, 0, 0x001},
//                                      {0x004, 0, 0x002},
//                                      {0x004, 0, 0x003},
//                                      {0x004, 0, 0x004}};

// UiInitData default_ui_init_data = {
//     .middle_vertical_line = {960, 500, 960, 580},
//     .front_7m_height = 560,
//     .front_7m_half_length = 10,
//     .front_5m_height = 540,
//     .front_5m_half_length = 7,
//     .front_3m_height = 520,
//     .front_3m_half_length = 4,

//     .auto_shoot_edge = {600, 600, 1400, 900},
//     .auto_shoot_target_position = {{80, 1000, 20}, {140, 1000, 20}, {200, 1000, 20}, {260, 1000, 20}, {320, 1000, 20}, {380, 1000, 20}, {440, 1000, 20}, {500, 1000, 20}},

//     .super_cap_vertical_line = {200, 400, 200, 600},
//     .super_cap_horizontal_line_half_length = 4,

//     .armored_plate_half_length = 10,
//     .armored_plate_r = 20,
//     .light_line_half_length = 8,
//     .light_line_r = 24,

//     .refresh_times = 0,

// };

// static void Rotation(int16_t line[4], int16_t center[2], float angle, int16_t rotated_line[4]);

// void UiTask(void)
// {
//   if (robot_chassis->ui_drawer_->refresh_tick_ == 0) {
//     robot_chassis->ui_drawer_->UiStartDrawPart1();
//   } else if (robot_chassis->ui_drawer_->refresh_tick_ == 1) {
//     robot_chassis->ui_drawer_->UiStartDrawPart2();
//   } else if (robot_chassis->ui_drawer_->refresh_tick_ == 2) {
//     robot_chassis->ui_drawer_->UiStartDrawPart3();
//   } else {
//     switch (robot_chassis->ui_drawer_->refresh_tick_ % 2) {
//       case 0:
//         robot_chassis->ui_drawer_->UiModifyDrawPart1();
//         break;
//       case 1:
//         robot_chassis->ui_drawer_->UiModifyDrawPart2();
//         break;
//     }
//   }
// }

// void UiInit(void)
// {
//   rfr_encoder = new rfr::RfrEncoder();
//   ui_drawer_ptr = new UiDrawer();
//   inter_graphic7_pkg = new rfr::InterGraphic7Package();
//   sender_id = robot_chassis->js_package_.robot_performance->getData().robot_id;
//   inter_graphic7_pkg->setSenderId(sender_id);
// }

// void UiDrawer ::UiStartDrawPart1()
// {
//   /*绘制Ui中垂线*/
//   rfr::StraightLine middle_vertical_line(kMiddleVerticalLine, rfr::Graphic::Operation::kGraphicAdd,
//                                          rfr::Graphic::Layer::kGraphicLayer0, rfr::Graphic::Color::kColorWhite,
//                                          default_ui_init_data.middle_vertical_line[0], default_ui_init_data.middle_vertical_line[1],
//                                          default_ui_init_data.middle_vertical_line[2], default_ui_init_data.middle_vertical_line[3]);
//   /*绘制7mm准星*/
//   rfr::StraightLine middle_horizotal1(kMiddleHorizital1, rfr::Graphic::Operation::kGraphicAdd,
//                                       rfr::Graphic::Layer::kGraphicLayer0, rfr::Graphic::Color::kColorWhite,
//                                       960 - default_ui_init_data.front_7m_half_length, default_ui_init_data.front_7m_height,
//                                       960 + default_ui_init_data.front_7m_half_length, default_ui_init_data.front_7m_height);
//   /*绘制5mm准星*/
//   rfr::StraightLine middle_horizotal2(kMiddleHorizital2, rfr::Graphic::Operation::kGraphicAdd,
//                                       rfr::Graphic::Layer::kGraphicLayer0, rfr::Graphic::Color::kColorWhite,
//                                       960 - default_ui_init_data.front_5m_half_length, default_ui_init_data.front_5m_height,
//                                       960 + default_ui_init_data.front_5m_half_length, default_ui_init_data.front_5m_height);
//   /*绘制3mm准星*/
//   rfr::StraightLine middle_horizotal3(kMiddleHorizital3, rfr::Graphic::Operation::kGraphicAdd,
//                                       rfr::Graphic::Layer::kGraphicLayer0, rfr::Graphic::Color::kColorWhite,
//                                       960 - default_ui_init_data.front_3m_half_length, default_ui_init_data.front_3m_height,
//                                       960 + default_ui_init_data.front_3m_half_length, default_ui_init_data.front_3m_height);
//   /*绘制超电垂线*/
//   rfr::StraightLine super_cap_vertical_line(kSuperCapVerticalLine, rfr::Graphic::Operation::kGraphicAdd,
//                                             rfr::Graphic::Layer::kGraphicLayer0, rfr::Graphic::Color::kColorWhite,
//                                             default_ui_init_data.super_cap_vertical_line[0], default_ui_init_data.super_cap_vertical_line[1],
//                                             default_ui_init_data.super_cap_vertical_line[2], default_ui_init_data.super_cap_vertical_line[3]);
//   /*绘制超电百分比*/
//   rfr::StraightLine super_cap_percentage(kSuperCapPercentage, rfr::Graphic::Operation::kGraphicAdd,
//                                          rfr::Graphic::Layer::kGraphicLayer1,
//                                          rfr::Graphic::Color::kColorYellow);
//   uint16_t height = (default_ui_init_data.super_cap_vertical_line[3] - default_ui_init_data.super_cap_vertical_line[1]) *
//                         this->ui_components_.super_cap_percentage / 100 +
//                     default_ui_init_data.super_cap_vertical_line[1];
//   super_cap_percentage.setStartPos(default_ui_init_data.super_cap_vertical_line[0] - default_ui_init_data.super_cap_horizontal_line_half_length,
//                                    height);
//   super_cap_percentage.setEndPos(default_ui_init_data.super_cap_vertical_line[0] + default_ui_init_data.super_cap_horizontal_line_half_length,
//                                  height);
//   super_cap_percentage.setLineWidth(3);
//   /*绘制自瞄圆*/
//   rfr::Circle auto_shoot_target1(kAutoShootCircle[0], rfr::Graphic::Operation::kGraphicAdd,
//                                  rfr::Graphic::Layer::kGraphicLayer0, rfr::Graphic::Color::kColorWhite,
//                                  default_ui_init_data.auto_shoot_target_position[0][0], default_ui_init_data.auto_shoot_target_position[0][1],
//                                  default_ui_init_data.auto_shoot_target_position[0][2], 5);
//   if (this->ui_components_.auto_shoot_target_list & (1 << 0)) {
//     auto_shoot_target1.setColor(rfr::Graphic::Color::kColorPink);
//   }
//   if (this->ui_components_.auto_shoot_target_id == 1) {
//     auto_shoot_target1.setColor(rfr::Graphic::Color::kColorGreen);
//   }

//   inter_graphic7_pkg->setStraightLineAt(middle_vertical_line, 0);
//   inter_graphic7_pkg->setStraightLineAt(middle_horizotal1, 1);
//   inter_graphic7_pkg->setStraightLineAt(middle_horizotal2, 2);
//   inter_graphic7_pkg->setStraightLineAt(middle_horizotal3, 3);
//   inter_graphic7_pkg->setStraightLineAt(super_cap_vertical_line, 4);
//   inter_graphic7_pkg->setCircleAt(auto_shoot_target1, 5);
//   inter_graphic7_pkg->setStraightLineAt(super_cap_percentage, 6);
// }

// void UiDrawer ::UiStartDrawPart2()
// {
//   /*绘制自瞄圈位置*/
//   for (size_t id = 1; id < 8; id++) {
//     rfr::Circle auto_shoot_target(kAutoShootCircle[id], rfr::Graphic::Operation::kGraphicModify, rfr::Graphic::Layer::kGraphicLayer0,
//                                   rfr::Graphic::Color::kColorWhite,
//                                   default_ui_init_data.auto_shoot_target_position[id][0], default_ui_init_data.auto_shoot_target_position[id][1],
//                                   default_ui_init_data.auto_shoot_target_position[id][2], 5);
//     if (ui_components_.auto_shoot_target_list & (1 << id)) {
//       auto_shoot_target.setColor(rfr::Graphic::Color::kColorPink);
//     }
//     if (ui_components_.auto_shoot_target_id == id) {
//       auto_shoot_target.setColor(rfr::Graphic::Color::kColorGreen);
//     }
//     inter_graphic7_pkg->setCircleAt(auto_shoot_target, id - 1);
//   }
// }

// void UiDrawer ::UiStartDrawPart3()
// {
//   int16_t rotated_line[4], line[4], center[2] = {960, 540};
//   line[0] = 960 - default_ui_init_data.armored_plate_half_length;
//   line[1] = 540 + default_ui_init_data.armored_plate_r;
//   line[2] = 960 + default_ui_init_data.armored_plate_half_length;
//   line[3] = 540 + default_ui_init_data.armored_plate_r;
//   for (size_t id = 0; id < 4; id++) {
//     rfr::StraightLine armored_plate(kArmoredPlate[id], rfr::Graphic::Operation::kGraphicAdd,
//                                     rfr::Graphic::Layer::kGraphicLayer0, rfr::Graphic::Color::kColorBlack);
//     Rotation(line, center, ui_components_.angle + PI / 2.0 * id, rotated_line);
//     armored_plate.setStartPos(rotated_line[0], rotated_line[1]);
//     armored_plate.setEndPos(rotated_line[2], rotated_line[3]);
//     armored_plate.setLineWidth(1);
//     if (ui_components_.shooted_armor == id + 1) {
//       armored_plate.setColor(rfr::Graphic::Color::kColorPink);
//       armored_plate.setLineWidth(3);
//     }
//     inter_graphic7_pkg->setStraightLineAt(armored_plate, id);
//   }

//   line[0] = 960 - default_ui_init_data.light_line_half_length;
//   line[1] = 540 - default_ui_init_data.light_line_r;
//   line[2] = 960 + default_ui_init_data.light_line_half_length;
//   line[3] = 540 - default_ui_init_data.light_line_r;
//   rfr::StraightLine light_line(kMiddleVerticalLine, rfr::Graphic::Operation::kGraphicAdd,
//                                rfr::Graphic::Layer::kGraphicLayer0, rfr::Graphic::Color::kColorWhite);
//   Rotation(line, center, ui_components_.angle, rotated_line);
//   light_line.setStartPos(rotated_line[0], rotated_line[1]);
//   light_line.setEndPos(rotated_line[2], rotated_line[3]);
//   light_line.setLineWidth(2);
//   inter_graphic7_pkg->setStraightLineAt(light_line, 4);
//   /*绘制自瞄框位置*/
//   rfr::Rectangle auto_shoot_edge(kAutoShootRectangle, rfr::Graphic::Operation::kGraphicAdd,
//                                  rfr::Graphic::Layer::kGraphicLayer0, rfr::Graphic::Color::kColorWhite,
//                                  default_ui_init_data.auto_shoot_edge[0], default_ui_init_data.auto_shoot_edge[1],
//                                  default_ui_init_data.auto_shoot_edge[2], default_ui_init_data.auto_shoot_edge[3], 2);

//   inter_graphic7_pkg->setRectangleAt(auto_shoot_edge, 5);

//   rfr::Circle empety(kAutoShootCircle[0], rfr::Graphic::Operation::kGraphicNone,
//                      rfr::Graphic::Layer::kGraphicLayer0, rfr::Graphic::Color::kColorWhite);
//   inter_graphic7_pkg->setCircleAt(empety, 6);
// }

// void UiDrawer::UiModifyDrawPart1()
// {
//   int16_t rotated_line[4], line[4], center[2] = {960, 540};
//   line[0] = 960 - default_ui_init_data.armored_plate_half_length;
//   line[1] = 540 + default_ui_init_data.armored_plate_r;
//   line[2] = 960 + default_ui_init_data.armored_plate_half_length;
//   line[3] = 540 + default_ui_init_data.armored_plate_r;
//   for (size_t id = 0; id < 4; id++) {
//     rfr::StraightLine armored_plate(kArmoredPlate[id], rfr::Graphic::Operation::kGraphicModify,
//                                     rfr::Graphic::Layer::kGraphicLayer0,
//                                     rfr::Graphic::Color::kColorBlack);
//     Rotation(line, center, ui_components_.angle + PI / 2.0 * id, rotated_line);
//     armored_plate.setStartPos(rotated_line[0], rotated_line[1]);
//     armored_plate.setEndPos(rotated_line[2], rotated_line[3]);
//     armored_plate.setLineWidth(1);
//     if (ui_components_.shooted_armor == id + 1) {
//       armored_plate.setColor(rfr::Graphic::Color::kColorPink);
//       armored_plate.setLineWidth(3);
//     }
//     inter_graphic7_pkg->setStraightLineAt(armored_plate, id);
//   }

//   line[0] = 960 - default_ui_init_data.light_line_half_length;
//   line[1] = 540 - default_ui_init_data.light_line_r;
//   line[2] = 960 + default_ui_init_data.light_line_half_length;
//   line[3] = 540 - default_ui_init_data.light_line_r;
//   rfr::StraightLine light_line(kMiddleVerticalLine, rfr::Graphic::Operation::kGraphicModify,
//                                rfr::Graphic::Layer::kGraphicLayer0, rfr::Graphic::Color::kColorWhite);
//   Rotation(line, center, ui_components_.angle, rotated_line);
//   light_line.setStartPos(rotated_line[0], rotated_line[1]);
//   light_line.setEndPos(rotated_line[2], rotated_line[3]);
//   light_line.setLineWidth(2);

//   rfr::StraightLine super_cap_percentage(kSuperCapPercentage, rfr::Graphic::Operation::kGraphicModify,
//                                          rfr::Graphic::Layer::kGraphicLayer1, rfr::Graphic::Color::kColorYellow);
//   uint16_t height = (default_ui_init_data.super_cap_vertical_line[3] - default_ui_init_data.super_cap_vertical_line[1]) *
//                         ui_components_.super_cap_percentage / 100 +
//                     default_ui_init_data.super_cap_vertical_line[1];
//   super_cap_percentage.setStartPos(default_ui_init_data.super_cap_vertical_line[0] - default_ui_init_data.super_cap_horizontal_line_half_length,
//                                    height);
//   super_cap_percentage.setEndPos(default_ui_init_data.super_cap_vertical_line[0] + default_ui_init_data.super_cap_horizontal_line_half_length,
//                                  height);
//   super_cap_percentage.setLineWidth(3);

//   rfr::Circle auto_shoot_target1(kAutoShootCircle[0], rfr::Graphic::Operation::kGraphicModify,
//                                  rfr::Graphic::Layer::kGraphicLayer0, rfr::Graphic::Color::kColorWhite,
//                                  default_ui_init_data.auto_shoot_target_position[0][0], default_ui_init_data.auto_shoot_target_position[0][1],
//                                  default_ui_init_data.auto_shoot_target_position[0][2], 5);

//   if (ui_components_.auto_shoot_target_list & (1 << 0)) {
//     auto_shoot_target1.setColor(rfr::Graphic::Color::kColorPink);
//   }
//   if (ui_components_.auto_shoot_target_id == 1) {
//     auto_shoot_target1.setColor(rfr::Graphic::Color::kColorGreen);
//   }

//   inter_graphic7_pkg->setStraightLineAt(light_line, 4);
//   inter_graphic7_pkg->setCircleAt(auto_shoot_target1, 5);
//   inter_graphic7_pkg->setStraightLineAt(super_cap_percentage, 6);
// }

// void UiDrawer::UiModifyDrawPart2()
// {
//   for (size_t id = 1; id < 8; id++) {
//     rfr::Circle auto_shoot_target(kAutoShootCircle[id], rfr::Graphic::Operation::kGraphicModify,
//                                   rfr::Graphic::Layer::kGraphicLayer0, rfr::Graphic::Color::kColorWhite,
//                                   default_ui_init_data.auto_shoot_target_position[id][0], default_ui_init_data.auto_shoot_target_position[id][1],
//                                   default_ui_init_data.auto_shoot_target_position[id][2], 5);
//     if (ui_components_.auto_shoot_target_list & (1 << id)) {
//       auto_shoot_target.setColor(rfr::Graphic::Color::kColorPink);
//     }
//     if (ui_components_.auto_shoot_target_id == id) {
//       auto_shoot_target.setColor(rfr::Graphic::Color::kColorGreen);
//     }
//     inter_graphic7_pkg->setCircleAt(auto_shoot_target, id - 1);
//   }
// }

// static void Rotation(int16_t line[4], int16_t center[2], float angle, int16_t rotated_line[4])
// {
//   float x1 = line[0] - center[0];
//   float y1 = line[1] - center[1];
//   float x2 = line[2] - center[0];
//   float y2 = line[3] - center[1];
//   rotated_line[0] = x1 * cos(angle) - y1 * sin(angle) + center[0];
//   rotated_line[1] = x1 * sin(angle) + y1 * cos(angle) + center[1];
//   rotated_line[2] = x2 * cos(angle) - y2 * sin(angle) + center[0];
//   rotated_line[3] = x2 * sin(angle) + y2 * cos(angle) + center[1];
// }

// void SetChassisUiDrawer()
// {
//   robot_chassis->ui_drawer_ = ui_drawer_ptr;
// }

// bool UiDrawer::UiEncoder(uint8_t* tx_data, size_t* tx_len)
// {
//   if (rfr_encoder->encodeFrame(inter_graphic7_pkg, tx_data, tx_len)) {
//     return true;
//   }
//   return false;
// }

// const uint8_t UiDrawer ::UiFrameLen()
// {
//   return frame_length_;
// }

// UiDrawer ::UiDrawer()
// {
//   refresh_tick_ = 0;
//   frame_length_ = rfr::kRefereeMaxFrameLength;
//   ui_components_ = {
//       .robot_id = 0,
//       .angle = 0,
//       .super_cap_percentage = 0,
//       .auto_shoot_target_id = 0,
//       .auto_shoot_target_list = 0,
//       .shooted_armor = 0,
//   };
// }