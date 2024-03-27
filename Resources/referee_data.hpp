

/** 
 * @file      referee_data.hpp
 * @brief     裁判系统相关数据
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RESOUCRES_REFEREE_DATA_HPP_
#define __RESOUCRES_REFEREE_DATA_HPP_

/* Includes ------------------------------------------------------------------*/
#include "referee.hpp"
#include "rfr_official_pkgs.hpp"
namespace referee=hello_world::referee;
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
enum
{
    Graphics = 0,
    String = 1,
};





struct UiComponents{
    referee::RfrId robot_id;
    float angle; //云台与底盘的夹角，0度时灯条朝后
    uint16_t super_cap_percentage; //超级电容百分比,0-100
    uint8_t auto_shoot_target_id;//自瞄锁定目标id,未锁定请置大于8
    uint8_t auto_shoot_target_list;//自瞄目标列表
    uint8_t shooted_armor;//被击打装甲板，1为被击打
    bool is_shooted_armor_updated;
    float leg_height[2];//腿部高度百分比,0-1
    bool is_base_attacked;//基地是否被攻击
    bool is_hero_attacked;//英雄是否被攻击
    bool is_outpost_attacked;//前哨站是否被攻击
    uint8_t chassis_mode;//底盘模式
    uint8_t steer_mode;//转向模式
};
struct UiInitData{
    uint16_t middle_vertical_line[4];//中线垂直线 x1,y1,x2,y2
    uint16_t front_7m_height; //7m准星高度
    uint16_t front_7m_half_length; //7m准星半长
    uint16_t front_5m_height; //5m准星高度
    uint16_t front_5m_half_length; //5m准星半长
    uint16_t front_3m_height; //3m准星高度
    uint16_t front_3m_half_length; //3m准星半长

    uint16_t auto_shoot_edge[4];//自瞄范围x1,y1,x2,y2
    uint16_t auto_shoot_target_num[8][3];//自瞄目标x,y,字体大小

    uint16_t super_cap_vertical_line[4];//超级电容框 x1,y1,x2,y2
    uint16_t super_cap_horizontal_line_half_length;//超级电容实际值半长

    uint16_t armer_detect_radius ;//装甲板半径,显示腿部位置
    uint16_t armer_detect_center[2];//装甲板中心 x,y

    uint16_t leg_height_line_start[2][2];//腿部高度显示对照线对角 x1,y1 x2,y2
    
    uint16_t attack_string_size;//受击字符大小
    uint16_t base_attack_pos[2];     //基地受攻击字符位置 x,y
    uint16_t hero_attack_pos[2];     //英雄受攻击字符位置 x,y
    uint16_t outpost_attack_pos[2];  //前哨站受攻击字符位置 x,y


    uint16_t mode_string_size;//模式字符大小
    uint16_t chassis_mode_string_pos[2];     //底盘状态字符位置 x,y
    uint16_t steer_mode_string_pos[2];     //转向状态字符位置 x,y



    uint8_t refresh_times;//刷新计数，确保为static，初始化置0
    uint8_t hurt_refresh_interval;//刷新间隔
};

extern UiInitData default_ui_init_data;
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
class Referee
{
    public:
        Referee(UiInitData ui_init_data);
        bool drawUi(UiComponents ui_components,uint8_t* tx_data,size_t* tx_len,bool is_enabled = true);
    private:
        void uiAddComponents1(referee::InterGraphic7Package *pag);
        void uiAddComponents2(referee::InterGraphic7Package *pag);
        void uiAddComponents3(referee::InterGraphic7Package *pag);
        void uiAddComponents4(referee::InterGraphic7Package *pag);
        void uiAddComponents5(referee::InterGraphicStringPackage *pag);
        void uiAddComponents6(referee::InterGraphicStringPackage *pag);
        void uiAddComponents7(referee::InterGraphicStringPackage *pag);
        void uiAddComponents8(referee::InterGraphicStringPackage *pag);
        void uiAddComponents9(referee::InterGraphicStringPackage *pag);
        void uiAddComponents10(referee::InterGraphic7Package *pag);




        void uiModifyComponents1(referee::InterGraphic7Package *pag);
        void uiModifyComponents2(referee::InterGraphic7Package *pag);
        void uiModifyComponents3(referee::InterGraphic7Package *pag);
        void uiModifyMode(referee::InterGraphicStringPackage *pag);


        referee::RfrDecoder decoder_;
        referee::RfrEncoder encoder_;
        UiInitData ui_init_data_;
        UiComponents ui_components_;
};


#endif /* __RESOUCRES_REFEREE_DATA_HPP_ */

