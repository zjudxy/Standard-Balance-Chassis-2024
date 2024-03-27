

/**
 * @file      referee_ui.hpp
 * @brief
 * @author    ZhouShichan (zsc19823382069@163.com)
 * @version   1.0.0
 * @date      2023-11-07
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
 * @par last edit time  2023-11-07
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HWCOMPONETS_DEVICES_REFEREE_UI_H_
#define HWCOMPONETS_DEVICES_REFEREE_UI_H_

/* Includes ------------------------------------------------------------------*/
#include <cstring>
#include <queue>

#include "referee_protocol.hpp"
namespace hello_world
{
namespace devices
{
namespace referee
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
const uint32_t kMaxPositionX = 1920;
const uint32_t kMaxPositionY = 1080;
const int16_t kIllegalUiElementProperty = -1;

typedef enum {
    kUiRefeshPriorityLow,
    kUiRefeshPriorityHigh,
} UiRefeshPriority;

class UiElement
{
    friend UiFrame;

   protected:
    UiFrame *patent_ = nullptr;
    UiRefeshPriority refresh_priority_;

   public:
    void setPatent(UiFrame *patent) { patent_ = patent; };
    void setRefeshPriority(UiRefeshPriority refresh_priority) { refresh_priority_ = refresh_priority; };
    virtual void encode(uint8_t *p_frame_data) = 0;
};

class UiFrame
{
   protected:
    const static FrameCmdId frame_cmd_id_ = kFrameCmdIdInterAmongRobots;
    static RobotId robot_id_;

   public:
    virtual bool encodeFrame(uint8_t *p_frame_data) = 0;
};

class UiFrameDelete : public UiFrame
{
   private:
    const static UiSubCmdId sub_cmd_id_ = kUiSubCmdIdDeleteLayer;

   public:
    bool encodeFrame(uint8_t *p_frame_data);
};

class UiFrameGraphicPack1 : public UiFrame
{
   private:
    const static UiSubCmdId sub_cmd_id_ = kUiSubCmdIdDrawGraphic1;

   public:
    bool encodeFrame(uint8_t *p_frame_data);
};
class UiFrameGraphicPack2 : public UiFrame
{
   private:
    const static UiSubCmdId sub_cmd_id_ = kUiSubCmdIdDrawGraphic2;

   public:
    bool encodeFrame(uint8_t *p_frame_data);
};
class UiFrameGraphicPack5 : public UiFrame
{
   private:
    const static UiSubCmdId sub_cmd_id_ = kUiSubCmdIdDrawGraphic5;

   public:
    bool encodeFrame(uint8_t *p_frame_data);
};
class UiFrameGraphicPack7 : public UiFrame
{
   private:
    const static UiSubCmdId sub_cmd_id_ = kUiSubCmdIdDrawGraphic7;

   public:
    bool encodeFrame(uint8_t *p_frame_data);
};
class UiFrameGraphicCharacter : public UiFrame
{
   private:
    const static UiSubCmdId sub_cmd_id_ = kUiSubCmdIdDrawCharacter;

   public:
    bool encodeFrame(uint8_t *p_frame_data);
};

class UiCore
{
   private:
    std::queue<UiFrame &> frame_queue_;

   public:
    void addFrame(UiFrame &frame);
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace referee

}  // namespace devices

}  // namespace hello_world

#endif /* HWCOMPONETS_DEVICES_REFEREE_UI_H_ */
