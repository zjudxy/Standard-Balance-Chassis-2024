/**
 * @file      referee_custom_define.hpp
 * @brief
 * @author    ZhouShichan (zsc19823382069@163.com)
 * @version   1.0.0
 * @date      2023-10-30
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
 * @par last edit time  2023-10-30
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HWCOMPONETS_DEVICES_REFEREE_CUSTOM_DEFINE_H_
#define HWCOMPONETS_DEVICES_REFEREE_CUSTOM_DEFINE_H_

/* Includes ------------------------------------------------------------------*/
#include "referee_protocol.hpp"
#include "referee_rx_template.hpp"
namespace hello_world
{
namespace devices
{
namespace referee
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef uint8_t Crc8_t;
typedef uint16_t Crc16_t;

typedef enum {
    kFramePartLengthHeader = sizeof(FrameHeader),
    kFramePartLengthCmdId = sizeof(FrameCmdId),
    kFramePartLengthInterFrameHeader = sizeof(InterFrameHeader),
    kFramePartLengthTail = sizeof(Crc16_t),
    kFramePartLengthExceptData = kFramePartLengthHeader + kFramePartLengthCmdId + kFramePartLengthTail,
} FramePartLength;

typedef enum {
    kFramePartIndexHeader = 0,
    kFramePartIndexHeaderSof = 0,
    kFramePartIndexHeaderDataLength = sizeof(FrameHeader::sof),
    kFramePartIndexHeaderSequenceNumber = kFramePartIndexHeaderDataLength + sizeof(FrameHeader::data_length),
    kFramePartIndexHeaderCrc = kFramePartIndexHeaderSequenceNumber + sizeof(FrameHeader::sequence_number),
    kFramePartIndexCmdId = sizeof(FrameHeader),
    kFramePartIndexData = kFramePartIndexCmdId + sizeof(FrameCmdId),
} FramePartIndex;

typedef enum : uint16_t {
    kInterAmongRobotsIdRadarWarning = 0x0200,
} InterAmongRobotsId;

typedef enum : uint16_t {
    kInterAmongRobotsDataLengthRadarWarning = 1u,
} InterAmongRobotsDataLength;

typedef struct __REFEREE_PACKED {
    uint8_t is_warning;
} InterRadarWarning;

class RxPackageInterRadarWarning : public RxPackage<InterRadarWarning>
{
   public:
    RxPackageInterRadarWarning() : RxPackage<InterRadarWarning>(kFrameCmdIdInterAmongRobots, 0){};
    inline bool decode(const uint8_t *p_frame_cmd_data)
    {
        bool decode_result = false;
        FrameCmdId cmd_id = (FrameCmdId)(p_frame_cmd_data[0] | p_frame_cmd_data[1] << 8);
        if (cmd_id == command_id_) {
            InterFrameHeader inter_frame_header;
            memcpy(&inter_frame_header, p_frame_cmd_data + 2, sizeof(inter_frame_header));
            if (inter_frame_header.sub_content_id == kInterAmongRobotsIdRadarWarning) {
                if (inter_frame_header.sender_id == kRobotIdRedRadar || inter_frame_header.sender_id == kRobotIdBlueRadar) {
                    memcpy((InterRadarWarning *)this, p_frame_cmd_data + 2 + sizeof(inter_frame_header), sizeof(InterRadarWarning));
                    decode_result = true;
                }
            }
        }

        return decode_result;
    };
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace referee

}  // namespace devices

}  // namespace hello_world

#endif /* HWCOMPONETS_DEVICES_REFEREE_CUSTOM_DEFINE_H_ */
