

/**
 * @file      referee.cpp
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
/* Includes ------------------------------------------------------------------*/
#include "referee.hpp"

#include "arm_math.h"
#include "referee_crc.hpp"
#include "referee_size_check.hpp"
namespace hello_world
{
namespace devices
{
namespace referee
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions -----------------------------------------------*/
bool Referee::decodeFrame(const uint8_t *p_frame, const size_t frame_length)
{
  bool process_result = false;
  for (size_t i = 0; i < frame_length; i++) {
    process_result = processByte(p_frame[i]);
    if (process_result) {
      break;
    }
  }
  float32_t src = 1, dst = -1;
  arm_abs_f32(&src, &dst, 1);
  return process_result;
};

void Referee::restartDecodeFrame(size_t frame_length)
{
  memset(&rx_info_.frame_raw, 0, frame_length);
  rx_info_.rx_status = kRxStatusWaitingHeaderSof;
  rx_info_.frame_data_index = 0;
  rx_info_.frame_expect_length = 0;
};

bool Referee::decodeRxPackage(const uint8_t *p_frame_cmd_data)
{
  bool process_result = false;
  for (size_t i = 0; i < kRefereeRxPackageNumber; i++) {
    RxPackageInfo *p_info = kRefereeRxPackagePtrs[i];
    process_result = p_info->decode(p_frame_cmd_data);
    if (process_result) {
      last_updated_frame_index_ = i;
      break;
    }
  }
  return process_result;
};

bool Referee::processByte(const uint8_t byte)
{
  bool process_result = false;
  switch (rx_info_.rx_status) {
    case kRxStatusWaitingHeaderSof:
      if (byte == kRefereeFrameHeaderSof) {
        rx_info_.frame_raw[rx_info_.frame_data_index++];
        rx_info_.rx_status = kRxStatusHeader;
      } else {
        restartDecodeFrame(1);
      }
      break;
    case kRxStatusHeader:
      rx_info_.frame_raw[rx_info_.frame_data_index++];
      if (rx_info_.frame_data_index == kFramePartIndexCmdId) {
        if (VerifyCrc8CheckSum(rx_info_.frame_raw, kFramePartLengthHeader)) {
          rx_info_.frame_expect_length =
              rx_info_.header.frame_header.data_length +
              kFramePartLengthExceptData;
          rx_info_.rx_status = kRxStatusCmdDataTail;
        } else {
          restartDecodeFrame(rx_info_.frame_data_index);
        }
      }
      break;
    case kRxStatusCmdDataTail:
      rx_info_.frame_raw[rx_info_.frame_data_index++];
      if (rx_info_.frame_data_index == rx_info_.frame_expect_length) {
        if (VerifyCrc16CheckSum(rx_info_.frame_raw,
                                rx_info_.frame_expect_length)) {
          process_result =
              decodeRxPackage(&rx_info_.frame_raw[kFramePartIndexCmdId]);
        }
        restartDecodeFrame(kRefereeMaxBuffer);
      }
      break;
    default:
      restartDecodeFrame();
      break;
  }
  return process_result;
};

bool Referee::encodeSetCrc(uint8_t *p_frame, size_t frame_length)
{
  if (SetEndCrc8CheckSum(p_frame, kFramePartLengthHeader)) {
    if (SetEndCrc16CheckSum(p_frame, frame_length)) {
      return true;
    }
  }
  return false;
};
/* Private function definitions -----------------------------------------------*/

}  // namespace referee

}  // namespace devices

}  // namespace hello_world
